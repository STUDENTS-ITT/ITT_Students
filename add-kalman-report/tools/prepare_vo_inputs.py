import csv, os, json, math
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "add-kalman-report", "data", "vo_hk03")
# Подача видеоодометрии в БИНС. Используем keyframes только (~1.67 Hz).
# ВАЖНО: интерполяция до 50 Hz катастрофически ухудшает точность (RMSE 291 м vs 33.6 м)!
USE_KEYFRAMES_ONLY = True  # True: gps_key.dat (~1.67 Hz), False: интерполяция 50 Hz
VO_FEED_HZ = 50.0  # Используется только если USE_KEYFRAMES_ONLY = False
# Шаг быстрее этого порога — ложное соответствие (крейсерская скорость здесь ~9 м/с).
# Такой прирост не накапливается: дальше идут только относительные шаги от последней годной точки.
MAX_STEP_MPS = 15.0
# Реинтеграция шагов через body frame с compass для коррекции дрейфа VO yaw
REROTATE_STEPS = True
os.makedirs(OUT, exist_ok=True)

def load_csv(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8", errors="replace") as f:
        return list(csv.DictReader(f))

def col(rows, name, dtype=float):
    return np.array([dtype(r[name]) for r in rows])

# ---------- timestamps ----------
cam = load_csv(r"camera\timestamps.csv")
cam_t = col(cam, "stamp_sec") + col(cam, "stamp_nsec") * 1e-9   # sensor time
imu = load_csv(r"imu\dji_osdk_ros_imu.csv")
imu_t = col(imu, "header.stamp.sec") + col(imu, "header.stamp.nanosec") * 1e-9
t0 = imu_t[0]
cam_t -= t0
imu_t -= t0
print(f"t0(IMU first)={t0:.9f}")
print(f"time ranges rel t0: camera {cam_t[0]:.3f}..{cam_t[-1]:.3f}  imu {imu_t[0]:.3f}..{imu_t[-1]:.3f}")

dt_imu = np.diff(imu_t)
print(f"IMU dt median {np.median(dt_imu)*1000:.3f} ms  mean rate {1/np.mean(dt_imu):.1f} Hz  min {dt_imu.min()*1000:.3f} max {dt_imu.max()*1000:.3f}")

# ---------- VO keyframes ----------
vo = load_csv(r"results_HKairport03\homography\visual_odometry\FAST-FREAK\trajectory.csv")
vo = [r for r in vo if r["Status"].startswith(("ok", "origin"))]
vframe = np.array([int(r["Frame"]) - 1 for r in vo])
vx = np.array([float(r["X"]) for r in vo])
vy = np.array([float(r["Y"]) for r in vo])
vz = np.array([float(r["Altitude"]) for r in vo])
vh = np.array([float(r["HeadingDeg"]) for r in vo])
vinliers = np.array([int(r["Inliers"]) if r["Inliers"] else 0 for r in vo])
vstatus = np.array([r["Status"] for r in vo])
vt = cam_t[vframe]
print(f"VO keyframes {len(vt)}  t {vt[0]:.3f}..{vt[-1]:.3f}  rate {len(vt)/(vt[-1]-vt[0]):.2f} Hz")
print(f"VO alt {vz.min():.2f}..{vz.max():.2f}  heading {vh.min():.1f}..{vh.max():.1f}")
print(f"VO inliers {vinliers.min()}..{vinliers.max()}  median {np.median(vinliers):.0f}")

# unwrap VO heading (only for diagnostics; it is NOT a compass)
vhu = np.rad2deg(np.unwrap(np.deg2rad(vh)))

# ---------- true body heading from attitude (computed early for reintegration) ----------
att = load_csv(r"other\dji_osdk_ros_attitude.csv")
att_t = col(att, "header.stamp.sec") + col(att, "header.stamp.nanosec") * 1e-9 - t0
qw = col(att, "quaternion.w"); qx = col(att, "quaternion.x")
qy = col(att, "quaternion.y"); qz = col(att, "quaternion.z")
yaw_enu = np.unwrap(np.arctan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz)))
# compass (deg, clockwise from north) = 90 - ENU yaw; unwrap for interpolation
compass = np.rad2deg(np.unwrap((np.pi / 2 - yaw_enu) % (2 * np.pi)))
print(f"attitude compass at t0: {compass[0]:.3f} deg")

# ---------- reject VO steps that are faster than the vehicle can fly ----------
def reject_fast_steps(t, x, y, z, h, hu, frames, inliers, status, cap):
    pos = np.array([x[0], y[0], z[0]], dtype=float)
    ot, ox, oy, oz = [t[0]], [pos[0]], [pos[1]], [pos[2]]
    oh, ohu, ofr = [h[0]], [hu[0]], [frames[0]]
    oinl, ost = [inliers[0]], [status[0]]
    dropped = 0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        dx, dy, dz = x[i] - x[i - 1], y[i] - y[i - 1], z[i] - z[i - 1]
        spd = math.hypot(dx, dy) / max(dt, 1e-3)
        if dt <= 0 or spd > cap:
            dropped += 1
            continue
        pos += (dx, dy, dz)
        ot.append(t[i]); ox.append(pos[0]); oy.append(pos[1]); oz.append(pos[2])
        oh.append(h[i]); ohu.append(hu[i]); ofr.append(frames[i])
        oinl.append(inliers[i]); ost.append(status[i])
    return (np.asarray(ot), np.asarray(ox), np.asarray(oy), np.asarray(oz),
            np.asarray(oh), np.asarray(ohu), np.asarray(ofr),
            np.asarray(oinl), np.asarray(ost), dropped)

def reintegrate_body_steps(t, x, y, h_vo_deg, att_t, compass):
    """Реинтеграция через body frame: VO step -> body -> world (compass).
    d = (x[i]-x[i-1], y[i]-y[i-1]) в VO world (X=East, Y=North)
    yv = pi/2 - rad(h_vo_deg[i-1]) = VO yaw предыдущей строки
    body = Rot(-yv) @ d
    yc = pi/2 - rad(interp(0.5*(t[i-1]+t[i]), att_t, compass))
    p[i] = p[i-1] + Rot(yc) @ body
    Возвращает новые (x_new, y_new).
    """
    n = len(t)
    px, py = np.zeros(n), np.zeros(n)
    px[0], py[0] = x[0], y[0]
    for i in range(1, n):
        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]
        # VO yaw previous row
        yv = np.pi / 2.0 - np.radians(h_vo_deg[i - 1])
        cv, sv = np.cos(yv), np.sin(yv)
        # to body
        bx = cv * dx + sv * dy
        by = -sv * dx + cv * dy
        # compass at step midpoint
        t_mid = 0.5 * (t[i - 1] + t[i])
        yc = np.pi / 2.0 - np.radians(np.interp(t_mid, att_t, compass))
        cc, sc = np.cos(yc), np.sin(yc)
        # back to world
        px[i] = px[i - 1] + (cc * bx - sc * by)
        py[i] = py[i - 1] + (sc * bx + cc * by)
    return px, py

n_before = len(vt)
vt, vx, vy, vz, vh, vhu, vframe, vinliers, vstatus, n_drop = reject_fast_steps(
    vt, vx, vy, vz, vh, vhu, vframe, vinliers, vstatus, MAX_STEP_MPS)
print(f"rejected {n_drop}/{n_before - 1} steps faster than {MAX_STEP_MPS:.0f} m/s, kept {len(vt)}")

# ---------- body-step reintegration (before Kabsch alignment) ----------
if REROTATE_STEPS:
    vx, vy = reintegrate_body_steps(vt, vx, vy, vh, att_t, compass)

# ---------- RTK truth (local ENU around first fix): prefix alignment and evaluation only ----------
rtk = load_csv(r"gps\dji_osdk_ros_rtk_position.csv")
rtk_t = col(rtk, "header.stamp.sec") + col(rtk, "header.stamp.nanosec") * 1e-9 - t0
lat = col(rtk, "latitude"); lon = col(rtk, "longitude"); alt = col(rtk, "altitude")
lat0, lon0, alt0 = lat[0], lon[0], alt[0]
a, e2 = 6378137.0, 6.69437999014e-3
s = math.sin(math.radians(lat0))
rn = a / math.sqrt(1 - e2 * s * s)
rm = rn * (1 - e2) / (1 - e2 * s * s)
E = np.radians(lon - lon0) * rn * math.cos(math.radians(lat0))
N = np.radians(lat - lat0) * rm
print(f"RTK t {rtk_t[0]:.3f}..{rtk_t[-1]:.3f}  origin {lat0:.9f},{lon0:.9f},{alt0:.3f}")

# ---------- prefix alignment VO -> ENU (first 105 s, like run_all) ----------
m = vt <= 105.0
A = np.c_[vx[m], vy[m]]
Bt = np.c_[np.interp(vt[m], rtk_t, E), np.interp(vt[m], rtk_t, N)]
cA = A.mean(0); cB = Bt.mean(0)
u, _, vt_svd = np.linalg.svd((A - cA).T @ (Bt - cB))
d = np.eye(2)
if np.linalg.det(u @ vt_svd) < 0:
    d[1, 1] = -1
R = (u @ d @ vt_svd).T
tr = cB - R @ cA
yaw_align = math.degrees(math.atan2(R[1, 0], R[0, 0]))
print(f"prefix align: yaw {yaw_align:.2f} deg, translation {tr}")

enu = (R @ np.c_[vx, vy].T).T + tr
alt_v = vz + alt0

# ---------- Функции для сглаживания скоростей ----------
def smooth_deriv(t, y, win_s=1.2):
    d = np.gradient(y, t)
    k = max(3, int(round(win_s / np.median(np.diff(t)))) | 1)
    ker = np.ones(k) / k
    return np.convolve(np.pad(d, (k//2, k//2), mode="edge"), ker, mode="valid")

def compute_step_velocities(t, pos, smooth_win=3):
    """Скорости из реинтегрированных шагов с легким сглаживанием."""
    n = len(t)
    v = np.zeros(n)
    # Forward difference для каждого шага
    for i in range(n - 1):
        dt = t[i + 1] - t[i]
        if dt > 1e-6:
            v[i] = (pos[i + 1] - pos[i]) / dt
    v[-1] = v[-2] if n > 1 else 0.0
    # 3-point сглаживание
    if smooth_win >= 3 and n >= smooth_win:
        k = smooth_win
        ker = np.ones(k) / k
        v = np.convolve(np.pad(v, (k//2, k//2), mode="edge"), ker, mode="valid")
    return v

# ---------- gps.dat / angle.dat: выбор режима ----------
if USE_KEYFRAMES_ONLY:
    # Keyframes только (рекомендовано): используем gps_key.dat / angle_key.dat
    print("Режим: keyframes только (~1.67 Hz)")
    # Создаем сначала как _interp для сохранения обратной совместимости
    grid = vt[vt >= 0.0]
    grid = np.r_[0.0, grid]  # якорь в t=0
    ge = np.r_[enu[0, 0], enu[vt >= 0.0, 0]]
    gn = np.r_[enu[0, 1], enu[vt >= 0.0, 1]]
    ga = np.r_[alt_v[0], alt_v[vt >= 0.0]]
    gh = np.interp(grid, att_t, compass) % 360.0
    # sig_scale from VO quality (anchor at t=0 gets 1.0)
    # NOTE: Disabled - testing showed it worsens RMS by 3.83 m (exp=1) or 6.42 m (exp=2)
    inlier_ref = 98.0
    fallback_mult = 2.0
    gsig = np.ones(len(grid))
    # grid_inliers = np.concatenate(([vinliers[0]], vinliers[vt >= 0.0]))
    # grid_status = [vstatus[0]] + list(vstatus[vt >= 0.0])
    # for i in range(1, len(grid)):
    #     if grid_inliers[i] > 0:
    #         scale = (inlier_ref / grid_inliers[i]) ** 1
    #         if grid_status[i] == 'ok_homography':
    #             scale *= fallback_mult
    #         gsig[i] = np.clip(scale, 1.0, 10.0)
    # Скорости из реинтегрированных шагов
    if REROTATE_STEPS:
        ve = compute_step_velocities(grid, ge, smooth_win=3)
        vn = compute_step_velocities(grid, gn, smooth_win=3)
        vvh = compute_step_velocities(grid, ga, smooth_win=3)
    else:
        ve = np.gradient(ge, grid)
        vn = np.gradient(gn, grid)
        vvh = np.gradient(ga, grid)
else:
    # Интерполяция до VO_FEED_HZ (НЕ рекомендовано: RMSE 291 м!)
    print(f"Режим: интерполяция до {VO_FEED_HZ} Hz (не рекомендовано)")
    t_feed0 = max(0.0, float(vt[0]))
    t_feed1 = float(min(vt[-1], imu_t[-1]))
    n_feed = int(np.floor((t_feed1 - t_feed0) * VO_FEED_HZ)) + 1
    grid = t_feed0 + np.arange(n_feed) / VO_FEED_HZ
    grid = grid[grid <= t_feed1 + 1e-9]
    ge = np.interp(grid, vt, enu[:, 0])
    gn = np.interp(grid, vt, enu[:, 1])
    ga = np.interp(grid, vt, alt_v)
    gh = np.interp(grid, vt, vhu)
    ve = smooth_deriv(grid, ge)
    vn = smooth_deriv(grid, gn)
    vvh = smooth_deriv(grid, ga)
    # sig_scale interpolated (not ideal but consistent)
    # NOTE: Disabled - testing showed it worsens RMS
    inlier_ref = 98.0
    fallback_mult = 2.0
    vsig = np.ones(len(vt))
    # for i in range(len(vt)):
    #     if vinliers[i] > 0:
    #         scale = (inlier_ref / vinliers[i]) ** 1
    #         if vstatus[i] == 'ok_homography':
    #             scale *= fallback_mult
    #         vsig[i] = np.clip(scale, 1.0, 10.0)
    gsig = np.ones(len(grid))  # vsig interpolated would go here if enabled

# ---------- imu.dat (body_FLU -> fusion body: x=fwd, y=up, z=right) ----------
imu_tab = os.path.join(OUT, "imu.dat")
last_t = -1.0
kept = 0
with open(imu_tab, "w", encoding="utf-8") as f:
    f.write("time_s\ttimestamp_ns\twx\twy\twz\tax\tay\taz\n")
    for i in range(len(imu_t)):
        t = imu_t[i]
        if t <= last_t:
            continue
        last_t = t
        gx, gy, gz = float(imu[i]["angular_velocity.x"]), float(imu[i]["angular_velocity.y"]), float(imu[i]["angular_velocity.z"])
        ax, ay, az = float(imu[i]["linear_acceleration.x"]), float(imu[i]["linear_acceleration.y"]), float(imu[i]["linear_acceleration.z"])
        f.write(f"{t:.6f}\t{int(t*1e9)}\t{gx:.9f}\t{gz:.9f}\t{-gy:.9f}\t{ax:.9f}\t{az:.9f}\t{-ay:.9f}\n")
        kept += 1
print(f"imu.dat: {kept} rows")

# ---------- initial heading from attitude ----------
initial_heading_deg = float(np.interp(0.0, att_t, compass)) % 360.0
gps_path = os.path.join(OUT, "gps.dat")
with open(gps_path, "w", encoding="utf-8") as f:
    f.write("time_s\ttimestamp_ns\tlatitude\tlongitude\taltitude\tvn\tvh\tve\tsig_scale\n")
    for i, t in enumerate(grid):
        la = lat0 + math.degrees(gn[i] / rm)
        lo = lon0 + math.degrees(ge[i] / (rn * math.cos(math.radians(lat0))))
        f.write(f"{t:.6f}\t{int(t*1e9)}\t{la:.9f}\t{lo:.9f}\t{ga[i]:.4f}\t{vn[i]:.5f}\t{vvh[i]:.5f}\t{ve[i]:.5f}\t{gsig[i]:.6f}\n")
print(f"gps.dat: {len(grid)} rows  {grid[0]:.3f}..{grid[-1]:.3f}  ({len(grid)/(grid[-1]-grid[0]):.2f} Hz)")
print(f"sig_scale: min {gsig.min():.3f}  max {gsig.max():.3f}  median {np.median(gsig):.3f}")

# body compass on the VO feed grid (NOT VO HeadingDeg: VO yaw drifts, compass does not)
gh = np.interp(grid, att_t, compass) % 360.0
ang_path = os.path.join(OUT, "angle.dat")
with open(ang_path, "w", encoding="utf-8") as f:
    f.write("time_s\ttimestamp_ns\troll\tpitch\tyaw\n")
    for i, t in enumerate(grid):
        f.write(f"{t:.6f}\t{int(t*1e9)}\t0.0\t0.0\t{math.radians(gh[i]):.6f}\n")
print(f"angle.dat: {len(grid)} rows  heading {gh[0]:.3f}..{gh[-1]:.3f} deg")

# ---------- keyframe-only variant (1.6 Hz) ----------
kgrid = vt[vt >= 0.0]
ke = np.interp(kgrid, vt, enu[:, 0])
kn = np.interp(kgrid, vt, enu[:, 1])
ka = np.interp(kgrid, vt, alt_v)
kh = np.interp(kgrid, att_t, compass) % 360.0
# anchor at t=0 so the static start is not unobserved
kgrid = np.r_[0.0, kgrid]
ke = np.r_[enu[0, 0], ke]
kn = np.r_[enu[0, 1], kn]
ka = np.r_[alt_v[0], ka]
kh = np.r_[initial_heading_deg, kh]
# sig_scale for keyframes
# NOTE: Disabled - testing showed it worsens RMS
ksig = np.ones(len(kgrid))
# kgrid_inliers = np.concatenate(([vinliers[0]], vinliers[vt >= 0.0]))
# kgrid_status = [vstatus[0]] + list(vstatus[vt >= 0.0])
# for i in range(1, len(kgrid)):
#     if kgrid_inliers[i] > 0:
#         scale = (inlier_ref / kgrid_inliers[i]) ** 1
#         if kgrid_status[i] == 'ok_homography':
#             scale *= fallback_mult
#         ksig[i] = np.clip(scale, 1.0, 10.0)
if REROTATE_STEPS:
    kve = compute_step_velocities(kgrid, ke, smooth_win=3)
    kvn = compute_step_velocities(kgrid, kn, smooth_win=3)
    kvh = compute_step_velocities(kgrid, ka, smooth_win=3)
else:
    kve = np.gradient(ke, kgrid)
    kvn = np.gradient(kn, kgrid)
    kvh = np.gradient(ka, kgrid)
with open(os.path.join(OUT, "gps_key.dat"), "w", encoding="utf-8") as f:
    f.write("time_s\ttimestamp_ns\tlatitude\tlongitude\taltitude\tvn\tvh\tve\tsig_scale\n")
    for i, t in enumerate(kgrid):
        la = lat0 + math.degrees(kn[i] / rm)
        lo = lon0 + math.degrees(ke[i] / (rn * math.cos(math.radians(lat0))))
        f.write(f"{t:.6f}\t{int(t*1e9)}\t{la:.9f}\t{lo:.9f}\t{ka[i]:.4f}\t{kvn[i]:.5f}\t{kvh[i]:.5f}\t{kve[i]:.5f}\t{ksig[i]:.6f}\n")
with open(os.path.join(OUT, "angle_key.dat"), "w", encoding="utf-8") as f:
    f.write("time_s\ttimestamp_ns\troll\tpitch\tyaw\n")
    for i, t in enumerate(kgrid):
        f.write(f"{t:.6f}\t{int(t*1e9)}\t0.0\t0.0\t{math.radians(kh[i]):.6f}\n")
print(f"gps_key.dat: {len(kgrid)} rows ({len(kgrid)/(kgrid[-1]-kgrid[0]):.2f} Hz)")

# ---------- StartupNav.ini (4 lines + optional initial heading) ----------
with open(os.path.join(OUT, "StartupNav.ini"), "w", encoding="utf-8") as f:
    f.write(f"{lon0:.12f}\n{lat0:.12f}\n{alt0:.6f}\n10.0\n{initial_heading_deg:.6f}\n")

# ---------- diagnostics ----------
print("\nfirst 10 gps rows:")
print(open(gps_path, encoding="utf-8").read().splitlines()[:4])
print("alignment residual (first 105 s):")
p = (R @ np.c_[vx[m], vy[m]].T).T + tr
res = np.hypot(p[:,0] - np.interp(vt[m], rtk_t, E), p[:,1] - np.interp(vt[m], rtk_t, N))
print(f"  rmse {np.sqrt(np.mean(res**2)):.2f} m  max {res.max():.2f} m")

# Re-integrated aligned VO vs RTK (whole flight)
print("\nRe-integrated aligned VO vs RTK (whole flight):")
e_interp = np.interp(vt, rtk_t, E)
n_interp = np.interp(vt, rtk_t, N)
err_full = np.hypot(enu[:, 0] - e_interp, enu[:, 1] - n_interp)
rms_full = np.sqrt(np.mean(err_full**2))
max_full = err_full.max()
final_full = err_full[-1]
print(f"  RMS {rms_full:6.2f} m  max {max_full:6.2f} m  final {final_full:6.2f} m")
