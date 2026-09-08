#!/usr/bin/env python3
"""Мост: траектория видеонавигации -> формат входных файлов ITT_Students.

Читает траекторию VINS-Mono или DSO в формате TUM и создаёт файлы, которые
ваша БИНС читает без единой правки C++:

    gps_vo.dat    time_s  timestamp_ns  lat  lon  alt  vn  vh  ve   (градусы, м, м/с)
    angle_vo.dat  time_s  timestamp_ns  roll  pitch  yaw            (радианы)
    sigma_vo.dat  time_s  sigma_pos_m  sigma_hdg_rad                (для варианта A)

Системы координат
-----------------
Навигационная СК проекта -- (Север, Вверх, Восток), курс отсчитывается от
севера по часовой стрелке. СК тела -- X вперёд, Y вверх, Z вправо.
Матрица перехода совпадает с bodyToNavMatrix из src/math_lib/transformations.h.

Перевод в геодезические координаты выполняется ТОЙ ЖЕ сферической моделью,
что и в вашем фильтре: R = 6371000 м, приращения интегрируются пошагово так
же, как в nav::integratePosition. Если взять эллипсоид WGS84, появится
систематическое расхождение с моделью фильтра, которое легко принять за
ошибку видеонавигации.

Привязка
--------
Видеоодометрия знает только относительное перемещение. Абсолютную точку
старта, начальный курс и (для DSO) масштаб приходится задавать снаружи --
из первых секунд эталона.

Примеры
-------
    # VINS-Mono: метрический, масштаб не подбираем
    python3 vo_to_gps_dat.py --traj vins.tum --ref-gps gps.dat \\
        --ref-angle angle.dat --out-dir ins_input

    # DSO: масштаб оцениваем по первым 30 с эталона
    python3 vo_to_gps_dat.py --traj dso.tum --ref-gps gps.dat \\
        --estimate-scale 30 --out-dir ins_input

    # только курс из видео, крен и тангаж оставить из эталона
    python3 vo_to_gps_dat.py --traj vins.tum --ref-gps gps.dat \\
        --ref-angle angle.dat --attitude yaw-only --out-dir ins_input
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

# Та же константа, что в src/utils/constants.h.
R_EARTH = 6371e3

# Перестановка осей: ENU (Восток, Север, Вверх) -> NUE (Север, Вверх, Восток).
ENU_TO_NUE = np.array([[0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0],
                       [1.0, 0.0, 0.0]])

# СК тела проекта (X вперёд, Y вверх, Z вправо) из ROS-конвенций.
BODY_FROM = {
    # FLU: X вперёд, Y влево, Z вверх (ROS, DJI OSDK, типовой VINS-Mono)
    "flu": np.array([[1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0],
                     [0.0, -1.0, 0.0]]),
    # FRD: X вперёд, Y вправо, Z вниз (аэрокосмическая конвенция)
    "frd": np.array([[1.0, 0.0, 0.0],
                     [0.0, 0.0, -1.0],
                     [0.0, 1.0, 0.0]]),
    # Уже совпадает с СК проекта
    "fur": np.eye(3),
}


# --------------------------------------------------------------------------
# Чтение и запись
# --------------------------------------------------------------------------

def read_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """TUM: time tx ty tz qx qy qz qw -> (t, xyz, quat_xyzw)."""
    rows = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.replace(",", " ").split()
            if len(p) < 8:
                continue
            try:
                rows.append([float(v) for v in p[:8]])
            except ValueError:
                continue
    if not rows:
        raise SystemExit(f"Пустая или нечитаемая траектория: {path}")
    a = np.array(rows)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def read_gps_dat(path: Path) -> dict[str, np.ndarray]:
    """gps.dat: time_s timestamp_ns lat lon alt vn vh ve (широта/долгота в градусах)."""
    rows = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 5:
                continue
            try:
                rows.append([float(v) for v in p[:8]] if len(p) >= 8
                            else [float(v) for v in p[:5]] + [0.0, 0.0, 0.0])
            except ValueError:
                continue
    if not rows:
        raise SystemExit(f"Пустой эталон: {path}")
    a = np.array(rows)
    return {
        "t": a[:, 0],
        "ns": a[:, 1],
        "lat": np.deg2rad(a[:, 2]),
        "lon": np.deg2rad(a[:, 3]),
        "alt": a[:, 4],
        "vn": a[:, 5], "vh": a[:, 6], "ve": a[:, 7],
    }


def read_angle_dat(path: Path) -> dict[str, np.ndarray]:
    """angle.dat: time_s timestamp_ns roll pitch yaw (радианы)."""
    rows = []
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 5:
                continue
            try:
                rows.append([float(v) for v in p[:5]])
            except ValueError:
                continue
    if not rows:
        raise SystemExit(f"Пустой или нечитаемый файл углов: {path}")
    a = np.array(rows)
    return {"t": a[:, 0], "roll": a[:, 2], "pitch": a[:, 3], "yaw": a[:, 4]}


def ensure_increasing(t: np.ndarray, *arrays: np.ndarray):
    """Убрать повторы и обратный ход времени: иначе np.gradient даёт inf."""
    keep = [0]
    for i in range(1, len(t)):
        if t[i] > t[keep[-1]] + 1e-9:
            keep.append(i)
    keep = np.array(keep)
    if len(keep) < len(t):
        print(f"  отброшено {len(t) - len(keep)} отсчётов с неверным временем")
    return (t[keep], *(a[keep] for a in arrays))


# --------------------------------------------------------------------------
# Геодезия (сферическая модель, как в фильтре)
# --------------------------------------------------------------------------

def geodetic_to_nue(lat, lon, alt, lat0, lon0, alt0) -> np.ndarray:
    """Геодезические -> локальные (Север, Вверх, Восток), сферическая модель."""
    rh = R_EARTH + alt0
    north = (lat - lat0) * rh
    east = (lon - lon0) * rh * math.cos(lat0)
    up = alt - alt0
    return np.column_stack([north, up, east])


def nue_to_geodetic(nue: np.ndarray, lat0: float, lon0: float, alt0: float):
    """Локальные (Север, Вверх, Восток) -> геодезические.

    Приращения интегрируются пошагово ровно так же, как в
    nav::integratePosition, чтобы не расходиться с моделью фильтра.
    """
    n = len(nue)
    lat = np.empty(n)
    lon = np.empty(n)
    alt = np.empty(n)

    cur_lat, cur_lon, cur_alt = lat0, lon0, alt0
    prev = nue[0].copy()
    for i in range(n):
        d = nue[i] - prev
        rh = R_EARTH + cur_alt
        cur_alt += d[1]
        cur_lat += d[0] / rh
        cur_lat = max(-math.pi / 2, min(math.pi / 2, cur_lat))
        cur_lon += d[2] / (rh * max(abs(math.cos(cur_lat)), 1e-10))
        lat[i], lon[i], alt[i] = cur_lat, cur_lon, cur_alt
        prev = nue[i]
    return lat, lon, alt


# --------------------------------------------------------------------------
# Выравнивание
# --------------------------------------------------------------------------

def horizontal_similarity(src: np.ndarray, dst: np.ndarray, with_scale: bool):
    """2D-подобие в горизонтальной плоскости (Умеяма): масштаб, поворот, сдвиг.

    Поворот ограничен вертикальной осью: крен и тангаж у VINS-Mono уже
    наблюдаемы через вектор тяжести, и «доворачивать» их было бы неверно.
    """
    if len(src) < 3:
        raise SystemExit("Слишком короткое окно выравнивания, увеличьте --align-window")

    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    s0, d0 = src - mu_s, dst - mu_d

    # Оптимальный поворот на плоскости через сумму комплексных произведений.
    num = np.sum(s0[:, 0] * d0[:, 1] - s0[:, 1] * d0[:, 0])
    den = np.sum(s0[:, 0] * d0[:, 0] + s0[:, 1] * d0[:, 1])
    theta = math.atan2(num, den)

    scale = 1.0
    if with_scale:
        var_s = np.sum(s0 ** 2)
        if var_s < 1e-12:
            raise SystemExit("Нулевое перемещение в окне выравнивания — масштаб не оценить")
        scale = float(math.hypot(num, den) / var_s)

    return scale, theta, mu_s, mu_d


def rot_z(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Кватернион (x, y, z, w) -> матрица поворота 3x3."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_project_euler(c: np.ndarray) -> tuple[float, float, float]:
    """C_body^nav (СК проекта) -> (heading, pitch, roll).

    Обратно к bodyToNavMatrix из src/math_lib/transformations.h:
        C[1][0] = sin(pitch)
        C[1][1] = cos(roll)*cos(pitch)
        C[1][2] = -sin(roll)*cos(pitch)
        C[0][0] = cos(pitch)*cos(heading)
        C[2][0] = cos(pitch)*sin(heading)
    """
    pitch = math.asin(max(-1.0, min(1.0, c[1, 0])))
    if abs(math.cos(pitch)) < 1e-6:
        # Вырождение при тангаже +-90 градусов.
        heading = math.atan2(-c[0, 2], c[2, 2])
        roll = 0.0
    else:
        roll = math.atan2(-c[1, 2], c[1, 1])
        heading = math.atan2(c[2, 0], c[0, 0])
    return heading, pitch, roll


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --------------------------------------------------------------------------
# Основная логика
# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traj", type=Path, required=True,
                    help="траектория видеонавигации в формате TUM")
    ap.add_argument("--ref-gps", type=Path, required=True,
                    help="эталонный gps.dat: задаёт точку привязки и выравнивание")
    ap.add_argument("--ref-angle", type=Path,
                    help="эталонный angle.dat: нужен для attitude=yaw-only")
    ap.add_argument("--out-dir", type=Path, required=True)

    ap.add_argument("--scale", type=float, default=None,
                    help="масштабный множитель для DSO, если известен извне")
    ap.add_argument("--estimate-scale", type=float, default=None, metavar="СЕК",
                    help="оценить масштаб по первым N секундам эталона (для DSO)")
    ap.add_argument("--align-window", type=float, default=20.0, metavar="СЕК",
                    help="окно выравнивания курса и точки старта, с")

    ap.add_argument("--body", choices=sorted(BODY_FROM), default="flu",
                    help="СК тела в траектории: flu (ROS/DJI), frd, fur")
    ap.add_argument("--attitude", choices=["vo", "yaw-only", "none"], default="vo",
                    help="vo: все углы из видео; yaw-only: только курс, "
                         "крен и тангаж из эталона; none: не писать angle_vo.dat")

    ap.add_argument("--rate", type=float, default=None, metavar="ГЦ",
                    help="прореживание выхода до заданной частоты")
    ap.add_argument("--sigma0", type=float, default=3.0,
                    help="начальная погрешность положения, м")
    ap.add_argument("--sigma-rate", type=float, default=0.02,
                    help="рост погрешности положения, м/с")
    ap.add_argument("--sigma-hdg0", type=float, default=1.0,
                    help="начальная погрешность курса, градусы")
    ap.add_argument("--sigma-hdg-rate", type=float, default=0.05,
                    help="рост погрешности курса, градусы/с")
    args = ap.parse_args()

    # ---- загрузка -------------------------------------------------------
    t_vo, p_vo, q_vo = read_tum(args.traj)
    ref = read_gps_dat(args.ref_gps)
    ang = read_angle_dat(args.ref_angle) if args.ref_angle else None

    print(f"Траектория: {len(t_vo)} поз, {t_vo[0]:.3f}..{t_vo[-1]:.3f} с")
    print(f"Эталон:     {len(ref['t'])} отсчётов, "
          f"{ref['t'][0]:.3f}..{ref['t'][-1]:.3f} с")

    t_vo, p_vo, q_vo = ensure_increasing(t_vo, p_vo, q_vo)

    # Приводим шкалу VO к шкале эталона: обе начинаются с нуля полёта.
    t_rel = t_vo - t_vo[0] + ref["t"][0]

    lat0, lon0, alt0 = ref["lat"][0], ref["lon"][0], ref["alt"][0]
    ref_nue = geodetic_to_nue(ref["lat"], ref["lon"], ref["alt"], lat0, lon0, alt0)

    # ---- выравнивание ---------------------------------------------------
    t_end = t_rel[0] + args.align_window
    mask = t_rel <= t_end
    if mask.sum() < 3:
        mask = np.arange(len(t_rel)) < max(3, len(t_rel) // 10)

    # Эталон в тех же моментах времени, что и VO в окне выравнивания.
    ref_n = np.interp(t_rel[mask], ref["t"], ref_nue[:, 0])
    ref_e = np.interp(t_rel[mask], ref["t"], ref_nue[:, 2])

    # VO пока в своей СК: считаем её ENU-подобной (x, y горизонталь, z вверх).
    src_xy = p_vo[mask][:, :2]
    dst_xy = np.column_stack([ref_e, ref_n])   # горизонталь как (Восток, Север)

    with_scale = args.estimate_scale is not None
    if with_scale:
        t_end_s = t_rel[0] + args.estimate_scale
        m2 = t_rel <= t_end_s
        if m2.sum() >= 3:
            src_xy = p_vo[m2][:, :2]
            dst_xy = np.column_stack([
                np.interp(t_rel[m2], ref["t"], ref_nue[:, 2]),
                np.interp(t_rel[m2], ref["t"], ref_nue[:, 0]),
            ])

    scale, theta, _, _ = horizontal_similarity(src_xy, dst_xy, with_scale)

    if args.scale is not None:
        scale = args.scale
        print(f"Масштаб задан вручную: {scale:.6f}")
    elif with_scale:
        print(f"Масштаб оценён по первым {args.estimate_scale:g} с: {scale:.6f}")
    else:
        scale = 1.0
        print("Масштаб принят равным 1.0 (метрическая траектория, VINS-Mono)")
    print(f"Поворот СК видеонавигации к северу: {math.degrees(theta):+.2f}°")

    # Применяем: масштаб, поворот вокруг вертикали, привязка к точке старта.
    rz = rot_z(theta)
    p_aligned = (rz @ (scale * p_vo).T).T
    p_aligned -= p_aligned[0]                       # старт в нуле
    enu = p_aligned                                  # (Восток, Север, Вверх)
    nue = (ENU_TO_NUE @ enu.T).T                     # (Север, Вверх, Восток)

    lat, lon, alt = nue_to_geodetic(nue, lat0, lon0, alt0)

    # ---- скорости -------------------------------------------------------
    # Центральные разности; на краях односторонние.
    vel = np.gradient(nue, t_rel, axis=0)
    vn, vh, ve = vel[:, 0], vel[:, 1], vel[:, 2]

    # ---- углы -----------------------------------------------------------
    roll = np.zeros(len(t_rel))
    pitch = np.zeros(len(t_rel))
    yaw = np.zeros(len(t_rel))

    if args.attitude != "none":
        r_body = BODY_FROM[args.body]        # СК проекта <- СК траектории
        r_body_inv = r_body.T
        m_nue_enu = ENU_TO_NUE
        for i in range(len(t_rel)):
            # C_body^nav в СК проекта:
            #   NUE <- ENU <- (поворот выравнивания) <- R_vo <- (СК тела проекта)
            c_vo = quat_to_matrix(q_vo[i])
            c = m_nue_enu @ rz @ c_vo @ r_body_inv
            yaw[i], pitch[i], roll[i] = matrix_to_project_euler(c)

        if ang is not None:
            # Привязка курса к северу по первому отсчёту эталона.
            yaw_offset = wrap_pi(np.interp(t_rel[0], ang["t"], ang["yaw"]) - yaw[0])
            yaw = np.array([wrap_pi(y + yaw_offset) for y in yaw])
            print(f"Курс привязан к эталону, сдвиг {math.degrees(yaw_offset):+.2f}°")

        if args.attitude == "yaw-only":
            if ang is None:
                raise SystemExit("--attitude yaw-only требует --ref-angle")
            roll = np.interp(t_rel, ang["t"], ang["roll"])
            pitch = np.interp(t_rel, ang["t"], ang["pitch"])
            print("Крен и тангаж взяты из эталона, из видео только курс")

    # ---- прореживание ---------------------------------------------------
    idx = np.arange(len(t_rel))
    if args.rate:
        step = 1.0 / args.rate
        keep, next_t = [], t_rel[0]
        for i, tt in enumerate(t_rel):
            if tt >= next_t - 1e-9:
                keep.append(i)
                next_t += step
        idx = np.array(keep)
        print(f"Прорежено до {args.rate:g} Гц: {len(idx)} отсчётов")

    # ---- запись ---------------------------------------------------------
    args.out_dir.mkdir(parents=True, exist_ok=True)
    ns0 = ref["ns"][0]

    gps_path = args.out_dir / "gps_vo.dat"
    with gps_path.open("w", encoding="utf-8") as f:
        f.write("# time_s\ttimestamp_ns\tlatitude\tlongitude\taltitude\tvn\tvh\tve\n")
        for i in idx:
            ns = int(ns0 + (t_rel[i] - t_rel[0]) * 1e9)
            f.write(f"{t_rel[i]:.6f}\t{ns}\t"
                    f"{math.degrees(lat[i]):.10f}\t{math.degrees(lon[i]):.10f}\t"
                    f"{alt[i]:.4f}\t{vn[i]:.6f}\t{vh[i]:.6f}\t{ve[i]:.6f}\n")

    written = [gps_path]

    if args.attitude != "none":
        ang_path = args.out_dir / "angle_vo.dat"
        with ang_path.open("w", encoding="utf-8") as f:
            f.write("# time_s\ttimestamp_ns\troll\tpitch\tyaw\n")
            for i in idx:
                ns = int(ns0 + (t_rel[i] - t_rel[0]) * 1e9)
                f.write(f"{t_rel[i]:.6f}\t{ns}\t"
                        f"{roll[i]:.10f}\t{pitch[i]:.10f}\t{yaw[i]:.10f}\n")
        written.append(ang_path)

    # Растущая погрешность: видеоодометрия дрейфует, и постоянная R в фильтре
    # для неё некорректна. Подробности в разделе 10.3 отчёта.
    sig_path = args.out_dir / "sigma_vo.dat"
    with sig_path.open("w", encoding="utf-8") as f:
        f.write("# time_s\tsigma_pos_m\tsigma_hdg_rad\n")
        for i in idx:
            dt = t_rel[i] - t_rel[0]
            sp = args.sigma0 + args.sigma_rate * dt
            sh = math.radians(args.sigma_hdg0 + args.sigma_hdg_rate * dt)
            f.write(f"{t_rel[i]:.6f}\t{sp:.4f}\t{sh:.8f}\n")
    written.append(sig_path)

    # ---- контроль -------------------------------------------------------
    print("\nЗаписано:")
    for p in written:
        print(f"  {p}")

    ref_at = np.column_stack([
        np.interp(t_rel[idx], ref["t"], ref_nue[:, 0]),
        np.interp(t_rel[idx], ref["t"], ref_nue[:, 1]),
        np.interp(t_rel[idx], ref["t"], ref_nue[:, 2]),
    ])
    err = np.linalg.norm(nue[idx] - ref_at, axis=1)
    print("\nОтклонение от эталона (справочно, эталон может быть неполным):")
    print(f"  СКО      {np.sqrt(np.mean(err ** 2)):8.2f} м")
    print(f"  медиана  {np.median(err):8.2f} м")
    print(f"  максимум {np.max(err):8.2f} м")
    print(f"  в конце  {err[-1]:8.2f} м")

    print("\nДальше:")
    print(f"  cp {args.out_dir}/gps_vo.dat   .../data/raw/gps.dat")
    if args.attitude != "none":
        print(f"  cp {args.out_dir}/angle_vo.dat .../data/raw/angle.dat")
    print("  cd build && ./imitator")
    print("\nОригиналы gps.dat и angle.dat сохраните заранее.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
