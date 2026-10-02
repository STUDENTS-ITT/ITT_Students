import numpy as np

p = r"C:\Users\ORNK\Downloads\ITT_Students-Testing_branch\ITT_Students-Testing_branch\newformat\tools\kalman15_line2.txt"
hdr = open(p, encoding="utf-8").readline().split()[1:]
rows = []
for line in open(p, encoding="utf-8"):
    parts = line.split()
    if not parts or parts[0].count(":") != 2:
        continue
    try:
        rows.append([float(x) for x in parts[1:]])
    except ValueError:
        pass
d = np.array(rows)
idx = {n: i for i, n in enumerate(hdr)}
gps = d[:, idx["lon_sns"] : idx["ve_sns"] + 1]
chg = np.ones(len(d), bool)
chg[1:] = np.any(np.abs(np.diff(gps, axis=0)) > 1e-12, axis=1)
g = d[chg]
t = g[:, idx["time"]]
m = (t >= 341) & (t <= 686)
g = g[m]
t = t[m]


def wrap(a):
    return (a + 180) % 360 - 180


for name in ["heading", "pitch", "roll"]:
    a = g[:, idx[name]]
    b = g[:, idx[name + "_sns"]]
    e = wrap(a - b)
    print(name, "kf", a.min(), a.max(), "sns", b.min(), b.max(), "rmse", np.sqrt(np.mean(e**2)), "median", np.median(np.abs(e)))

print("--- samples ---")
for ts in [350, 400, 430, 460, 500, 520, 550, 580, 610, 650, 680]:
    i = int(np.argmin(np.abs(t - ts)))
    print(
        f"t={t[i]:.1f} hdg {g[i, idx['heading']]:.1f}/{g[i, idx['heading_sns']]:.1f} "
        f"pitch {g[i, idx['pitch']]:.1f}/{g[i, idx['pitch_sns']]:.1f} "
        f"roll {g[i, idx['roll']]:.1f}/{g[i, idx['roll_sns']]:.1f}"
    )

e = wrap(g[:, idx["heading"]] - g[:, idx["heading_sns"]])
bad = np.where(np.abs(e) > 20)[0]
print("bad hdg", len(bad), "of", len(e))
if len(bad):
    jumps = np.where(np.diff(bad) > 5)[0]
    starts = [bad[0]] + [bad[k + 1] for k in jumps]
    ends = [bad[k] for k in jumps] + [bad[-1]]
    for a, b in zip(starts, ends):
        print(
            f"  {t[a]:.1f}-{t[b]:.1f} n={b - a + 1} "
            f"kf {g[a, idx['heading']]:.1f}->{g[b, idx['heading']]:.1f} "
            f"sns {g[a, idx['heading_sns']]:.1f}->{g[b, idx['heading_sns']]:.1f}"
        )

level = (np.abs(g[:, idx["pitch_sns"]]) < 45) & (np.abs(g[:, idx["roll_sns"]]) < 45)
print("level samples", int(level.sum()), "t", t[level][0], t[level][-1] if level.any() else None)
for name in ["heading", "pitch", "roll"]:
    e = wrap(g[level, idx[name]] - g[level, idx[name + "_sns"]])
    print(" level", name, "rmse", np.sqrt(np.mean(e**2)), "max", np.max(np.abs(e)))
print("alt rmse", np.sqrt(np.mean((g[:, idx["alt"]] - g[:, idx["alt_sns"]]) ** 2)))
print("vh rmse", np.sqrt(np.mean((g[:, idx["vh"]] - g[:, idx["vh_sns"]]) ** 2)))
print("pitch kf median", np.median(g[:, idx["pitch"]]), "roll kf", np.median(g[:, idx["roll"]]))

ep = wrap(g[:, idx["pitch"]] - g[:, idx["pitch_sns"]])
bad = np.where(np.abs(ep) > 20)[0]
print("bad pitch", len(bad), "of", len(ep), "sns unique-ish", np.percentile(g[:, idx["pitch_sns"]], [0, 50, 100]))
er = wrap(g[:, idx["roll"]] - g[:, idx["roll_sns"]])
print("bad roll", int(np.sum(np.abs(er) > 20)), "sns", np.percentile(g[:, idx["roll_sns"]], [0, 50, 100]))
