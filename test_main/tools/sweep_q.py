# Перебор параметров Q: собирает и прогоняет программу в отдельном каталоге,
# рабочее дерево не трогает. Метрики — СКО по углам и высоте плюс размах
# оценки смещения вертикального акселерометра.

import io
import math
import os
import shutil
import subprocess
import sys

GXX = r"C:\Users\User\Desktop\kat\w64devkit\bin\g++.exe"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORK = os.path.join(ROOT, "_sweep")

import re

Q_LINE = re.compile(r"^(\s*const double (sig_g|sig_a|sig_bg|sig_ba)) = [^;]+;",
                    re.MULTILINE)


def prepare():
    if os.path.isdir(WORK):
        shutil.rmtree(WORK)
    os.makedirs(WORK)
    shutil.copytree(os.path.join(ROOT, "src"), os.path.join(WORK, "src"))
    bak = os.path.join(WORK, "src", "navigation", "trajectory.h.bak")
    if os.path.exists(bak):
        os.remove(bak)
    for name in ("imu.dat", "gps.dat", "angle.dat"):
        shutil.copy(os.path.join(ROOT, name), os.path.join(WORK, name))


ORIGINAL = {}


def patch(sig_g, sig_bg, sig_a, sig_ba):
    path = os.path.join(WORK, "src", "ins", "ins_filter.h")
    if "text" not in ORIGINAL:
        ORIGINAL["text"] = io.open(path, encoding="utf-8").read()
    values = {"sig_g": sig_g, "sig_a": sig_a, "sig_bg": sig_bg, "sig_ba": sig_ba}
    text, count = Q_LINE.subn(
        lambda m: "%s = %.6e;" % (m.group(1), values[m.group(2)]), ORIGINAL["text"])
    if count != 4:
        raise SystemExit("v ins_filter.h naydeno %d iz 4 parametrov Q" % count)
    io.open(path, "w", encoding="utf-8").write(text)


def build_and_run():
    src = os.path.join(WORK, "src")
    cmd = [GXX, "-std=c++17", "-O2", "-I", src,
           os.path.join(src, "main.cpp"),
           os.path.join(src, "data_io", "data_reader.cpp"),
           os.path.join(src, "data_io", "data_writer.cpp"),
           "-o", os.path.join(WORK, "sweep.exe")]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit("сборка не удалась:\n" + r.stderr[:2000])
    subprocess.run([os.path.join(WORK, "sweep.exe")], cwd=WORK,
                   capture_output=True, text=True)


def load(path, cols):
    out = []
    with io.open(os.path.join(WORK, path), encoding="utf-8", errors="replace") as f:
        f.readline()
        for line in f:
            q = line.split()
            if len(q) >= cols:
                out.append([float(v) for v in q[:cols]])
    return out


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def metrics():
    k = [r for r in load("kalman15_line2.txt", 21) if r[0] >= 181]

    # d_1.txt хранит поправку за цикл, накопленное смещение — её сумма
    ba = bg = 0.0
    tail_ba, tail_bg = [], []
    for r in load("d_1.txt", 16):
        ba += r[11]
        bg += r[14]
        if r[0] > 400:
            tail_ba.append(ba)
            tail_bg.append(bg)

    return (rms([r[3] - r[13] for r in k]), rms([r[4] - r[15] for r in k]),
            rms([r[5] - r[14] for r in k]), rms([r[9] - r[12] for r in k]),
            max(tail_ba) - min(tail_ba), max(tail_bg) - min(tail_bg))


P_LINE = re.compile(r"^(\s*const double p_ba) = [^;]+;", re.MULTILINE)


def patch_p0(p_ba):
    path = os.path.join(WORK, "src", "navigation", "aligner.h")
    if "p0" not in ORIGINAL:
        ORIGINAL["p0"] = io.open(path, encoding="utf-8").read()
    text, count = P_LINE.subn(r"\1 = %.6e;" % p_ba, ORIGINAL["p0"])
    if count != 1:
        raise SystemExit("ne nayden p_ba v aligner.h")
    io.open(path, "w", encoding="utf-8").write(text)


def main():
    prepare()
    sig_g, sig_a = 1.10e-4, 3.05e-3  # измерено по неподвижному участку imu.dat
    base_bg, base_ba = 3.8785e-5, 2e-4

    patch_p0(9e-6)  # sigma_ba0 = 3e-3, подобрано предыдущим перебором

    print("%9s %8s %8s %8s %8s %8s %8s %11s" %
          ("sig_g", "ba mn", "bg mn", "hdg", "pitch", "roll", "dh", "razmah ba"))
    for sg in (3.394e-4, 1.10e-4):
        for fa in (1.0, 0.1, 0.03):
            for fg in (1.0, 0.3):
                patch(sg, base_bg * fg, sig_a, base_ba * fa)
                build_and_run()
                m = metrics()
                print("%9.2e %8.2f %8.2f %8.4f %8.4f %8.4f %8.3f %11.3e" %
                      ((sg, fa, fg) + m[:4] + (m[4],)))
                sys.stdout.flush()

    shutil.rmtree(WORK, ignore_errors=True)


if __name__ == "__main__":
    main()
