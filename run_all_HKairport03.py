# -*- coding: utf-8 -*-
"""
Прогон всех моделей движения на наборе HKairport03 и сверка с RTK.

Все параметры этого набора уже проставлены ниже, запускать без аргументов:

    python HKairport03/run_all.py

Скрипт делает четыре вещи:
  1) готовит входные файлы набора - калибровку камеры, высоту по кадрам и
     эталонный трек GPS (если их ещё нет);
  2) последовательно прогоняет vo_final2.py тремя моделями движения -
     similarity, homography, essential;
  3) замеряет, сколько миллисекунд стоит каждый этап конвейера;
  4) сверяет полученные траектории с RTK и рисует общий график.

Коррекции по GPS/ИНС в самой одометрии НЕТ. RTK участвует двумя способами:
как эталон для сверки и как замена высотомера - вертикальный канал
выгружается в altitude_per_frame.csv и подаётся ключом --altitude-file.
Без него (постоянная высота 80 м) ошибка гомографии растёт с 27 до 36 м.
"""
import csv
import io
import os
import subprocess
import sys
import time
from ctypes import pythonapi

import numpy as np

# =============================================================================
#  ПАРАМЕТРЫ НАБОРА HKairport03
# =============================================================================
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                      # папка с vo_final2.py

FRAMES = os.path.join(HERE, 'camera', 'images')   # 3657 кадров, 2448x2048
FPS = 10.0                                        # аппаратная синхронизация PPS

# Камера Hikvision CA-050-11UC: матрица Sony IMX264, пиксель 3.45 мкм,
# объектив 5 мм. Отсюда f = 5 мм / 3.45 мкм = 1449 пикселей. Без калибровки
# подставилось бы приближение 0.9*2448 = 2203 px - ошибка масштаба в полтора
# раза. Дисторсия нулевая: подбор k1 по самим кадрам дал минимум невязки
# 1.76 px при k1 = 0.03 против 1.79 px при нуле, то есть кадры уже исправлены.
PIXEL_UM = 3.45
LENS_MM = 5.0
WIDTH, HEIGHT = 2448, 2048

CALIBRATION = os.path.join(HERE, 'calibration', 'camera.json')
ALTITUDE_FILE = os.path.join(HERE, 'altitude_per_frame.csv')
GPS_REFERENCE = os.path.join(HERE, 'gps_reference.csv')
RTK_CSV = os.path.join(HERE, 'gps', 'dji_osdk_ros_rtk_position.csv')
CAMERA_CSV = os.path.join(HERE, 'camera', 'timestamps.csv')

# Связка особых точек. FAST-FREAK: детектор FAST + двоичный дескриптор FREAK.
# На надирной съёмке инвариантности SIFT/KAZE не нужны, а стоит эта связка
# заметно меньше.
ALGORITHM = 'FAST-FREAK'

# Модели движения прогоняются по очереди. Порядок - от простой к сложной.
MOTION_MODELS = ('similarity', 'homography', 'essential')

# Масштаб не подгоняется: фокусное расстояние и высота известны из паспорта
# камеры и телеметрии, поэтому поправка единичная.
SCALE_CORRECTION = 1.0

RESULTS = os.path.join(ROOT, 'results_HKairport03')
PLOTS_DIR = os.path.join(ROOT, 'trajectory_plots')
PLOT = os.path.join(PLOTS_DIR, 'HK_01_models.png')             # трек и ошибка
PLOT_PROFILE = os.path.join(PLOTS_DIR, 'HK_02_profile.png')    # условия полёта
PLOT_BY_MODEL = os.path.join(PLOTS_DIR, 'HK_03_by_model.png')  # модели порознь
ATTITUDE_CSV = os.path.join(HERE, 'other', 'dji_osdk_ros_attitude.csv')

# Границы фаз полёта, секунды от первого кадра, и развороты между галсами -
# по ним считается, где набирается ошибка.
PHASES = [('подъём', 0, 60), ('висение', 60, 95),
          ('маршрут', 95, 310), ('снижение', 310, 400)]
TURNS = [(105, 120), (140, 155), (185, 200), (232, 247), (275, 292)]


# =============================================================================
#  1. ПОДГОТОВКА ВХОДНЫХ ФАЙЛОВ
# =============================================================================
def sensor_time(path, sec_col, nsec_col):
    """
    Метка ДАТЧИКА в секундах, а не bag_time.

    Разница принципиальна: запись в bag отстаёт от съёмки на 2.8 с, а при
    скорости 9 м/с это 25 метров - весь бюджет ошибки.
    """
    raw = np.loadtxt(path, delimiter=',', skiprows=1,
                     usecols=(sec_col, nsec_col), dtype=float)
    return raw[:, 0] + raw[:, 1] * 1e-9


def read_rtk():
    """RTK: время датчика, широта, долгота, высота над эллипсоидом."""
    lat, lon, alt = [], [], []
    with io.open(RTK_CSV, encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            lat.append(float(row['latitude']))
            lon.append(float(row['longitude']))
            alt.append(float(row['altitude']))
    t = sensor_time(RTK_CSV, 3, 4)
    return t, np.array(lat), np.array(lon), np.array(alt)


def to_local(lat, lon):
    """WGS-84 -> местные метры (восток, север) от первой точки."""
    lat0, lon0 = lat[0], lon[0]
    a, e2 = 6378137.0, 6.69437999014e-3
    s = np.sin(np.radians(lat0))
    rn = a / np.sqrt(1 - e2 * s * s)
    rm = rn * (1 - e2) / (1 - e2 * s * s)
    east = np.radians(lon - lon0) * rn * np.cos(np.radians(lat0))
    north = np.radians(lat - lat0) * rm
    return east, north


def prepare():
    """Калибровка, высота по кадрам и эталонный трек - всё, что нужно прогону."""
    import json

    os.makedirs(os.path.dirname(CALIBRATION), exist_ok=True)
    focal = LENS_MM * 1e-3 / (PIXEL_UM * 1e-6)
    json.dump({'fx': focal, 'fy': focal,
               'cx': (WIDTH - 1) / 2.0, 'cy': (HEIGHT - 1) / 2.0,
               'imageSize': [HEIGHT, WIDTH], 'distortion': [0.0] * 5,
               'note': 'Hikvision CA-050-11UC, 2448x2048, пиксель 3.45 мкм, '
                       'объектив 5 мм'},
              io.open(CALIBRATION, 'w', encoding='utf-8'),
              indent=2, ensure_ascii=False)

    t_rtk, lat, lon, alt = read_rtk()
    t_cam = sensor_time(CAMERA_CSV, 6, 7)
    ground = float(np.median(alt[:20]))

    # Высота над точкой взлёта на КАЖДЫЙ кадр камеры. Топик
    # height_above_takeoff в этом наборе не заполнен (везде около нуля),
    # поэтому высота берётся из вертикального канала RTK.
    height = np.maximum(np.interp(t_cam, t_rtk, alt) - ground, 0.05)
    with io.open(ALTITUDE_FILE, 'w', encoding='utf-8', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(['frame', 'altitude'])
        for i, h in enumerate(height):
            writer.writerow([i, '%.3f' % h])

    east, north = to_local(lat, lon)
    with io.open(GPS_REFERENCE, 'w', encoding='utf-8', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(['time_s', 'latitude', 'longitude',
                         'east_m', 'north_m', 'height_m'])
        for i in range(len(t_rtk)):
            writer.writerow(['%.3f' % (t_rtk[i] - t_cam[0]),
                             '%.9f' % lat[i], '%.9f' % lon[i],
                             '%.3f' % east[i], '%.3f' % north[i],
                             '%.3f' % (alt[i] - ground)])

    path = float(np.hypot(*np.diff(np.c_[east, north], axis=0).T).sum())
    print('Входные файлы готовы:')
    print('  калибровка      %s  (f = %.1f px)' % (os.path.basename(CALIBRATION), focal))
    print('  высота по кадрам %s  (%.1f..%.1f м)'
          % (os.path.basename(ALTITUDE_FILE), height.min(), height.max()))
    print('  эталон GPS      %s  (%d точек, путь %.0f м)'
          % (os.path.basename(GPS_REFERENCE), len(t_rtk), path))
    return t_rtk - t_cam[0], east, north, alt - ground, t_cam - t_cam[0]


# =============================================================================
#  2. ПРОГОН МОДЕЛЕЙ
# =============================================================================
def run_models():
    """Последовательный прогон vo_final2.py всеми моделями движения."""
    common = ['--input', FRAMES,
              '--calibration', CALIBRATION,
              '--altitude-file', ALTITUDE_FILE,
              '--scale-correction', str(SCALE_CORRECTION),
              '--fps', str(FPS),
              '--algorithm', ALGORITHM,
              '--quiet']
    elapsed = {}
    for model in MOTION_MODELS:
        out = os.path.join(RESULTS, model)
        print('\n--- модель %s ---' % model)
        started = time.perf_counter()
        # Каждая модель пишет в СВОЮ папку: имя подпапки внутри out-dir
        # складывается из связки точек, а не из модели движения, поэтому
        # общий out-dir три прогона затёрли бы друг другом.
        code = subprocess.call(
            [sys.executable, os.path.join(ROOT, 'vo_final2.py')] + common
            + ['--motion-model', model, '--out-dir', out], cwd=ROOT)
        elapsed[model] = time.perf_counter() - started
        if code:
            print('[ОШИБКА] модель %s завершилась с кодом %d' % (model, code))
    return elapsed


# =============================================================================
#  3. ЗАМЕР ВРЕМЕНИ ПО ЭТАПАМ
# =============================================================================
def benchmark(frames=(1200, 1208, 1600, 1608, 2000, 2008, 2400, 2408),
              repeats=5):
    """
    Сколько стоит каждый этап на кадрах этого набора.

    Модели сравниваются на ОДНИХ И ТЕХ ЖЕ соответствиях, иначе сравнение
    нечестное: разные модели уводят адаптивный выбор кадров на разный шаг, и
    в общем времени прогона мешаются совсем другие величины.
    """
    import cv2
    sys.path.insert(0, ROOT)
    argv, sys.argv = sys.argv, ['run_all.py']
    import vo_final2 as vo
    sys.argv = argv

    cfg = vo.build_config()
    cfg['Geometry']['ScaleCorrection'] = SCALE_CORRECTION
    cfg['VO']['Algorithms'] = [ALGORITHM]
    intr = vo.Intrinsics(LENS_MM * 1e-3 / (PIXEL_UM * 1e-6),
                         LENS_MM * 1e-3 / (PIXEL_UM * 1e-6),
                         (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0,
                         (HEIGHT, WIDTH), None)
    runner = vo.AlgorithmRunner(vo.select_algorithms(cfg)[0], cfg)

    images = [cv2.imread(os.path.join(FRAMES, '%06d.jpg' % f),
                         cv2.IMREAD_GRAYSCALE) for f in frames]

    started = time.perf_counter()
    for _ in range(repeats):
        data = [vo.detect_and_describe(im, runner, cfg, None) for im in images]
    detect_ms = 1000 * (time.perf_counter() - started) / (repeats * len(images))

    started = time.perf_counter()
    for _ in range(repeats):
        for i in range(0, len(images), 2):
            vo.match_feature_sets(data[i], data[i + 1], cfg)
    match_ms = 1000 * (time.perf_counter() - started) / (repeats * len(images) // 2)

    pairs = []
    for i in range(0, len(images), 2):
        idx = vo.match_feature_sets(data[i], data[i + 1], cfg)['IndexPairs']
        pairs.append((data[i]['Points'][idx[:, 0]],
                      data[i + 1]['Points'][idx[:, 1]]))

    model_ms = {}
    for model in MOTION_MODELS:
        cfg['Ransac']['MotionModel'] = model
        for a, b in pairs:                       # прогрев
            vo.estimate_motion(a, b, intr, cfg, 80.0)
        started = time.perf_counter()
        for _ in range(repeats):
            for a, b in pairs:
                vo.estimate_motion(a, b, intr, cfg, 80.0)
        model_ms[model] = 1000 * (time.perf_counter() - started) / (repeats * len(pairs))

    return {'points': int(data[0]['NumValid']), 'matches': int(len(pairs[0][0])),
            'detect': detect_ms, 'match': match_ms, 'models': model_ms}


# =============================================================================
#  4. СВЕРКА С RTK
# =============================================================================
def load_track(model, t_cam):
    """Траектория модели: время кадров и координаты X, Y."""
    path = os.path.join(RESULTS, model, 'visual_odometry', ALGORITHM,
                        'trajectory.csv')
    frames, x, y = [], [], []
    with io.open(path, encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            # Неудачные шаги ('invalid_pose', 'too_few_inliers') положение не
            # меняют - их в траектории нет.
            if row['Status'].startswith(('origin', 'ok')):
                frames.append(int(row['Frame']) - 1)
                x.append(float(row['X']))
                y.append(float(row['Y']))
    return t_cam[np.array(frames)], np.c_[x, y]


def align(track, truth):
    """
    Совмещение поворотом и сдвигом. Масштаб НЕ подгоняется.

    Поворот нужен потому, что одометрия не знает, где север - её оси
    привязаны к первому кадру. А масштаб подгонять нельзя: он и есть
    проверяемая величина.
    """
    ct, cg = track.mean(0), truth.mean(0)
    u, _, vt = np.linalg.svd((track - ct).T @ (truth - cg))
    d = np.eye(2)
    if np.linalg.det(u @ vt) < 0:
        d[1, 1] = -1
    rot = (u @ d @ vt).T
    return (rot @ track.T).T + cg - (rot @ ct)


def read_attitude():
    """Крен и тангаж корпуса, градусы, из кватерниона DJI."""
    t, quat = [], []
    with io.open(ATTITUDE_CSV, encoding='utf-8') as fh:
        for row in csv.DictReader(fh):
            t.append(float(row['bag_time_ns']) * 1e-9)
            quat.append([float(row['quaternion.x']), float(row['quaternion.y']),
                         float(row['quaternion.z']), float(row['quaternion.w'])])
    t = np.array(t)
    x, y, z, w = np.array(quat).T
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return t - t[0], roll, pitch


def plot_profile(t_rtk, east, north, height, t_cam, plt):
    """
    Условия, в которых работала одометрия: высота, скорость, наклон корпуса и
    шаг, выбранный регулятором. По этому рисунку видно, почему планарная
    модель здесь не справилась.
    """
    fig, ax = plt.subplots(4, 1, figsize=(12.5, 11), sharex=True)

    ax[0].plot(t_rtk, height, 'k-')
    ax[0].set_ylabel('высота, м')
    ax[0].set_title('HKairport03: условия полёта и выбор кадров')

    speed = np.hypot(*np.diff(np.c_[east, north], axis=0).T) / np.diff(t_rtk)
    ax[1].plot(t_rtk[:-1], speed, 'k-')
    ax[1].set_ylabel('скорость, м/с')

    try:
        t_att, roll, pitch = read_attitude()
        ax[2].plot(t_att, roll, lw=.7, label='крен')
        ax[2].plot(t_att, pitch, lw=.7, label='тангаж')
        ax[2].legend(fontsize=8, loc='upper right')
        ax[2].text(0.01, 0.04,
                   'камера закреплена жёстко: 1 градус наклона = 25 px сдвига кадра',
                   transform=ax[2].transAxes, fontsize=8, color='tab:red')
    except (OSError, KeyError):
        ax[2].text(0.5, 0.5, 'нет файла углов', ha='center',
                   transform=ax[2].transAxes)
    ax[2].set_ylabel('наклон корпуса, град')

    names = {'homography': 'гомография', 'similarity': 'планарная'}
    for model, colour in (('homography', 'tab:green'),
                          ('similarity', 'tab:orange')):
        path = os.path.join(RESULTS, model, 'visual_odometry', ALGORITHM,
                            'trajectory.csv')
        if not os.path.exists(path):
            continue
        frames, steps = [], []
        with io.open(path, encoding='utf-8') as fh:
            for row in csv.DictReader(fh):
                if row['Status'].startswith('ok'):
                    frames.append(int(row['Frame']) - 1)
                    steps.append(float(row['FrameStep']))
        ax[3].plot(t_cam[np.array(frames)], steps, '.', ms=3, color=colour,
                   label=names[model])
    ax[3].legend(fontsize=8, loc='upper right')
    ax[3].set_ylabel('выбранный шаг, кадров')
    ax[3].set_xlabel('время, с')

    for a in ax:
        a.grid(alpha=.3)
        for moment in (60, 95, 310):
            a.axvline(moment, color='gray', ls='--', lw=.8)
    plt.tight_layout()
    plt.savefig(PLOT_PROFILE, dpi=115)
    plt.close()


def plot_by_model(tracks, east, north, plt):
    """Каждая модель отдельной панелью: на общем графике они накладываются."""
    titles = {'similarity': 'планарная', 'homography': 'гомография',
              'essential': 'существенная матрица'}
    colours = {'similarity': 'tab:orange', 'homography': 'tab:green',
               'essential': 'tab:red'}
    present = [m for m in MOTION_MODELS if m in tracks]
    if not present:
        return
    # Оси НЕ связываются через sharex/sharey: с ними matplotlib запрещает
    # axis('equal'), а равный масштаб по осям здесь обязателен - иначе
    # траектория выглядит растянутой. Одинаковые пределы ставятся вручную.
    fig, ax = plt.subplots(1, len(present), figsize=(5.4 * len(present), 6))
    if len(present) == 1:
        ax = [ax]
    for a, model in zip(ax, present):
        fitted, rms, ratio = tracks[model]
        a.plot(east, north, 'k-', lw=2.0, label='RTK')
        a.plot(fitted[:, 0], fitted[:, 1], '-', color=colours[model], lw=1.3,
               label='одометрия')
        a.plot(east[0], north[0], 'ko', ms=8)
        a.set_title('%s%sRMS %.0f м, путь %.2f от эталона'
                    % (titles[model], chr(10), rms, ratio))
        a.set_xlabel('восток, м')
        a.axis('equal')
        a.grid(alpha=.3)
        a.legend(fontsize=8, loc='lower left')
    ax[0].set_ylabel('север, м')
    lo = [min(a.get_xlim()[0] for a in ax), min(a.get_ylim()[0] for a in ax)]
    hi = [max(a.get_xlim()[1] for a in ax), max(a.get_ylim()[1] for a in ax)]
    for a in ax:
        a.set_xlim(lo[0], hi[0])
        a.set_ylim(lo[1], hi[1])
    plt.tight_layout()
    plt.savefig(PLOT_BY_MODEL, dpi=115)
    plt.close()


def compare(t_rtk, east, north, height, t_cam, timing, elapsed):
    truth_path = float(np.hypot(*np.diff(np.c_[east, north], axis=0).T).sum())

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        plt = None

    if plt is not None:
        fig, ax = plt.subplots(1, 2, figsize=(15.5, 7))
        ax[0].plot(east, north, 'k-', lw=2.4, zorder=5,
                   label='RTK (эталон), %.0f м' % truth_path)
        ax[0].plot(east[0], north[0], 'ko', ms=10, zorder=6)

    colours = {'homography': 'tab:green', 'similarity': 'tab:orange',
               'essential': 'tab:red'}
    titles = {'homography': 'гомография', 'similarity': 'планарная',
              'essential': 'существенная'}

    tracks = {}
    print('\n%-14s %8s %6s %8s %8s %9s %9s'
          % ('модель', 'путь,м', 'отн.', 'RMS,м', 'макс,м', 'замык,м', 'прогон,с'))
    print('-' * 72)
    summary = {}
    for model in MOTION_MODELS:
        if not os.path.isdir(os.path.join(RESULTS, model)):
            print('%-14s прогона нет' % titles[model])
            continue
        t, track = load_track(model, t_cam)
        truth = np.c_[np.interp(t, t_rtk, east), np.interp(t, t_rtk, north)]
        fitted = align(track, truth)
        err = np.hypot(*(fitted - truth).T)
        rms = float(np.sqrt((err ** 2).mean()))
        length = float(np.hypot(*np.diff(track, axis=0).T).sum())
        summary[model] = (length, length / truth_path, rms, float(err.max()),
                          float(np.hypot(*(track[-1] - track[0]))))
        tracks[model] = (fitted, rms, length / truth_path)
        print('%-14s %8.0f %6.2f %8.1f %8.1f %9.1f %9.0f'
              % (titles[model], length, length / truth_path, rms, err.max(),
                 np.hypot(*(track[-1] - track[0])), elapsed.get(model, 0.0)))
        if plt is not None:
            ax[0].plot(fitted[:, 0], fitted[:, 1], '-', color=colours[model],
                       lw=1.4, label='%s, RMS %.0f м' % (titles[model], rms))
            ax[1].plot(t, err, '-', color=colours[model], lw=1.5,
                       label=titles[model])

    print('\nэталон RTK: путь %.0f м, невязка замыкания %.1f м'
          % (truth_path, np.hypot(east[-1] - east[0], north[-1] - north[0])))

    print('\nВремя по этапам (связка %s, кадр %dx%d):' % (ALGORITHM, WIDTH, HEIGHT))
    print('  детектирование и дескрипторы  %6.1f мс на кадр (%d точек)'
          % (timing['detect'], timing['points']))
    print('  сопоставление дескрипторов    %6.1f мс на пару (%d соответствий)'
          % (timing['match'], timing['matches']))
    for model in MOTION_MODELS:
        print('  модель %-22s %6.1f мс на пару'
              % (titles[model], timing['models'][model]))

    if plt is not None:
        ax[0].set_title('HKairport03: ВО без коррекции по GPS/ИНС, связка ' + ALGORITHM)
        ax[0].set_xlabel('восток, м')
        ax[0].set_ylabel('север, м')
        ax[0].axis('equal')
        ax[0].grid(alpha=.3)
        ax[0].legend(loc='lower left', fontsize=9)
        for moment, label in ((60, 'конец подъёма'), (95, 'начало маршрута'),
                              (310, 'начало снижения')):
            ax[1].axvline(moment, color='gray', ls='--', lw=1)
            ax[1].text(moment + 3, ax[1].get_ylim()[1] * .95, label,
                       rotation=90, fontsize=8, color='gray', va='top')
        for a, b in TURNS:
            ax[1].axvspan(a, b, color='tab:blue', alpha=.10)
        ax[1].set_title('Ошибка положения во времени (синим - развороты)')
        ax[1].set_xlabel('время, с')
        ax[1].set_ylabel('ошибка, м')
        ax[1].grid(alpha=.3)
        ax[1].legend(fontsize=9)
        os.makedirs(PLOTS_DIR, exist_ok=True)
        plt.tight_layout()
        plt.savefig(PLOT, dpi=115)
        plt.close()
        plot_by_model(tracks, east, north, plt)
        plot_profile(t_rtk, east, north, height, t_cam, plt)
        print('\nГрафики:')
        for name in (PLOT, PLOT_BY_MODEL, PLOT_PROFILE):
            print('  ' + name)
    return summary


def main():
    print('=== HKairport03: все модели движения, связка %s ===\n' % ALGORITHM)
    t_rtk, east, north, height, t_cam = prepare()
    elapsed = run_models()
    print('\n--- замер времени по этапам ---')
    timing = benchmark()
    compare(t_rtk, east, north, height, t_cam, timing, elapsed)
    return 0


if __name__ == '__main__':
    sys.exit(main())
