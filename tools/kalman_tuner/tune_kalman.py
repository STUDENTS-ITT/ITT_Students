#!/usr/bin/env python3
"""
tune_kalman_v3.py — двухэтапный подбор коэффициентов фильтра Калмана (v3).

Этап 1. Глобальный поиск: равномерная сетка Sobol в логарифмическом
        пространстве u = log10(sigma/sigma_ref).  Ищем перспективные области.
Этап 2. Локальная полировка (Nelder-Mead) от лучших точек этапа 1, окрестности
        u=0 (B6) и архивного best того же набора данных (F8, warm-start).
Этап 3. B5: уточняющая сетка Sobol ±0.5 декады вокруг найденного бассейна +
        вторая полировка — подтверждение минимума и борьба с плато/ложными ямами.
Этап 4. E1b: центроид плато вокруг минимума — робастный финальный выбор.

Отличия от tune_kalman_v2.py:
  * метрика J_v3 = J_v2 + TAIL_WEIGHT·P99.9(pos@200 Гц)/допуск — штраф за
    редкие прострелы на тактах ИМУ (P95 к ним слеп); численно J_v3 НЕ сопоставим
    с J_v2 (A–D), P95-компоненты и хвостовые статистики сопоставимы поточечно;
  * оптимизация в log-пространстве (равномерно по порядкам величины);
  * метрика: P95 ошибки позиции в NED (через cos(lat)), скорости и курса,
    нормированные порогами допуска; выбросы не доминируют в среднем;
  * tie-break по Mean и регуляризация (штраф за удаление от σ_ref);
  * физически правдоподобные границы поиска (не дают «копии эталона»);
  * train/valid-разделение по времени (60/40) для контроля переобучения;
  * параллельные прогоны: каждый рабочий процесс пишет в свою папку tools,
    данные читаются только на чтение — без гонок и копирования;
  * горизонт оценки: исключён переходный участок (выставка/GPS-hold);
  * вывод раскладки J по компонентам (pos/vel/hdg — P95:допуск) для top-5
    этапа 1, лучшей точки --global-only и финальной точки этапа 2;
  * eval.json для каждой оценки (A4) — ландшафт J по шагу сетки после прогона;
  * откат к точке с лучшим J_valid при J_valid/J_train > 2.0 (B7);
  * вторая сетка вокруг бассейна (B5) с параметрами N_GLOBAL2/B5_SPAN;
#   * A2: ранний стоп сетки (EARLY_STOP_STALE точек без улучшения best_train);
#   * A3: run_eval возвращает статус сбоя, счётчик упавших — в лог этапа 1;
#   * B6: третий старт полировки в окрестности базовой u=0;
#   * B8: basinhopping --basinhopping вместо Nelder-Mead для «гребешков».

Запуск:  python tune_kalman_v3.py [--global-only] [--quick]
"""

import glob
import io
import json
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime

import numpy as np
from scipy.stats import qmc
from scipy.optimize import minimize
from multiprocessing import Pool

# ============================================================
# КОНФИГУРАЦИЯ
# ============================================================

DATA_DIR = r"C:\ITT_Students_Qt_Version\run_hk\data\raw"
if os.environ.get("TUNE_DATA_DIR"):
    DATA_DIR = os.environ["TUNE_DATA_DIR"]
BUILD_DIR = r"C:\ITT_Students_Qt_Version\build\src\console"
TARGET_EXE = "imitator.exe"

# v3: отдельный рабочий каталог в папке скрипта (не трогает temp_tune_v2).
WORK_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "temp_tune_v3")
# Дочерние процессы (multiprocessing spawn на Windows) переимпортируют модуль и
# не видят переопределённый в родителе WORK_ROOT. Пробрасываем его, как DATA_DIR,
# через переменную окружения — иначе оценки этапа 2 пишутся в каталог по умолчанию.
if os.environ.get("TUNE_WORK_ROOT"):
    WORK_ROOT = os.environ["TUNE_WORK_ROOT"]

# Параметры подбора: (имя, sigma_ref, полуширина диапазона u = log10(sigma/sigma_ref)).
# Полуширины ЗАДАЮТ ФИЗИЧЕСКИ ПРАВДОПОДОБНЫЙ диапазон (см. примечание ниже):
#   * σ СНС (pos/v/ang) — ±1 декада: R не может стать исчезающе малым,
#     иначе фильтр превращается в «копию эталона СНС» (метрика самоподтверждается,
#     т.к. эталон равен входному измерению).
#   * σ ИНС (g/a) — ±1.5 декады вокруг типового MEMS.
#   * скорости дрейфа смещений (bg/ba) — ±2 декады.
# Замечание (2026-09-20, поздний вечер): границы ВОЗВРАЩЕНЫ к варианту прогона A
# по указанию пользователя (кватернионы/ESKF-П4 отклонены). Промежуточные расширения
# (+0.2: до g/a 2.1, pos/ang 1.6, дрейфы 2.2) дали J_train 0.21, но уводили в «копию
# эталона СНС» (IMPROVEMENTS.md, F1/F3).
# 2026-09-21: по указанию пользователя выполняется ПОВТОР ПРОГОНА D — границы
# возвращены к варианту D (g/a ±2.1, СНС pos/ang ±1.6, СНС v ±1.2, дрейфы ±2.2):
# восстановление результатов D и контроль прострелов (docs/tuning_runs_report.md).
# 2026-09-24: по указанию пользователя — возврат к границам ПРОГОНА А (g/a ±1.5,
# СНС pos/v/ang ±1.0, дрейфы bg/ba ±2.0) для v3-прогона и сравнения v3(A) vs v3(D)
# в tune_kalman_v3/tuning_runs_comparison.md. Контроль выполнен; по указанию
# пользователя границы ВОЗВРАЩЕНЫ к варианту D (g/a ±2.1, СНС pos/ang ±1.6,
# СНС v ±1.2, дрейфы ±2.2) — финальная конфигурация v3.
# 2026-09-27 (вечер): по указанию пользователя интервалы ВСЕХ σ «немного» расширены
# на +0.3 (g/a ±2.4, pos/ang ±1.9, v ±1.5, bg/ba ±2.5) — см. IMPROVEMENTS.md, F1.
# 2026-09-27 (ночь): расширение прогона №3 дало прострелы (MAX pos 2.85 м на valid,
# tail valid 2.795 м, «копия СНС»); по указанию пользователя ВОЗВРАЩЕНЫ границы
# ПРОГОНА D (MAX 1.05 м) — g/a ±2.1, СНС pos/ang ±1.6, СНС v ±1.2, дрейфы ±2.2.
# 2026-09-27 (второй раунд, по указанию пользователя): расширены ТОЛЬКО интервалы
# с уперевшимися в границу параметрами — sigma_g/sigma_a на верхней стене (+2.1)
# до ±2.4; остальные σ остаются в варианте D. Опасность: направление «копия СНС»
# (F1) — верхняя стена Q для ИНС-шумов; контроль — tail-метрикой v3, B7/E1b и
# разрывом train/valid (см. прогон №3).
# 2026-09-27 (третий раунд, по указанию пользователя): прогон №5 (g/a ±2.4) дал
# доминирование над D (J 0.2636/0.2670, MAX 0.81 м, точек>1 м: 0), но sigma_a снова
# у стенки (2.400). Те же границы g/a увеличены ещё на +0.3 до ±2.7; остальные σ
# остаются в варианте D.
# 2026-09-27 (четвёртый раунд, по указанию пользователя): в прогоне №6 (g/a ±2.7,
# J 0.1768/0.1835) в стену упёрлась ТОЛЬКО sigma_a (2.700); sigma_g сошла со стены
# внутрь (1.797). Расширен интервал sigma_a до ±3.0; sigma_g и остальные σ —
# без изменений.
PARAMS = [
    ("sigma_g",   3.394e-4, 2.7),   # шум гироскопа, рад/√с
    ("sigma_a",   3.05e-3,  3.0),   # шум акселерометра, м/с²/√с
    ("sigma_bg",  1.16e-5,  2.2),   # дрейф смещения гироскопа, рад/с/√с
    ("sigma_ba",  2.0e-5,   2.2),   # дрейф смещения акселерометра, м/с²/√с
    ("sigma_pos", 7.85e-7,  1.6),   # СНС позиция, рад
    ("sigma_h",   5.0,      1.6),   # СНС высота, м (8-й коэффициент, в v3 не подбирался)
    ("sigma_v",   0.1,      1.2),   # СНС скорость, м/с
    ("sigma_ang", 0.0175,   1.6),   # СНС курс, рад
]

def rebuild_bounds():
    """Пересобирает асимметричные u-границы U_LO/U_HI из глобального PARAMS.

    Элемент PARAMS: (name, sigma_ref, span). span может быть:
      * скаляром s — симметричный диапазон [-s, +s];
      * парой (lo, hi) — асимметричный диапазон.
    """
    global U_LO, U_HI
    lo, hi = [], []
    for _, _, s in PARAMS:
        if isinstance(s, (list, tuple)) and len(s) == 2:
            lo.append(min(s[0], s[1])); hi.append(max(s[0], s[1]))
        else:
            lo.append(-float(s)); hi.append(float(s))
    U_LO = np.array(lo, dtype=float)
    U_HI = np.array(hi, dtype=float)


rebuild_bounds()

# Пороги допуска для нормировки ошибок (метрика J = sum err_i / tol_i).
TOL_POS = 5.0      # м
TOL_VEL = 0.5      # м/с
TOL_HDG = 1.0      # °

# Вторичный критерий (tie-break): к J прибавляется малая доля Mean-ошибки.
TIE_WEIGHT = 0.05

# v3 (шаг 1): штраф за «хвост» прострелов на тактах ИМУ (200 Гц).
# P95 на тактах СНС слеп к редким выбросам: прострел длится доли секунды и
# в 5 Гц-выборку почти не попадает (прогон D: пик 26 м, 0.16% отсчётов).
# Добавляем высокий квантиль планарной ошибки позиции P99.9, посчитанный
# на каждом такте result.txt (интерполяция reference на время такта), с весом
# TAIL_WEIGHT. Он «видит» 26-метровые пики и делает «копию эталона» на стенках
# невыгодной (docs/tuning_runs_report.md, §6).
TAIL_Q = 99.9
TAIL_WEIGHT = 0.5

# 13C: согласованность/робастность хвоста.
#   * WINDOW_S — длина окна (с) для оконного хвоста: series 200 Гц режется на
#     окна, в каждом берётся квантиль TAIL_Q; берём худшее окно по valid. Так
#     локальный артефакт (напр. 138 м на square_650 ~t=638.3) видно отдельно от
#     «среднего» фона, и метрика устойчива, если выброс разовый.
#   * CONSISTENCY_WEIGHT — вес члена J по худшему окну (доля от P95-нормировки);
#     0.0 — член выключен, J бит-в-бит совпадает с прежним.
#   * OUTLIER_POS_M — порог (м) для доли выбросов среди тактов ИМУ (метрика).
CONSISTENCY_WEIGHT = 0.0
WINDOW_S = 5.0
OUTLIER_POS_M = 1.0

# Регуляризация: J' = J + REG_LAMBDA * Σ log10(σᵢ/σ_ref)².
WALL_PENALTY_W = 0.1
WALL_PENALTY_EPS_FRAC = 0.05
REG_LAMBDA = 0.005


# Разбиение горизонта: доля, отбрасываемая в начале (выставка).
SKIP_FIRST_S = 30.0
# Доля горизонта, используемая для оптимизации (train); остальное — valid.
TRAIN_FRAC = 0.6

N_GLOBAL = 64         # точек сетки Sobol на этапе 1
N_LOCAL = 80          # оценок Nelder-Mead на этапе 2
N_WORKERS = 8         # параллельных прогонов
RUN_TIMEOUT = 300     # с на один прогон imitator
N_GLOBAL2 = 32        # B5: точек уточняющей сетки вокруг бассейна
B5_SPAN = 0.5         # B5: полуширина сжатия второй сетки, декады по u

SEED = 42
BAD_ERROR = 1e9

# B7: допустимое соотношение J_valid/J_train. Превышение — подозрение на переобучение:
# откат к точке с лучшим J_valid среди кандидатов (этапы 1+2).
VALID_OVFIT_RATIO = 2.0

# A2: ранний стоп сетки — число подряд успешных точек без улучшения best_train.
EARLY_STOP_STALE = 16
# B6: полуширина окрестности базовой u=0 для третьего старта полировки, декады.
NEAR_ZERO_SPAN = 0.2
# B8: метод локальной полировки ("nm" — Nelder-Mead, "bh" — basinhopping).
POLISH_METHOD = "nm"
BH_NITER = 2          # туров basinhopping (помимо начальной NM-экскурсии)
BH_STEPSIZE = 0.5     # шаг случайного переноса бассейна, декады по u

# E1b: центроид «плато» вместо одиночной итоговой точки (см. plateau_centroid).
# J — один скаляр на 7 параметров при ступенчатом P95, поэтому вокруг минимума
# обычно существует целое «плато» почти равных J; медиана плато как результат
# робастнее точки, до которой случайно довёл Nelder-Mead (край плато).
PLATEAU_SCAN_HALF = 0.3  # полуширина координатного скана вокруг минимума, декады u
PLATEAU_SCAN_STEP = 0.1  # шаг скана по каждой оси
PLATEAU_BAND = 0.05      # полоса «плато»: J_train <= J_min + band, абс. по J


# ============================================================
# МЕТРИКА ОШИБКИ
# ============================================================

def _load_skip_header(path):
    try:
        return np.loadtxt(path, skiprows=1)
    except Exception:
        pass
    try:
        return np.loadtxt(path)
    except Exception:
        pass
    try:
        data = np.genfromtxt(io.StringIO(
            "\n".join(l for l in open(path, encoding="utf-8", errors="replace")
                      if not l.startswith(("time", "#", "@")))))
        if data.ndim == 2 and data.shape[1] >= 10:
            return data
    except Exception:
        pass
    return None


def _windowed_q(t, x, win_s, q):
    """Худший и средний квантиль q по окнам длины win_s для ряда (t, x).

    Возвращает (worst, mean, n_win). Пустые/короткие окна (<5 отсчётов)
    отбрасываются. Служит для 13C: локальный хвост виден отдельно от фона.
    """
    if len(t) < 2:
        return 0.0, 0.0, 0
    widx = ((t - t[0]) // win_s).astype(int)
    vals = []
    for w in np.unique(widx):
        seg = x[widx == w]
        if len(seg) >= 5:
            vals.append(float(np.percentile(seg, q)))
    if not vals:
        return 0.0, 0.0, 0
    return float(np.max(vals)), float(np.mean(vals)), len(vals)


def _frac_above(x, thr):
    """Доля отсчётов строго выше порога (для доли выбросов, 13C)."""
    return float(np.mean(x > thr)) if len(x) else 0.0


def _moments(x):
    """(mean, std, skew, kurtosis) — моменты распределения ошибки позиции.

    Куртозис — обычный (для нормального ≈ 3.0); заострённый хвост > 3.
    """
    if len(x) < 2:
        return 0.0, 0.0, 0.0, 0.0
    mu = float(np.mean(x))
    sd = float(np.std(x))
    if sd <= 0.0:
        return mu, sd, 0.0, 0.0
    z = (x - mu) / sd
    return mu, sd, float(np.mean(z ** 3)), float(np.mean(z ** 4))


def compute_error(result_path, reference_path, skip_first_s, train_frac):
    """Возвращает (err_train, err_valid, summary).

    err_train/valid — J с tie-break и регуляризацией.
    summary — P95 и Mean по train и valid.
    """
    res = _load_skip_header(result_path)
    ref = _load_skip_header(reference_path)
    if res is None or ref is None or res.ndim != 2 or ref.ndim != 2:
        return BAD_ERROR, BAD_ERROR, None

    # Столбцы: 0 time, 1 lon, 2 lat, 3 alt, 4 heading, 5 pitch, 6 roll,
    #          7 vn, 8 vh, 9 ve.
    t_ref = ref[:, 0]
    t_res = res[:, 0]
    if len(t_ref) < 10 or len(t_res) < 10:
        return BAD_ERROR, BAD_ERROR, None

    # Для каждого времени reference берём ближайшую строку result.
    idx = np.searchsorted(t_res, t_ref)
    idx = np.clip(idx, 0, len(t_res) - 1)
    idx_prev = np.clip(idx - 1, 0, len(t_res) - 1)
    closer = np.abs(t_res[idx] - t_ref) <= np.abs(t_res[idx_prev] - t_ref)
    idx = np.where(closer, idx, idx_prev)

    r = res[idx]
    lat_ref = ref[:, 2]
    lon_ref = ref[:, 1]

    d_north = (r[:, 2] - lat_ref) * 111000.0
    d_east = (r[:, 1] - lon_ref) * 111000.0 * np.cos(np.deg2rad(lat_ref))
    pos_err = np.hypot(d_north, d_east)

    vel_err = np.hypot(r[:, 7] - ref[:, 7], r[:, 9] - ref[:, 9])

    hdg = np.mod(r[:, 4] - ref[:, 4] + 180.0, 360.0) - 180.0
    hdg_err = np.abs(hdg)

    # Горизонт: отбрасываем переходный участок.
    keep = t_ref >= skip_first_s
    if keep.sum() < 20:
        return BAD_ERROR, BAD_ERROR, None

    tt = t_ref[keep]
    pe, ve, he = pos_err[keep], vel_err[keep], hdg_err[keep]

    def p95(x):
        return float(np.percentile(x, 95)) if len(x) else 0.0

    def mean(x):
        return float(np.mean(x)) if len(x) else 0.0

    def J_parts(pe_, ve_, he_):
        return (p95(pe_) / TOL_POS, p95(ve_) / TOL_VEL, p95(he_) / TOL_HDG), \
               (mean(pe_) / TOL_POS, mean(ve_) / TOL_VEL, mean(he_) / TOL_HDG)

    split = tt[0] + (tt[-1] - tt[0]) * train_frac
    m = tt <= split
    if m.sum() < 10 or (~m).sum() < 10:
        return BAD_ERROR, BAD_ERROR, None

    (p_tr, v_tr, h_tr), (pm_tr, vm_tr, hm_tr) = J_parts(pe[m], ve[m], he[m])
    (p_va, v_va, h_va), (pm_va, vm_va, hm_va) = J_parts(pe[~m], ve[~m], he[~m])

    # Простая сумма частей: J = Σ P95_i / tol_i.
    def Jsum(parts):
        return parts[0] + parts[1] + parts[2]

    def tie(means):
        return TIE_WEIGHT * (means[0] + means[1] + means[2])

    err_train = Jsum((p_tr, v_tr, h_tr)) + tie((pm_tr, vm_tr, hm_tr))
    err_valid = Jsum((p_va, v_va, h_va)) + tie((pm_va, vm_va, hm_va))

    # ----- v3 (шаг 1): штраф за «хвост» на тактах ИМУ (200 Гц) -----
    # Интерполируем reference (5 Гц) на времена каждого такта result (200 Гц)
    # по lat/lon и берём планарную ошибку — P95 слеп к редким прострелам.
    lat_ref_i = np.interp(t_res, t_ref, ref[:, 2])
    lon_ref_i = np.interp(t_res, t_ref, ref[:, 1])
    d_n200 = (res[:, 2] - lat_ref_i) * 111000.0
    d_e200 = (res[:, 1] - lon_ref_i) * 111000.0 * np.cos(np.deg2rad(lat_ref_i))
    pos200 = np.hypot(d_n200, d_e200)
    t200 = t_res[t_res >= skip_first_s]
    p200 = pos200[t_res >= skip_first_s]
    m200 = t200 <= split

    def ptail(x):
        return float(np.percentile(x, TAIL_Q)) if len(x) else 0.0

    tail_tr = TAIL_WEIGHT * ptail(p200[m200]) / TOL_POS
    tail_va = TAIL_WEIGHT * ptail(p200[~m200]) / TOL_POS
    err_train = err_train + tail_tr
    err_valid = err_valid + tail_va

    # ----- 13C: оконный хвост + доля выбросов + моменты распределения -----
    # Оконный квантиль по тактам ИМУ (200 Гц) отдельно для train/valid.
    win_tr_worst, win_tr_mean, win_tr_n = _windowed_q(t200[m200], p200[m200],
                                                      WINDOW_S, TAIL_Q)
    win_va_worst, win_va_mean, win_va_n = _windowed_q(t200[~m200], p200[~m200],
                                                      WINDOW_S, TAIL_Q)
    out_tr = _frac_above(p200[m200], OUTLIER_POS_M)
    out_va = _frac_above(p200[~m200], OUTLIER_POS_M)
    mom_tr = _moments(p200[m200])
    mom_va = _moments(p200[~m200])

    if CONSISTENCY_WEIGHT > 0.0:
        # Худшее окно нормируется допуском позиции; штраф за локальный хвост.
        err_train = err_train + CONSISTENCY_WEIGHT * win_tr_worst / TOL_POS
        err_valid = err_valid + CONSISTENCY_WEIGHT * win_va_worst / TOL_POS

    summary = {
        "n_train": int(m.sum()),
        "n_valid": int((~m).sum()),
        "p95_pos_train": p95(pe[m]),
        "p95_pos_valid": p95(pe[~m]),
        "p95_vel_train": p95(ve[m]),
        "p95_vel_valid": p95(ve[~m]),
        "p95_hdg_train": p95(he[m]),
        "p95_hdg_valid": p95(he[~m]),
        "mean_pos_train": mean(pe[m]),
        "mean_pos_valid": mean(pe[~m]),
        "tail_pos_train": ptail(p200[m200]),
        "tail_pos_valid": ptail(p200[~m200]),
        # 13C: робастность хвоста.
        "win_q_train": win_tr_worst,
        "win_q_valid": win_va_worst,
        "win_q_mean_train": win_tr_mean,
        "win_q_mean_valid": win_va_mean,
        "n_win_train": win_tr_n,
        "n_win_valid": win_va_n,
        "outlier_frac_train": out_tr,
        "outlier_frac_valid": out_va,
        "outlier_thr_m": OUTLIER_POS_M,
        "kurt_pos_train": mom_tr[3],
        "kurt_pos_valid": mom_va[3],
        "skew_pos_train": mom_tr[2],
        "skew_pos_valid": mom_va[2],
    }
    return err_train, err_valid, summary


# ============================================================
# ОДИН ПРОГОН (вызывается из рабочих процессов)
# ============================================================

def run_eval(u_vec, work_dir):
    """u_vec — вектор log10(sigma/sigma_ref).

    Возвращает (err_train, err_valid, summary, status):
      * status=None при успешной оценке;
      * иначе короткая строка причины сбоя (A3): returncode≠0, отсутствие
        файлов result.txt/reference.txt, таймаут/исключение.
    """
    # Собираем params.ini из log-параметров.
    lines = ["# params.ini (сгенерировано tune_kalman_v3.py)"]
    sigma = []
    for (name, ref, _), u in zip(PARAMS, u_vec):
        s = ref * (10.0 ** float(u))
        sigma.append(s)
        lines.append(f"{name} = {s:.6e}")
    os.makedirs(work_dir, exist_ok=True)
    config_path = os.path.join(work_dir, "params.ini")
    with open(config_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    tools = os.path.join(work_dir, "tools")
    os.makedirs(tools, exist_ok=True)

    exe = os.path.join(BUILD_DIR, TARGET_EXE)
    try:
        proc = subprocess.run(
            [exe, DATA_DIR, config_path],
            cwd=work_dir,
            capture_output=True,
            text=True,
            timeout=RUN_TIMEOUT,
        )
    except Exception as exc:
        return BAD_ERROR, BAD_ERROR, None, f"искл/таймаут: {exc.__class__.__name__}"

    if proc.returncode != 0:
        return BAD_ERROR, BAD_ERROR, None, f"rc={proc.returncode}"

    result_path = os.path.join(tools, "result.txt")
    reference_path = os.path.join(tools, "reference.txt")
    if os.environ.get("TUNE_REF_TRUTH") == "1":
        # Наземная истина синтетических наборов. Историческое имя —
        # reference_truth.txt; штатно в наборе лежит truth.txt (этап 13A).
        reference_path = None
        for cand in ("reference_truth.txt", "truth.txt"):
            truth_ref = os.path.join(DATA_DIR, cand)
            if os.path.exists(truth_ref):
                reference_path = truth_ref
                break
        if reference_path is None:
            # Истину просили явно, но файла нет — не подменяем её молча СНС.
            return BAD_ERROR, BAD_ERROR, None, "нет truth.txt в наборе"
    if not os.path.exists(result_path) or not os.path.exists(reference_path):
        return BAD_ERROR, BAD_ERROR, None, "нет result/reference.txt"

    err_train, err_valid, summary = compute_error(result_path, reference_path,
                                                  SKIP_FIRST_S, TRAIN_FRAC)

    # Регуляризация: штраф за «нефизичные» σ (далеко от σ_ref).
    reg = REG_LAMBDA * float(np.sum(np.square(u_vec)))
    wpen = 0.0
    epsf = WALL_PENALTY_EPS_FRAC
    ww = WALL_PENALTY_W
    for i, (name, ref, span) in enumerate(PARAMS):
        u = u_vec[i]
        lo = float(U_LO[i]); hi = float(U_HI[i])
        width = hi - lo
        if width <= 0.0:
            # 13G: фиксированная ось (нулевая ширина) — штрафа за стенку нет.
            continue
        eps = epsf * width
        if u < lo + eps: wpen += ww * ((lo + eps) - u)**2
        if u > hi - eps: wpen += ww * (u - (hi - eps))**2
    reg += wpen
    if summary is not None:
        err_train = err_train + reg
        err_valid = err_valid + reg
        summary["reg"] = reg

    # A4: eval.json для ЛЮБОЙ оценки (сетка этапа 1, полировка, чувствительность).
    # Позволяет после прогона строить «J vs шаг сетки» и анализировать ландшафт.
    if summary is not None:
        with open(os.path.join(work_dir, "eval.json"), "w", encoding="utf-8") as f:
            json.dump({"u": [float(x) for x in u_vec],
                       "J_train": err_train, "J_valid": err_valid,
                       "summary": summary}, f, ensure_ascii=False, indent=1)
    return err_train, err_valid, summary, None


def worker(args):
    u_vec, work_dir = args
    return work_dir, run_eval(u_vec, work_dir)


# ============================================================
# ЭТАП 1: ГЛОБАЛЬНЫЙ ПОИСК (Sobol) В LOG-ПРОСТРАНСТВЕ
# ============================================================

def sobol_points(n, d, seed):
    sampler = qmc.Sobol(d=d, scramble=True, seed=seed)
    points = sampler.random(n)
    lower = np.array(U_LO)
    upper = np.array(U_HI)
    return lower + points * (upper - lower)


def global_search(n_points, pool, center=None, span=None, early_stop=EARLY_STOP_STALE):
    """Сетка Sobol + базовая точка (u=0). Если заданы center/span —
    B5: уточняющая сетка, сжатая вокруг найденного бассейна.

    Возвращает (results, meta):
      * results — list (J_train, J_valid, summary, work_dir, status);
      * meta — сводка: n_done/n_fail/n_skipped/early_stopped/best_train.

    A2: ранний стоп — если best_train не улучшался `early_stop` успешных точек
    подряд, оценивание сетки прекращается. Чтобы не «разослать» уже-не-нужные
    задания, сетка идёт батчами по числу воркеров через pool.map (иначе
    imap_unordered продолжает раздавать точки в фоне даже после break).
    A3: статусы сбоев (rc/нет файлов/таймаут) подсчитываются и печатаются.
    """
    d = len(PARAMS)
    if center is None:
        grid = sobol_points(n_points, d, SEED)
        grid = np.vstack([np.zeros(d), grid])
        tag = "Этап 1"
    else:
        lo = np.clip(np.array(center) - span, U_LO, U_HI)
        hi = np.clip(np.array(center) + span, U_LO, U_HI)
        sampler = qmc.Sobol(d=d, scramble=True, seed=SEED)
        pts = sampler.random(n_points)
        grid = lo + pts * (hi - lo)
        grid = np.vstack([np.array(center), grid])
        tag = f"B5: уточняющая сетка ±{span} вокруг бассейна"
    prefix = "global" if center is None else "global2"

    print(f"\n=== {tag}, {len(grid)} точек Sobol "
          f"(ранний стоп после {early_stop} нейтральных) ===")
    t0 = time.time()
    results = []
    n_fail = 0
    n_done = 0
    total = len(grid)
    best_train = float("inf")
    stale = 0
    early = False
    batch = max(1, getattr(pool, "processes", 4) or 4)
    for i in range(0, total, batch):
        chunk = grid[i:i + batch]
        tasks = [(list(u), os.path.join(WORK_ROOT, f"{prefix}_{i + k:03d}"))
                 for k, u in enumerate(chunk)]
        for wrk_dir, (err_train, err_valid, summary, status) in pool.map(worker, tasks):
            n_done += 1
            results.append((err_train, err_valid, summary, wrk_dir, status))
            if status is None:
                if err_train < best_train - 1e-12:
                    best_train = err_train
                    stale = 0
                else:
                    stale += 1
            else:
                n_fail += 1
        print(f"   [{n_done}/{total}] best_train={best_train:.4f} "
              f"(упало: {n_fail}, {(time.time() - t0):.0f} с)")
        if early_stop and stale >= early_stop and n_done < total:
            print(f"   Early-stop (A2): best_train не улучшался {stale} точек подряд — "
                  f"остальные {total - n_done} точек сетки пропущены.")
            early = True
            break
    print(f"   Завершено за {(time.time() - t0):.0f} с "
          f"({n_done}/{total} точек, упало {n_fail})")
    return results, {"n_done": n_done, "n_fail": n_fail,
                     "n_skipped": total - n_done,
                     "early_stopped": early, "best_train": best_train}


# ============================================================
# ЭТАП 2: ЛОКАЛЬНАЯ ПОЛИРОВКА
# Запускается как отдельная задача в пуле (по одной на рестарт),
# внутри оценивает imitator.exe последовательно в собственном cwd.
# method="nm" — Nelder-Mead, method="bh" — basinhopping (B8).
# ============================================================

def local_polish(u_start, max_evals, tag, method=None):
    method = method or POLISH_METHOD
    d = len(PARAMS)
    bounds = list(zip(U_LO, U_HI))
    lo = np.array([b[0] for b in bounds])
    hi = np.array([b[1] for b in bounds])
    u_start = list(np.clip(u_start, lo, hi))
    best = {"u": list(u_start), "J": BAD_ERROR, "valid": BAD_ERROR, "summary": None, "wrk": None}
    counter = {"n": 0}
    cache = {}

    def objective(u):
        u = np.clip(u, lo, hi)
        key = tuple(round(x, 8) for x in u)
        if key in cache:
            return cache[key]
        counter["n"] += 1
        work_dir = os.path.join(WORK_ROOT, f"local_{tag}_{counter['n']:03d}")
        J, Jv, summary, _status = run_eval(u, work_dir)
        cache[key] = J
        if J < best["J"]:
            best["J"] = J
            best["valid"] = Jv
            best["u"] = list(u)
            best["summary"] = summary
            best["wrk"] = work_dir
            print(f"   [{tag}] лучше J={J:.4f} (valid {Jv:.4f}), оценок {counter['n']}")
        return J

    print(f"   [{tag}] полировка ({method}) от {np.round(u_start, 3).tolist()}, "
          f"до {max_evals} оценок")
    if method == "bh":
        from scipy.optimize import basinhopping
        rng = np.random.default_rng(SEED + 3033)
        min_iter = max(5, max_evals // (BH_NITER + 2))
        def take_step(x):
            return np.clip(x + rng.uniform(-BH_STEPSIZE, BH_STEPSIZE, size=d), lo, hi)
        bh = basinhopping(
            objective, np.array(u_start),
            niter=BH_NITER,
            stepsize=BH_STEPSIZE,
            take_step=take_step,
            seed=SEED + 7,
            minimizer_kwargs={
                "method": "Nelder-Mead",
                "bounds": bounds,
                "options": {"maxiter": min_iter, "xatol": 1e-4, "fatol": 1e-5},
            },
        )
        print(f"   [{tag}] завершена: J_train={bh.fun:.4f}, best J_valid={best['valid']:.4f}")
    else:
        res = minimize(
            objective, np.array(u_start),
            method="Nelder-Mead",
            bounds=bounds,
            options={"maxiter": max_evals, "xatol": 1e-6, "fatol": 1e-5},
        )
        print(f"   [{tag}] завершена: J_train={res.fun:.4f}, best J_valid={best['valid']:.4f}")
    return best


def polish_worker(args):
    u_start, tag, method = args
    return local_polish(u_start, N_LOCAL, tag, method=method)


def near_zero_start(span=NEAR_ZERO_SPAN):
    """B6: детерминированная стартовая точка в окрестности базовой u=0 —
    вдруг ручные параметры σ_ref сидят в другом, не покрытом сеткой бассейне."""
    rng = np.random.default_rng(SEED + 101)
    return rng.uniform(-span, span, size=len(PARAMS))


def warm_start_u():
    """F8: u-вектор лучшего (по J_train) архивного `best_params.txt` v3-прогона
    ТОГО ЖЕ набора данных. None, если таких архивов нет (или данные другие).

    Мотивация (прогон 2026-09-27): Sobol + Nelder-Mead на ступенчатой P95-метрике
    детерминированы, но к узкой boundary-нише могут не вернуться — известный
    бассейн D (J 0.355) остался не найден, поиск лёг в плато 0.53. Полировка от
    архивного best гарантирует монотонный прогресс между прогонами и возврат к
    лучшей известной точке. Отбираются только v3-архивы (признак — строка
    "tail pos (P99"), чтобы не смешивать метрику J с v2 (tail-член меняет оптимум).
    """
    param_names = {p[0] for p in PARAMS}
    found = []
    # База поиска archives — каталог репозитория (через __file__), а не cwd:
    # запуск скрипта возможен из любого места (DATA_DIR/BUILD_DIR абсолютные),
    # а warm-start обязан находить results/ независимо от рабочего каталога.
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for path in glob.glob(os.path.join(base, "results", "run_*",
                                       "best_params.txt")):
        archive_data = None
        j_train = None
        u_by_name = {}
        is_v3 = False
        try:
            with open(path, encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("данные"):
                        archive_data = line.split(":", 1)[1].strip()
                    elif line.startswith("tail pos (P99"):
                        is_v3 = True
                    elif line.startswith("J_train"):
                        m = re.search(r"([0-9]+\.[0-9]+)", line)
                        if m:
                            j_train = float(m.group(1))
                    parts = line.split()
                    if len(parts) == 3 and parts[0] in param_names:
                        try:
                            u_by_name[parts[0]] = float(parts[2])
                        except ValueError:
                            pass
        except OSError:
            continue
        if not (is_v3 and j_train is not None
                and os.path.normcase(archive_data)
                == os.path.normcase(os.path.abspath(DATA_DIR))
                and len(u_by_name) == len(PARAMS)):
            continue
        u = [u_by_name[name] for name, _, _ in PARAMS]
        found.append((j_train, u, path))
    if not found:
        return None
    found.sort(key=lambda r: r[0])
    j_train, u, path = found[0]
    u = np.clip(u, list(U_LO), list(U_HI))
    print(f"   F8: warm-start от архивного best {path} "
          f"(J_train={j_train:.4f}, данные совпадают)")
    return list(u), j_train


# ============================================================
# E5: ЧУВСТВИТЕЛЬНОСТЬ J К ПАРАМЕТРАМ + ГРАФИКИ
# ============================================================

SENS_DELTA = 0.15   # шаг по u = log10(σ/σ_ref) в одну сторону
PLOT_SCRIPT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "plot_trajectory.py"))
PLOT_ERRORS_SCRIPT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "plot_errors.py"))


def run_sensitivity(pool, u_base):
    """Оценка J вокруг u_base: базовая точка и ±δ по каждому параметру
    (шаг наружу за границу не выходит — клиппится, что тоже информативно)."""
    d = len(PARAMS)
    bounds = list(zip(U_LO, U_HI))
    points = [("base", 0, np.array(u_base))]
    for i in range(d):
        for sign in (-1.0, 1.0):
            u = np.array(u_base, dtype=float)
            u[i] = np.clip(u[i] + sign * SENS_DELTA, bounds[i][0], bounds[i][1])
            if not np.isclose(u[i], u_base[i]):
                points.append((PARAMS[i][0], sign, u))
    tasks = [(list(u), os.path.join(WORK_ROOT, f"sens_{k:03d}"))
             for k, (_, _, u) in enumerate(points)]
    results = list(pool.imap(worker, tasks))
    records = []
    for (name, sign, u), (_, (_Jt, _Jv, _summary, _status)) in zip(points, results):
        records.append({"name": name, "sign": sign, "u": list(u),
                        "J_train": _Jt, "J_valid": _Jv,
                        "status": _status})
    base = next(r for r in records if r["name"] == "base")
    print("\n=== E5: чувствительность J к параметрам "
          f"(шаг {SENS_DELTA:.2f} по u) ===")
    print(f"   база (лучшая точка): J_train={base['J_train']:.4f}, "
          f"J_valid={base['J_valid']:.4f}")
    for i, (name, _, _) in enumerate(PARAMS):
        pert = [r for r in records if r["name"] == name]
        avail = {r["sign"] for r in pert}
        side = ("внутрь(к базе)=" if -1.0 in avail else "") \
             + (" наружу=" if 1.0 in avail else "")
        line = f"   {name:10s}{side}"
        for sign in sorted(avail, reverse=True):
            r = next(r for r in pert if r["sign"] == sign)
            line += f" J_train={r['J_train']:.4f} J_valid={r['J_valid']:.4f} |"
        if len(avail) < 2:
            lo = np.isclose(pert[0]["u"][i], list(U_LO)[i], atol=1e-9)
            hi = np.isclose(pert[0]["u"][i], list(U_HI)[i], atol=1e-9)
            if lo or hi:
                line += " (у границы, обратное направление невозможно)"
        print(line)
    return records


def plot_sensitivity(records, save_path):
    """Столбчатый график: J_train/J_valid в базовой точке и при доступных ±δ."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    base = next(r for r in records if r["name"] == "base")
    labels, train, valid = [], [], []
    for i, (name, _, _) in enumerate(PARAMS):
        for r in (r for r in records if r["name"] == name):
            side = "вниз" if r["sign"] < 0 else "вверх"
            labels.append(f"{name} {side}")
            train.append(r["J_train"])
            valid.append(r["J_valid"])

    x = np.arange(len(labels))
    w = 0.4
    fig, ax = plt.subplots(figsize=(13, 5.5), dpi=150)
    ax.bar(x - w / 2, train, w, label="J_train", color="#0072bd")
    ax.bar(x + w / 2, valid, w, label="J_valid", color="#d95319")
    ax.axhline(base["J_train"], color="#0072bd", ls="--", lw=0.8, alpha=0.6)
    ax.axhline(base["J_valid"], color="#d95319", ls="--", lw=0.8, alpha=0.6)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=7, rotation=30, ha="right")
    ax.set_ylabel("J = P95/tol + tie + reg")
    ax.set_title("E5: чувствительность J к параметрам (шаг по u)")
    ax.grid(True, axis="y", alpha=0.35)
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path)
    plt.close(fig)
    return save_path


def plot_comparison(best):
    """Переиспользует tools/plot_trajectory.py: Калман vs эталон для лучшей точки."""
    if not best.get("wrk"):
        return None
    tools = os.path.join(best["wrk"], "tools")
    result_path = os.path.join(tools, "result.txt")
    reference_path = os.path.join(tools, "reference.txt")
    if not (os.path.exists(result_path) and os.path.exists(reference_path)):
        return None
    if not os.path.exists(PLOT_SCRIPT):
        print(f"   (нет скрипта графиков: {PLOT_SCRIPT})")
        return None
    out_png = os.path.join(WORK_ROOT, "best_comparison.png")
    subprocess.run(
        [sys.executable, PLOT_SCRIPT, result_path, reference_path,
         "-o", out_png, "--save-only"],
        check=False,
        capture_output=True,
        text=True,
    )
    if os.path.exists(out_png):
        print(f"   График «Калман vs эталон»: {out_png}")
        _plot_errors(best)
        return out_png
    return None


def _plot_errors(best):
    """Дополнительные графики ошибок (после best_comparison.png):
    best_errors.png (J-метрика), best_diffs.png (все разности),
    best_kf_errors.png (внутренние ошибки КФ)."""
    tools = os.path.join(best["wrk"], "tools")
    result_path = os.path.join(tools, "result.txt")
    reference_path = os.path.join(tools, "reference.txt")
    errors_path = os.path.join(tools, "errors.txt")
    out_dir = WORK_ROOT
    cmd = [sys.executable, PLOT_ERRORS_SCRIPT,
           "--result", result_path,
           "--reference", reference_path,
           "--out-dir", out_dir]
    if os.path.exists(errors_path):
        cmd += ["--errors", errors_path]
    subprocess.run(cmd, check=False, capture_output=True, text=True)
    for name in ("best_errors.png", "best_diffs.png", "best_kf_errors.png"):
        png = os.path.join(out_dir, name)
        if os.path.exists(png):
            print(f"   График ошибок: {png}")


# ============================================================
# E1b: ЦЕНТРОИД «ПЛАТО» ВМЕСТО ОДИНОЧНОЙ ТОЧКИ ИТОГА
# ============================================================

def plateau_centroid(pool, best):
    """E1b: вложенный скан соседей вокруг best по осям u (±PLATEAU_SCAN_HALF
    шагом PLATEAU_SCAN_STEP), выделение «плато» (J_train в полосе PLATEAU_BAND
    от минимума) и возврат его центроида (медианы по каждой оси) как робастной
    финальной точки. Оценка идёт через pool (папки plateau_XXX).

    Возвращает новый best-словарь. Логика выбора:
      * если центроид ещё не оценивался — оценивается (один прогон);
      * если J_train(центроид) в полосе — центроид становится итогом;
      * иначе остаётся исходный best (лучший по J_train в плато).
    """
    d = len(PARAMS)
    best_u = np.asarray(best["u"], dtype=float)
    grid = []
    for i in range(d):
        lo, hi = float(U_LO[i]), float(U_HI[i])
        k = 1
        while k * PLATEAU_SCAN_STEP <= PLATEAU_SCAN_HALF + 1e-12:
            for sgn in (-1.0, 1.0):
                v = min(hi, max(lo, best_u[i] + sgn * k * PLATEAU_SCAN_STEP))
                u = best_u.copy()
                u[i] = v
                if not any(np.allclose(u, g, atol=1e-9) for g in grid):
                    grid.append(u)
            k += 1
    n_total = len(grid)
    tasks = [(list(u), os.path.join(WORK_ROOT, f"plateau_{k:03d}"))
             for k, u in enumerate(grid)]
    print(f"\n=== E1b: центроид плато (скан ±{PLATEAU_SCAN_HALF:g} шагом "
          f"{PLATEAU_SCAN_STEP:g} по осям u, {n_total} соседей, "
          f"полоса J_train <= best + {PLATEAU_BAND}) ===")
    t0 = time.time()
    results = list(pool.imap(worker, tasks))

    n_fail = sum(1 for u, (_wrk, (Jt, _Jv, _s, st)) in zip(grid, results)
                 if st is not None or Jt >= BAD_ERROR)
    best_j_min = best["J"]
    plateau = [(u, Jt, Jv, _s)
               for u, (_wrk, (Jt, Jv, _s, st)) in zip(grid, results)
               if (st is None and Jt < BAD_ERROR
                   and Jt <= best_j_min + PLATEAU_BAND)]
    j_min_scan = min((r[1] for r in plateau), default=best_j_min)
    print(f"   Соседей в плато: {len(plateau)}/{n_total} (упало {n_fail}, "
          f"J_min={j_min_scan:.4f}, {time.time() - t0:.0f} с)")
    if len(plateau) < 2:
        print("   Плато из одной точки — оставляем лучший результат без изменений.")
        return dict(best, plateau={"n": len(plateau), "j_min": j_min_scan})

    centroid = np.median(np.array([r[0] for r in plateau]), axis=0)
    already = next((r for r in plateau if np.allclose(r[0], centroid, atol=1e-9)), None)
    if already is not None:
        print("   Центроид совпал с уже оценённой точкой сетки — переоценка не нужна.")
        j_cent, j_valid, _sum = already[1], already[2], already[3]
        m = next(k for k, u in enumerate(grid) if np.allclose(u, centroid, atol=1e-9))
        cdir = tasks[m][1]
    else:
        cdir = os.path.join(WORK_ROOT, "plateau_centroid")
        j_cent, j_valid, _sum, _st = run_eval(centroid, cdir)
        if _st is not None or j_cent >= BAD_ERROR:
            print(f"   Центроид не оценился ({_st}) — оставляем точку плато.")
            return dict(best, plateau={"n": len(plateau), "j_min": j_min_scan})
    print(f"   Центроид: J_train={j_cent:.4f}, J_valid={j_valid:.4f}")

    if j_cent <= best_j_min + PLATEAU_BAND:
        print("   -> финальная точка = центроид плато (робастный итог, "
              f"J_train={j_cent:.4f})")
        best_new = dict(best)
        best_new["u"] = list(centroid)
        best_new["J"] = j_cent
        best_new["valid"] = j_valid
        best_new["summary"] = _sum
        best_new["wrk"] = cdir
        best_new["plateau"] = {"n": len(plateau), "j_min": j_min_scan,
                               "span": PLATEAU_SCAN_HALF,
                               "band": PLATEAU_BAND}
        return best_new
    print(f"   Центроид выше полосы (J_train={j_cent:.4f} > "
          f"{best_j_min:.4f} + {PLATEAU_BAND}) — оставляем точку плато "
          "с лучшим J_train.")
    return dict(best, plateau={"n": len(plateau), "j_min": j_min_scan})


# ============================================================
# ОСНОВНАЯ ПРОГРАММА
# ============================================================

def save_best_txt(best, results_top, sens=None):
    """Записывает подобранные величины в WORK_ROOT/best_params.txt.

    Параметры (СИ + u=log10(sigma/ref)), метрики J (train/valid),
    breakdown по компонентам и (при наличии) чувствительность E5.
    """
    best_u = np.clip(np.array(best["u"]),
                     list(U_LO), list(U_HI))
    s = best.get("summary") or {}

    def row(p95, tol):
        return p95 / tol if p95 else 0.0

    lines = []
    lines.append("=== Подобранные параметры фильтра Калмана ===")
    lines.append(f"данные            : {DATA_DIR}")
    dataset_name = os.path.basename(os.path.normpath(str(DATA_DIR))) if DATA_DIR else ""
    lines.append(f"набор             : {dataset_name}")
    ref_truth = os.environ.get("TUNE_REF_TRUTH") == "1"
    lines.append("эталон ошибки     : " + ("truth.txt (наземная истина)" if ref_truth
                                          else "reference.txt (эталон СНС)"))
    lines.append(f"J_train           : {best.get('J', 0.0):.4f} "
                 f"(этап 1: {results_top[0][0]:.4f})")
    lines.append(f"J_valid           : {best.get('valid', 0.0):.4f}")
    reg = s.get("reg", 0.0)
    lines.append(f"reg (штраф u^2)   : {reg:.4f}")
    lines.append("")
    lines.append("--- breakdown по компонентам (норм. = P95/допуск) ---")
    lines.append(f"train pos: {row(s.get('p95_pos_train', 0.0), TOL_POS):6.3f} "
                 f"(RAW {s.get('p95_pos_train', 0.0):.3f} м / допуск {TOL_POS} м)")
    lines.append(f"train vel: {row(s.get('p95_vel_train', 0.0), TOL_VEL):6.3f} "
                 f"(RAW {s.get('p95_vel_train', 0.0):.3f} м/с / допуск {TOL_VEL} м/с)")
    lines.append(f"train hdg: {row(s.get('p95_hdg_train', 0.0), TOL_HDG):6.3f} "
                 f"(RAW {s.get('p95_hdg_train', 0.0):.3f} ° / допуск {TOL_HDG} °)")
    lines.append(f"valid pos: {row(s.get('p95_pos_valid', 0.0), TOL_POS):6.3f} "
                 f"(RAW {s.get('p95_pos_valid', 0.0):.3f} м)")
    lines.append(f"valid vel: {row(s.get('p95_vel_valid', 0.0), TOL_VEL):6.3f} "
                 f"(RAW {s.get('p95_vel_valid', 0.0):.3f} м/с)")
    lines.append(f"valid hdg: {row(s.get('p95_hdg_valid', 0.0), TOL_HDG):6.3f} "
                 f"(RAW {s.get('p95_hdg_valid', 0.0):.3f} °)")
    tail_tr = s.get('tail_pos_train', 0.0)
    tail_va = s.get('tail_pos_valid', 0.0)
    lines.append(f"tail pos (P{TAIL_Q:g}@200Гц) train: "
                 f"{TAIL_WEIGHT * tail_tr / TOL_POS:6.3f} (RAW {tail_tr:.3f} м)")
    lines.append(f"tail pos (P{TAIL_Q:g}@200Гц) valid: "
                 f"{TAIL_WEIGHT * tail_va / TOL_POS:6.3f} (RAW {tail_va:.3f} м)")
    lines.append("")
    lines.append("--- sigma (СИ) ---")
    lines.append(f"{'параметр':<12}{'sigma':>16}{'u=log10(s/ref)':>16}")
    for (name, ref, _), u in zip(PARAMS, best_u):
        lines.append(f"{name:<12}{ref * 10.0 ** u:>16.6e}{u:>16.4f}")
    lines.append("")
    plateau = best.get("plateau")
    if plateau:
        lines.append("--- E1b: центроид плато ---")
        lines.append(f"точек в плато     : {plateau.get('n', 0)}")
        lines.append(f"J_min плато       : {plateau.get('j_min', 0.0):.4f}")
        lines.append(f"полоса band       : +{plateau.get('band', 0.0):.2f} по J")
        lines.append(f"скан по u         : ±{plateau.get('span', 0.0):.1f} "
                     f"шагом {PLATEAU_SCAN_STEP:g}")
        lines.append("")
    if sens:
        lines.append("--- чувствительность J к параметрам (шаг ±0.15 по u) ---")
        base = next((r for r in sens if r["name"] == "base"), None)
        if base:
            lines.append(f"база: J_train={base['J_train']:.4f}, "
                         f"J_valid={base['J_valid']:.4f}")
        for r in sens:
            if r["name"] == "base":
                continue
            lines.append(f"{r['name']:<12} sign={r['sign']:+1.0f} "
                         f"J_train={r['J_train']:.4f} J_valid={r['J_valid']:.4f}")
    header = os.path.join(WORK_ROOT, "best_params.txt")
    with open(header, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print(f"   Сохранено: {header}")
    return header


def _fmt_dur(sec):
    """Форматирование длительности этапа (без греческих символов — cp1251)."""
    sec = int(round(sec))
    h, rem = divmod(sec, 3600)
    m, s = divmod(rem, 60)
    if h:
        return f"{h} ч {m:02d} мин {s:02d} с"
    if m:
        return f"{m} мин {s:02d} с"
    return f"{s} с"


def load_config(config_path=None):
    """Читает tuner_config.json рядом со скриптом и переопределяет глобалы.

    Структура JSON — tools/kalman_tuner/tuner_config.json (см. DEVELOPMENT_PLAN,
    «Подбор — опциональная функция»). Если файла нет — остаются встроенные
    значения (v3-базлайн + sigma_h). config_path — путь к альтернативному
    конфигу (--config=...).
    """
    global DATA_DIR, BUILD_DIR, TARGET_EXE, WORK_ROOT
    global PARAMS, TOL_POS, TOL_VEL, TOL_HDG, TIE_WEIGHT, REG_LAMBDA
    global TAIL_Q, TAIL_WEIGHT, SKIP_FIRST_S, TRAIN_FRAC
    global CONSISTENCY_WEIGHT, WINDOW_S, OUTLIER_POS_M
    global N_GLOBAL, N_LOCAL, N_WORKERS, RUN_TIMEOUT, N_GLOBAL2, B5_SPAN
    global SEED, EARLY_STOP_STALE, VALID_OVFIT_RATIO, POLISH_METHOD

    if config_path:
        path = os.path.abspath(config_path)
    else:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tuner_config.json")
    if not os.path.exists(path):
        return
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    if "data_dir" in cfg:
        DATA_DIR = cfg["data_dir"]
        os.environ["TUNE_DATA_DIR"] = data_dir_abs(DATA_DIR)
    if "build_dir" in cfg:
        BUILD_DIR = cfg["build_dir"]
    if "target_exe" in cfg:
        TARGET_EXE = cfg["target_exe"]
    if "work_root" in cfg:
        WORK_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 cfg["work_root"])
        os.environ["TUNE_WORK_ROOT"] = WORK_ROOT

    p = cfg.get("params")
    if p:
        new_params = []
        for e in p:
            name = e["name"]
            sref = float(e.get("sigma_ref", 0.0))
            if "fixed" in e:
                # 13G: параметр не подбирается — фиксируется на абсолютном
                # значении `fixed` (σ_ref = fixed, нулевая ширина интервала).
                # Значение 0 допустимо (детерминированное смещение без дрейфа).
                sref = float(e["fixed"])
                span = (0.0, 0.0)
            elif "u_lo" in e and "u_hi" in e:
                span = (float(e["u_lo"]), float(e["u_hi"]))
            elif "u_bounds" in e and isinstance(e["u_bounds"], (list, tuple)) and len(e["u_bounds"]) == 2:
                span = (float(e["u_bounds"][0]), float(e["u_bounds"][1]))
            else:
                span = float(e.get("u_span", 1.0))
            new_params.append((name, sref, span))
        PARAMS = new_params
        rebuild_bounds()

    m = cfg.get("metric")
    if m:
        TOL_POS = float(m.get("tol_pos", TOL_POS))
        TOL_VEL = float(m.get("tol_vel", TOL_VEL))
        TOL_HDG = float(m.get("tol_hdg", TOL_HDG))
        TIE_WEIGHT = float(m.get("tie_weight", TIE_WEIGHT))
        REG_LAMBDA = float(m.get("ref_lambda", REG_LAMBDA))
        TAIL_Q = float(m.get("tail_q", TAIL_Q))
        TAIL_WEIGHT = float(m.get("tail_weight", TAIL_WEIGHT))
        CONSISTENCY_WEIGHT = float(m.get("consistency_weight", CONSISTENCY_WEIGHT))
        WINDOW_S = float(m.get("window_s", WINDOW_S))
        OUTLIER_POS_M = float(m.get("outlier_pos_m", OUTLIER_POS_M))

    h = cfg.get("horizon")
    if h:
        SKIP_FIRST_S = float(h.get("skip_first_s", SKIP_FIRST_S))
        TRAIN_FRAC = float(h.get("train_frac", TRAIN_FRAC))

    o = cfg.get("optim")
    if o:
        N_GLOBAL = int(o.get("n_global", N_GLOBAL))
        N_LOCAL = int(o.get("n_local", N_LOCAL))
        N_WORKERS = int(o.get("n_workers", N_WORKERS))
        RUN_TIMEOUT = int(o.get("run_timeout_s", RUN_TIMEOUT))
        N_GLOBAL2 = int(o.get("n_global2", N_GLOBAL2))
        B5_SPAN = float(o.get("b5_span", B5_SPAN))
        SEED = int(o.get("seed", SEED))
        EARLY_STOP_STALE = int(o.get("early_stop_stale", EARLY_STOP_STALE))
        VALID_OVFIT_RATIO = float(o.get("valid_ovfit_ratio", VALID_OVFIT_RATIO))
        POLISH_METHOD = o.get("polish_method", POLISH_METHOD)
        wpen = o.get("wall_penalty", {}) if o else {}
        global WALL_PENALTY_W, WALL_PENALTY_EPS_FRAC
        WALL_PENALTY_W = float(wpen.get("w", 0.1))
        WALL_PENALTY_EPS_FRAC = float(wpen.get("eps_frac", 0.05))
def data_dir_abs(d):
    """Нормализует путь к каталогу данных (прямые/обратные слэши, ~)."""
    d = d.replace("/", os.sep).replace("\\", os.sep)
    if d.startswith("~"):
        d = os.path.expanduser(d)
    return os.path.abspath(d)


def save_preset(best, preset_path):
    """Записывает best-точку как пресет presets/tuned.ini.

    Формат совпадает с dumpFilterSettings (src/core/navigation/filter_settings.cpp):
    секционный INI с группой [filter]; ключи вне секций игнорируются. Пресет
    применяется явно: `imitator <data_dir> <preset.ini>` (loader сам определит
    секционный формат). Сами дефолты и текущий settings.ini не трогает.

    Значения берутся из params.ini финального прогона (best['wrk']/params.ini), а не
    пересчитываются из u-вектора: на Windows воркеры Pool пересоздают модуль и
    используют sigma_ref из шаблона (а не из конфига), поэтому пересчёт по PARAMS
    главного процесса может не совпадать с реально отработавшими в imitator σ.
    Если params.ini финального прогона недоступен — запасной путь через u-вектор.
    """
    sigma = {}
    wrk = best.get("wrk")
    ini_path = os.path.join(wrk, "params.ini") if wrk else ""
    if os.path.exists(ini_path):
        for line in open(ini_path, encoding="utf-8", errors="replace"):
            line = line.strip()
            if line.startswith(("#", "[")) or "=" not in line:
                continue
            name, val = [x.strip() for x in line.split("=", 1)]
            if name in {p[0] for p in PARAMS}:
                sigma[name] = float(val)
    if len(sigma) != len(PARAMS):
        if sigma:
            print("   (в params.ini финального прогона не все ключи — "
                  "пересчёт пресета из u-вектора)")
        best_u = np.clip(np.array(best["u"]),
                         list(U_LO), list(U_HI))
        sigma = {name: ref * 10.0 ** u for (name, ref, _), u in zip(PARAMS, best_u)}

    # sigma_ang в тюнере — это СНС-курс (sig_hdg -> key filter/sigma_hdg).
    names = [("sigma_g", "sigma_g"), ("sigma_a", "sigma_a"),
             ("sigma_bg", "sigma_bg"), ("sigma_ba", "sigma_ba"),
             ("sigma_pos", "sigma_pos"), ("sigma_h", "sigma_h"),
             ("sigma_v", "sigma_v"), ("sigma_ang", "sigma_hdg")]

    # Паспорт набора для секции [meta]: имя каталога данных и эталон ошибки.
    data_dir = str(globals().get("DATA_DIR", "") or "")
    dataset_name = os.path.basename(os.path.normpath(data_dir)) if data_dir else ""
    ref_truth = os.environ.get("TUNE_REF_TRUTH") == "1"

    lines = ["# Пресет подобранных коэффициентов КФ (см. tools/kalman_tuner).",
             "# Применение явное: imitator <data_dir> <preset.ini>.",
             "# Отсутствующие ключи берутся из значений по умолчанию.",
             "",
             "# Паспорт набора: с каким каталогом данных подобран пресет. Секция",
             "# [meta] ядру неизвестна (loadFilterSettings читает только",
             "# filter/tilt/run) и на математику не влияет — GUI использует её",
             "# для сверки имён «набор ↔ пресет».",
             "[meta]",
             f"dataset = {dataset_name}",
             f"data_dir = {data_dir}",
             f"error_ref = {'truth.txt' if ref_truth else 'reference.txt'}",
             "",
             "[filter]"]
    for tuner_key, ini_key in names:
        lines.append(f"{ini_key} = {sigma[tuner_key]:.17g}")
    lines.append("")

    preset_path = os.path.abspath(preset_path)
    os.makedirs(os.path.dirname(preset_path), exist_ok=True)
    with open(preset_path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print(f"   Пресет сохранён: {preset_path}")
    return preset_path


def main():
    global DATA_DIR, WORK_ROOT, N_GLOBAL, N_LOCAL, N_WORKERS, N_GLOBAL2, POLISH_METHOD
    global_only = "--global-only" in sys.argv
    quick = "--quick" in sys.argv
    if "--basinhopping" in sys.argv:
        POLISH_METHOD = "bh"
    if "--ref-truth" in sys.argv:
        os.environ["TUNE_REF_TRUTH"] = "1"
    # Вручную заданные --data-dir/--work-root имеют приоритет над конфигом.
    cli_data = None
    cli_work = None
    load_config_path = None
    for a in sys.argv[1:]:
        if a.startswith("--config="):
            load_config_path = a.split("=", 1)[1]
        elif a.startswith("--data-dir="):
            cli_data = a.split("=", 1)[1]
        elif a.startswith("--work-root="):
            cli_work = a.split("=", 1)[1]
    load_config(load_config_path)
    if cli_data:
        DATA_DIR = cli_data
        os.environ["TUNE_DATA_DIR"] = DATA_DIR
    if cli_work:
        WORK_ROOT = os.path.abspath(cli_work)
        os.environ["TUNE_WORK_ROOT"] = WORK_ROOT
    if quick:
        N_GLOBAL, N_LOCAL, N_WORKERS, N_GLOBAL2 = 16, 30, 8, 8
        print("Быстрый режим: N_GLOBAL=16, N_LOCAL=30, N_GLOBAL2=8")

    t0 = time.monotonic()
    t_stage = t0
    print("=" * 70)
    print("   ПОДБОР КОЭФФИЦИЕНТОВ ФИЛЬТРА КАЛМАНА (v3)")
    print(f"   Данные: {DATA_DIR}")
    print(f"   exe: {BUILD_DIR}\\{TARGET_EXE}")
    print(f"   Метрика: J = P95(pos)/{TOL_POS} + P95(vel)/{TOL_VEL} + P95(hdg)/{TOL_HDG}"
          f" + {TIE_WEIGHT}·Mean + {REG_LAMBDA}·u2"
          f" + {TAIL_WEIGHT}·P{TAIL_Q:g}(pos@200Hz)/{TOL_POS} [v3]")
    print(f"   Горизонт: skip={SKIP_FIRST_S} с, train={TRAIN_FRAC:.0%}, workers={N_WORKERS}")
    print("   Эталон ошибки: " + ("truth.txt (наземная истина)"
                                  if os.environ.get("TUNE_REF_TRUTH") == "1"
                                  else "reference.txt (эталон СНС)"))
    print(f"   Сетка: ранний стоп после {EARLY_STOP_STALE} нейтральных точек (A2); "
          f"полировка: {'basinhopping (B8)' if POLISH_METHOD == 'bh' else 'Nelder-Mead'}; "
          f"старт у u=0 (B6)")
    print(f"   Старт: {datetime.now():%H:%M:%S}")
    print("=" * 70)

    if not os.path.exists(DATA_DIR):
        print(f"Ошибка: нет каталога {DATA_DIR}")
        return
    exe = os.path.join(BUILD_DIR, TARGET_EXE)
    if not os.path.exists(exe):
        print(f"Ошибка: не найден {exe}")
        return

    # Очищаем рабочий каталог.
    if os.path.exists(WORK_ROOT):
        shutil.rmtree(WORK_ROOT)
    os.makedirs(WORK_ROOT)

    def read_u_from_ini(wrk):
        u = []
        valid_keys = {p[0] for p in PARAMS}
        for line in open(os.path.join(wrk, "params.ini"), encoding="utf-8").read().splitlines():
            if " = " not in line:
                continue
            name, val = line.split(" = ")
            if name not in valid_keys:
                continue
            ref = {p[0]: p[1] for p in PARAMS}[name]
            u.append(np.log10(float(val) / ref))
        return np.array(u)

    with Pool(processes=N_WORKERS) as pool:
        # --- Этап 1 ---
        results, meta1 = global_search(N_GLOBAL, pool)
        # results: (J_train, J_valid, summary, work_dir, status)
        results = [r for r in results if r[0] < BAD_ERROR]
        if not results:
            print("\nОшибка: ни один прогон этапа 1 не завершился успешно.")
            return
        results.sort(key=lambda r: r[0])
        print(f"   Этап 1: оценок {meta1['n_done']}/{N_GLOBAL + 1}, "
              f"упало {meta1['n_fail']}, пропущено по раннему стопу {meta1['n_skipped']}")
        print(f"   Этап 1 (сетка) занял {_fmt_dur(time.monotonic() - t_stage)}")
        t_stage = time.monotonic()

        def print_breakdown(tag, Jt, Jv, summary):
            print(f"   {tag}: J_train={Jt:.4f}, J_valid={Jv:.4f}")
            if summary:
                row = (summary.get("p95_pos_train", 0.0), summary.get("p95_vel_train", 0.0),
                       summary.get("p95_hdg_train", 0.0))
                print(f"      train -> pos {row[0]/TOL_POS:6.3f} | "
                      f"vel {row[1]/TOL_VEL:6.3f} | "
                      f"hdg {row[2]/TOL_HDG:6.3f} "
                      f"(RAW: pos {row[0]:.3f} м, "
                      f"vel {row[1]:.3f} м/с, "
                      f"hdg {row[2]:.3f} °)")
                row = (summary.get("p95_pos_valid", 0.0), summary.get("p95_vel_valid", 0.0),
                       summary.get("p95_hdg_valid", 0.0))
                print(f"      valid -> pos {row[0]/TOL_POS:6.3f} | "
                      f"vel {row[1]/TOL_VEL:6.3f} | "
                      f"hdg {row[2]/TOL_HDG:6.3f} "
                      f"(RAW: pos {row[0]:.3f} м, "
                      f"vel {row[1]:.3f} м/с, "
                      f"hdg {row[2]:.3f} °)")
                reg = summary.get("reg", 0.0)
                print(f"      (reg {reg:.4f})")
                tail_tr = summary.get("tail_pos_train", 0.0)
                tail_va = summary.get("tail_pos_valid", 0.0)
                print(f"      tail P{TAIL_Q:g}@200Hz -> "
                      f"train {TAIL_WEIGHT * tail_tr / TOL_POS:6.3f} | "
                      f"valid {TAIL_WEIGHT * tail_va / TOL_POS:6.3f} "
                      f"(RAW: {tail_tr:.3f} / {tail_va:.3f} м)")
                wq_tr = summary.get("win_q_train", 0.0)
                wq_va = summary.get("win_q_valid", 0.0)
                of_va = summary.get("outlier_frac_valid", 0.0)
                print(f"      13C win P{TAIL_Q:g}@{WINDOW_S:g}s -> "
                      f"train {wq_tr:6.3f} | valid {wq_va:6.3f} м "
                      f"(outlier>{OUTLIER_POS_M:g}м valid {of_va * 100:.3f}%)")

        print("\nТоп-5 после этапа 1 (breakdown по компонентам J):")
        for i, (Jt, Jv, summary, wrk, _status) in enumerate(results[:5], 1):
            line = " ".join(open(os.path.join(wrk, "params.ini"), encoding="utf-8")
                           .read().splitlines()[1:4])
            print(f"   #{i}  J_train={Jt:.4f}, J_valid={Jv:.4f}  {line}")
            print_breakdown(f"#{i}", Jt, Jv, summary)

        if global_only:
            print("\nРежим --global-only, локальная полировка пропущена.")
            print_breakdown("Лучшая точка", results[0][0], results[0][1], results[0][2])
            sens = run_sensitivity(pool, read_u_from_ini(results[0][3]))
            plot_sensitivity(sens, os.path.join(WORK_ROOT, "sensitivity.png"))
            print(f"\n   Готово за {_fmt_dur(time.monotonic() - t0)} "
                  f"(финиш {datetime.now():%H:%M:%S})")
            return

        # --- Этап 2: параллельные полировки от топ-2 рестартов + B6 (у u=0) ---
        starts = [(read_u_from_ini(wrk), i, POLISH_METHOD)
                  for i, (_, _, _, wrk, _) in enumerate(results[:2])]
        starts.append((near_zero_start(), 2, POLISH_METHOD))  # B6: окрестность u=0
        ws = warm_start_u()  # F8: старт от архивного best того же набора данных
        if ws is not None:
            starts.append((ws[0], 20, POLISH_METHOD))
        polished = pool.map(polish_worker, starts)
        best = min(polished, key=lambda b: b["J"])
        print(f"   Этап 2 (полировки, {len(starts)} стартов) занял "
              f"{_fmt_dur(time.monotonic() - t_stage)}")
        t_stage = time.monotonic()

        # --- B5: вторая (уточняющая) сетка вокруг бассейна + вторая полировка ---
        g2, meta_b5 = global_search(N_GLOBAL2, pool, center=best["u"], span=B5_SPAN)
        g2 = [r for r in g2 if r[0] < BAD_ERROR]
        if meta_b5["n_fail"]:
            print(f"   B5: упало {meta_b5['n_fail']} точек, "
                  f"пропущено {meta_b5['n_skipped']} (ранний стоп)")
        g2.sort(key=lambda r: r[0])
        print("B5: топ-3 уточняющей сетки (breakdown по компонентам J):")
        for i, (Jt, Jv, summary, wrk, _status) in enumerate(g2[:3], 1):
            print_breakdown(f"#{i}", Jt, Jv, summary)
        starts2 = [(read_u_from_ini(wrk), 3 + i, POLISH_METHOD)
                   for i, (_, _, _, wrk, _) in enumerate(g2[:2])]
        polished.extend(pool.map(polish_worker, starts2))
        best = min(polished, key=lambda b: b["J"])
        print(f"   B5 (уточняющая сетка + полировки) занял "
              f"{_fmt_dur(time.monotonic() - t_stage)}")
        t_stage = time.monotonic()

        # --- B7: откат к точке с лучшим J_valid при подозрении на переобучение ---
        cand = [("полир.", b) for b in polished]
        cand += [("эт.1", {"J": r[0], "valid": r[1], "summary": r[2],
                           "u": read_u_from_ini(r[3]), "wrk": r[3]})
                 for r in results[:5]]
        cand += [("эт.B5", {"J": r[0], "valid": r[1], "summary": r[2],
                            "u": read_u_from_ini(r[3]), "wrk": r[3]})
                 for r in g2[:5]]
        cand = [(tag, b) for tag, b in cand
                if b["J"] < BAD_ERROR and b["valid"] < BAD_ERROR]
        best_v = min(b["valid"] for _, b in cand)
        if best["J"] > 0 and best_v / best["J"] > VALID_OVFIT_RATIO:
            tag, rollback = min(cand, key=lambda x: x[1]["valid"])
            print(f"\n   B7: J_valid/J_train={best_v / best['J']:.2f} > "
                  f"{VALID_OVFIT_RATIO:.1f} — возможное переобучение.")
            print(f"   Откат: {tag} J_train={rollback['J']:.4f}, "
                  f"J_valid={rollback['valid']:.4f}")
            best = rollback

        # --- E1b: центроид плато вместо одиночной точки итога ---
        best = plateau_centroid(pool, best)
        print(f"   E1b (центроид плато) занял {_fmt_dur(time.monotonic() - t_stage)}")
        t_stage = time.monotonic()

        # --- E5: чувствительность J к параметрам у лучшей точки ---
        sens = run_sensitivity(pool, np.array(best["u"]))
        print(f"   E5 (чувствительность) занял {_fmt_dur(time.monotonic() - t_stage)}")

    # --- Итог ---
    print("\n" + "=" * 70)
    print("   РЕЗУЛЬТАТЫ")
    print("=" * 70)
    print(f"Этап 1: J_train={results[0][0]:.4f} ({results[0][3]})")
    print(f"Финальная точка: J_train={best['J']:.4f}, J_valid={best['valid']:.4f}")
    print_breakdown("Финальная точка", best["J"], best["valid"], best.get("summary"))

    best_u = np.clip(np.array(best["u"]),
                     list(U_LO), list(U_HI))
    print("Найденные параметры:")
    for (name, ref, _), u in zip(PARAMS, best_u):
        print(f"   {name:10s} = {ref * 10.0 ** u:.6e}")

    # --- Сохранение подобранных величин в txt ---
    try:
        save_best_txt(best, results, sens)
    except Exception as exc:
        print(f"   (не удалось сохранить best_params.txt: {exc})")

    # --- Пресет presets/tuned.ini (если путь задан в tuner_config.json) ---
    try:
        preset_cfg = None
        cfg_path = load_config_path or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "tuner_config.json")
        if os.path.exists(cfg_path):
            with open(cfg_path, "r", encoding="utf-8") as f:
                preset_cfg = json.load(f).get("output", {}).get("preset")
        if preset_cfg:
            save_preset(best, preset_cfg)
    except Exception as exc:
        print(f"   (не удалось сохранить пресет: {exc})")

    # --- Графики по итогам ---
    plot_comparison(best)
    plot_sensitivity(sens, os.path.join(WORK_ROOT, "sensitivity.png"))
    print(f"\n   Готово за {_fmt_dur(time.monotonic() - t0)} "
          f"(финиш {datetime.now():%H:%M:%S})")


if __name__ == "__main__":
    main()
