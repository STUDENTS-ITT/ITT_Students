# Подробное руководство по коду NoQT (для разбора алгоритма)

## 1. Назначение и структура
Проект NoQT — консольный имитатор БИНС с ESKF 15-го порядка без Qt. Основная цель — разобраться в алгоритме от выставки до счисления.

Структура:
- `src/main.cpp` — точка входа, выставка, основной цикл
- `src/navigation/averaging.*` — окно усреднения, расчёт средних/ба/бг, гейт покоя, `x0/P0`
- `src/navigation/trajectory.h` — `nav::step`, интегрирование, предсказание/коррекции, логика аксель-апдейта и коррекции по СНС
- `src/navigation/gnss_guard.h` — защита СНС уровни 1-2
- `src/ins/ins_filter.h` — ESKF 15-го порядка (`predict/correct/correctAccel`), матрицы, reset
- `src/ins/attitude_calc.h`, `src/ins/imu_processor.h` — интегрирование ориентации, обработка ИМУ
- `src/navigation/position_calc.h`, `src/navigation/gps_processor.h` — интегрирование координат/скорости
- `src/data_io/*` — чтение/запись файлов
- `src/utils/*`, `src/math_lib/*` — константы, типы, матричные операции

## 2. Точка входа (main.cpp)
Ключевые участки:
1. Разбор аргументов (`--bias-log`, `--bias-report`, `--consistency`), определение `data_dir`, `params.ini`
2. Чтение `StartupNav.ini` → `start_lon/start_lat/start_alt/align_time`. Если не найдено — берётся первый кадр СНС, `align_time = ALIGN_FALLBACK_S` (120.0)
3. Выбор файлов `pickFile()` для `imu/gps/angle` (.dat/.data)
4. Сбор окна усреднения: `nav::AveragingWindow window(align_time, has_angle)`. Пока `row[0] <= align_time` добавляются ИМУ (`addImu`), СНС (`addSns`). Кадры с `t > align_time` сохраняются в `pending_row`/`last_ref` для основного цикла
5. `avg = window.finish(lat_rad, alt_m)` — расчёт статистик
6. Инициализация `nav::NavState st`: координаты/скорость из последнего кадра СНС окна, курс — из последнего или гирокомпасного при наличии `angle.dat`, `ba = avg.ba_est`, `bg = {0,0,0}`
7. `st.P = nav::initialCovarianceFromAveraging(ins::kalman_cfg, avg, st.lat, has_angle, sig_heading0)`; `st.x = nav::initialErrorsFromAveraging(avg, avg.rest_ok)`. При наличии `angle.dat` `sig_heading0` берётся из него, иначе `sigma_bg/Omega_h`
8. Открытие логов `data_io::NavLogger` (result/reference/errors + по флагам bias/consistency/gnss_guard)
9. Основной цикл по строкам `imu.dat`: синхронизация СНС (`hold_ref/last_ref`), проверка `outage`, вызов `nav::step(row, hold_ref, st, sink, do_correction, has_angle)`

## 3. Окно усреднения (navigation/averaging.*)
### AveragingWindow
- Буферизирует ИМУ/СНС до `T_align`
- После сбора для каждого канала считает среднее `mean` и максимум отклонения `Δ = max_t |x(t) - mean|`
- Перевод в СКО: `σ̂_отсчёта = Δ/k(N)`, `σ̂_среднего = σ̂_отсчёта/√N`, где `k(N)=E[max|N(0,1)|]` (таблица в `maxDeviationFactor`)
- Номинал позиции/скорости/курса — **последний отсчёт** окна (ему приписывается `σ̂_отсчёта`)
- Наклон, `ba`, `bg` — по **средним** (`σ̂_среднего`)
- Оценки: `ba_est = f_mean - C^T*(0,g,0)`, `bg_est = w_mean - C^T*ω_Земли` (с учётом ориентации по уровню/гирокомпасу)
- Гейт покоя (`checkRest`): проверка `|v|`, отклонения угловой скорости от проекции вращения Земли, отклонения вертикальной удельной силы от g; результат в `avg.rest_ok` и `avg.rest_reason`

### AveragingResult (ключевые поля)
- `imu_count/sns_count`, `t_start/t_end`
- `f_mean[3], w_mean[3]`, `df_max[3], dw_max[3]` (макс отклонения)
- `sigma_f_sample[3], sigma_f_mean[3], sigma_w_sample[3], sigma_w_mean[3]`
- `sigma_p_sample[3], sigma_v_sample[3], sigma_hdg_sample`
- `ba_est[3], bg_est[3]`
- `rest_ok`, `rest_reason`
- `sns_last` (последний СНС окна — источник номинала)

### Инициализация x0/P0
- `initialErrorsFromAveraging(avg, rest_ok)`: `x[0..2]` = координатные ошибки (отклонение последнего от среднего), `x[3..5]` = скоростные, `x[6]` = ошибка курса, `x[7..8]` = ошибки наклона, `x[9..11]` = ошибки `ba` (инициализируются нулём либо малыми), `x[12..14]` = ошибки `bg` (из `bg_est`). Фактически `x0` формируется как "(последний отсчёт окна − среднее)" для тех величин, где среднее осмысленно.
- `initialCovarianceFromAveraging(cfg, avg, lat_rad, has_angle, sig_heading0)`: диагональ/ковариации по СКО из averaging. Корреляции наклон↔`ba`. Если есть `angle.dat` — `sig_heading0` из него, иначе `cfg.sig_bg/Omega_h`. Наложены `floors` (минимальные значения) чтобы избежать нулевой P0.

## 4. NavState (navigation/trajectory.h)
Ключевые поля:
- `lat/lon/alt`, `ins::Attitude att`, `Vector V = [Vn,Vh,Ve]`
- Предыдущие значения для интегрирования (`V_dot_prev`, `*_dot_prev`, `rates_prev`, `time_prev`)
- `Vector ba`, `Vector bg` — текущие смещения
- `SnsGuard sns_guard` — защита СНС
- `Vector x` (15), `Matrix P` (15x15) — ошибки ESKF
- `bg_static`, `tilt_maneuver_time` — служебные

## 5. Основной цикл step (navigation/trajectory.h)
Вход: `row` (ИМУ), `ref` (кадр СНС), `st`, `sink`, `do_correction`, `has_heading_ref`. Выход: обновляет `st`, пишет логи.

Порядок:
1. `dt = time_s - time_prev` (при <=0 берётся `IMU_PERIOD`)
2. `C = bodyToNavMatrix(att.heading,att.pitch,att.roll)`, `f_body = accel(row, st.ba)`, `n_nav = C*f_body`
3. Интегрирование: `a_harm = harmfulAccel(...)`, `V_dot = velocityDot(n_nav,a_harm)`, `V = integrateVelocity(...)`, `pos = integratePosition(...)` (по скорости/положению). Расчёт угловых скоростей: `w_nav = navAngularRate`, `w_rel = relativeRate(gyro_raw - bg, w_nav, C)`, `rates = eulerRates(w_rel, pitch, roll)`. Учёт манёвров (`tilt_maneuver_time`) для горизонта аксель-апдейта
4. `att = integrate(att,rates,rates_prev,dt)`, обновление `st.V/st.lat/st.lon/st.alt/st.att` и предыдущих значений
5. **Предсказание KF**: `C_cur = bodyToNavMatrix(...)`, `n_nav_cur = bodyToNav(C_cur, f_body)`, `ins::predict(dt, st.lat, st.alt, C_cur, n_nav_cur, st.att, st.x, st.P)`
6. **Аксель-апдейт (коррекция по удельной силе)**: `f_meas = f_body + ba`, `z_accel = (navToBody(C_cur,a_harm) + ba) - f_meas`. Если `shouldApplyAccelHorizon(...)` истинно: `ins::correctAccel(C_cur, a_harm, st.att, z_accel, sig_a*sqrt(dt), st.x, st.P)`, `applyTiltKalmanCorrections(st)`. Иначе `clearTiltKalmanErrors(st.x)`. Аксель-апдейт влияет преимущественно на наклон/крен и смещения `ba/bg` (корректирует `x[7..14]`), после применения `x[7..14]` обнуляются
7. **Защита СНС**: `sns_ok = st.sns_guard.update(ref, time_s, ins::kalman_cfg)`. `manual = st.sns_guard.is_new_frame && !do_correction`. `apply_correction = do_correction && sns_ok`
8. **Коррекция по СНС** (если `apply_correction`): формируем `bins = [lat,lon,alt,vn,vh,ve,heading,pitch,roll]`, `sns = [ref.lat,ref.lon,ref.alt,ref.vn,ref.vh,ref.ve, has_heading_ref?ref.heading:st.att.heading, st.att.pitch, st.att.roll]`. `SIG_HDG` берётся из `kalman_cfg.sig_hdg` если есть заголовок, иначе `1e6*DEG_TO_RAD` (игнор). `ins::correct(bins,sns,st.x,st.P,SIG_HDG,SIG_TILT_IGNORE,SIG_TILT_IGNORE, stats)` при необходимости пишет `consistency.txt` (NIS), `sink.writeErrors`, затем `applyKalmanCorrections(st)`, `sink.writeReference(makeReference(...))`
9. Логирование `gnss_guard.txt` при включённом флаге, `sink.writeResult(makeResult(time_s,st))`

### Вспомогательные функции в trajectory.h
- `applyTiltKalmanCorrections(st)`: `att.pitch -= x[7]`, `att.roll -= x[8]`, `ba += x[9..11]`, `bg += x[12..14]`, обнулить `x[7..14]`, вызвать `ins::apply_reset_covariance(st.P)`
- `applyKalmanCorrections(st)`: инъекция по всем 9 (`lat/lon/alt/V[0..2]/heading/pitch/roll`) и `ba/bg` по `x[9..14]`, обнулить весь `x`, `apply_reset_covariance(st.P)`
- `makeResult/makeReference` — конвертация в формат вывода (вычитание ошибок из номинала для `result.txt`)

## 6. ESKF (src/ins/ins_filter.h)
### Состояние
`KF_STATE = 15`. Компоненты (типично): ошибки координат `[0..2]`, скорости `[3..5]`, ориентации `[6..8]` (ошибки углов), смещения акселерометров `[9..11]`, смещения гироскопов `[12..14]`.

### predict
`ins::predict(dt, lat, alt, C, n_nav, att, x, P)`: строит/применяет матрицу перехода `F` (линеаризация кинематики/ошибок), `x = F*x`, `P = F*P*F^T + Q`. `Q` формируется по шумам ИМУ.

### correct (по СНС)
`ins::correct(bins, sns, x, P, sig_hdg, sig_tilt_p, sig_tilt_r, stats)`: строит измерения (координаты/скорость/ориентация), матрицу наблюдения `H` с учётом `1/cos(lat)` для долготы, ковариацию измерений `R`, вычисляет `K`, обновляет `x += K*(z-Hx)`, `P` по Джозефовой форме, при `stats` считает NIS/dof.

### correctAccel
`ins::correctAccel(C, f_nav, att, z_accel, sig_a_eff, x, P)`: измерение — невязка по удельной силе. `H_accel_matrix` строится по компонентам, применяются только к наклону/крену и `ba/bg` (`x[7..14]`). Обновление по формулам Калмана с Джозефовой формой. Эффективный шум учитывает `dt`.

### apply_reset_covariance
Сбрасывает/корректирует ковариацию после инъекции ошибок (ESKF reset) для поддержания численной устойчивости.

## 7. Защита СНС (src/navigation/gnss_guard.h)
Уровни 1-2 согласно `docs/gnss_guard_design.md`. `SnsGuard.update(ref, time_s, cfg)`:
- Проверки конечности/валидности
- Кинематические проверки
- Проверки скачков по положению/скорости
- Проверки молчания/устаревания
- При срабатывании переводит статус в `REJECTED/INVALID` или ограничивает приём
- Счётчики `accepted_frames/rejected_frames`, сохраняет последние `dpos/dvel/dt`
- `is_new_frame` определяет наличие нового кадра СНС

Решение `apply_correction` = `do_correction && sns_ok`. Логируется в `gnss_guard.txt`.

## 8. Выходные файлы (src/data_io/*)
Каталог вывода — `tools/` (через `utils::toolsDir()`).
- `result.txt` — результаты счисления (после вычитания ошибок из номинала): время, долгота/широта/высота (град/м), курс/тангаж/крена (град), `vn/vh/ve` (м/с)
- `reference.txt` — опорные данные СНС/курс из `ref` (только при коррекции/записи по кадру)
- `errors.txt` — вектор ошибок `x[0..14]` по времени
- `init.txt` — параметры выставки/инициализации
- `gnss_guard.txt` — статус/причина/флаги защиты СНС
- `consistency.txt` — NIS при `--consistency`
- `bias.txt` — смещения при `--bias-log`
- `bias_report.json` (при `--bias-report`) — сводка

## 9. Практика разбора (что смотреть по порядку)
1. `src/main.cpp` — понять путь прогона
2. `src/navigation/averaging.cpp/h` — как формируются `avg`, `x0`, `P0`
3. `src/navigation/trajectory.h` — `step()` полностью
4. `src/ins/ins_filter.h` — `predict/correct/correctAccel`
5. `src/navigation/gnss_guard.h` — логику защит
6. `src/data_io/data_writer.*` — что реально пишется

## 10. Важные детали
- Углы внутри — радианы, на выходе — градусы
- Матрицы построчные, размеры передаются явно
- `x0 = "последний отсчёт окна − среднее"`, `P0` по максимумам отклонений с переводом в СКО
- Аксель-апдейт работает по горизонту (учёт манёвров) и корректирует преимущественно наклон/крен и `ba/bg`
- После аксель-апдейта и после коррекции по СНС применяется reset ковариации
- Трим-контур отсутствует в NoQT
- Потоки ИМУ/СНС не перематываются — окна строится по первым кадрам до `align_time`
- Коррекция по СНС игнорирует заголовок, если он отсутствует/невалиден
