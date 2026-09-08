# Команды — пошаговая инструкция

Документ для GitHub: что запускать, зачем, и что должно получиться.

**Не знаете, что такое VINS / DSO / ROS?** → сначала [PROJECTS.ru.md](PROJECTS.ru.md).

## О проекте

**fire-dron** — конвейер визуальной одометрии для БПЛА с надирной камерой:

- **VINS-Mono** — основной алгоритм (камера + IMU), полёт MARS HKairport03
- **DSO** — запасной канал (только камера), «Поворот_коптер»
- **Диспетчер** — выбирает алгоритм по сцене
- **RTK fusion** — постобработка траектории VINS с эталоном GNSS/RTK

Два режима работы:

| Режим | Зачем | GUI |
|-------|-------|-----|
| **Live RViz** | Посмотреть треки и траекторию «как в видео VINS-Mono» | Да, окно RViz |
| **Batch** | Получить CSV/TUM, ATE, PDF, видео | Нет |

---

## Перед любой командой

Все команды ниже — из **корня репозитория** `fire-dron`:

```bash
cd ~/Downloads/cursor/Projects/fire-dron
```

Если клонировали в другое место — подставьте свой путь. Можно зафиксировать:

```bash
export VNAV_ROOT=~/Downloads/cursor/Projects/fire-dron
cd "$VNAV_ROOT"
```

**Почему важно `cd`:** пути вроде `results/...` и `data/mars/...` — **относительные**. Из домашней папки (`~`) они не найдутся.

---

## 1. Первичная установка (один раз)

Нужна только на **чистой машине** или после переустановки Ubuntu. Если VINS уже собран и MARS уже гоняли — переходите к разделу 2.

### 1.1. Зависимости системы, ROS, VINS, DSO

```bash
cd ~/Downloads/cursor/Projects/fire-dron

./scripts/install/01_install_base.sh
./scripts/install/02_install_ros.sh
./scripts/install/03_install_ceres.sh
./scripts/install/04_install_pangolin.sh
./scripts/install/05_build_vins_mono.sh
./scripts/install/06_build_dso.sh
./scripts/install/07_install_evo.sh
```

| Скрипт | Что ставит | Зачем |
|--------|----------|-------|
| `01_install_base.sh` | git, cmake, OpenCV, Eigen… | Базовые инструменты сборки |
| `02_install_ros.sh` | ROS Noetic, rosbag, rviz | Шина данных и визуализация |
| `03_install_ceres.sh` | Ceres Solver 1.14 | Оптимизация в VINS/DSO |
| `04_install_pangolin.sh` | Pangolin | GUI DSO (опционально) |
| `05_build_vins_mono.sh` | VINS-Mono в `~/catkin_ws` | Основной estimators |
| `06_build_dso.sh` | DSO в `~/dso` | Direct sparse odometry |
| `07_install_evo.sh` | evo (Python) | Метрики ATE/RPE |

Запускайте **по порядку**. Каждый шаг может занять 5–30 минут. Ошибка на шаге N — исправьте её, не переходите к N+1.

### 1.2. Конфиг VINS для MARS (один раз)

Конфиги лежат в репозитории `fire-dron/config/`, но **ROS ищет их в catkin_ws**. Копируем вручную:

```bash
cd ~/Downloads/cursor/Projects/fire-dron

mkdir -p ~/catkin_ws/src/VINS-Mono/config/mars
cp config/mars_nadir.yaml ~/catkin_ws/src/VINS-Mono/config/mars/
cp config/mars_nadir.launch ~/catkin_ws/src/VINS-Mono/vins_estimator/launch/
sed -i "s|/home/USER|$HOME|g" ~/catkin_ws/src/VINS-Mono/config/mars/mars_nadir.yaml
```

**Что внутри:**

- `mars_nadir.yaml` — камера, IMU, extrinsic, parallax, `show_track: 1`
- `mars_nadir.launch` — три ноды: `feature_tracker`, `vins_estimator`, `pose_graph`

`sed` заменяет `/home/USER` на ваш логин — иначе VINS не найдёт папку для сохранения траектории.

После изменения yaml в репозитории повторите `cp` и `sed`, чтобы catkin получил актуальную версию.

### 1.3. Патч nadir parallax + пересборка VINS (run8e)

Патч правит исходники VINS в `~/catkin_ws` — блокирует keyframe при быстром повороте камеры (rot-gate, gyro-gate). Без пересборки патч не работает.

```bash
cd ~/Downloads/cursor/Projects/fire-dron

python3 tools/patch_nadir_parallax.py

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws && catkin_make -j$(nproc)
```

**Успех:** в конце `catkin_make` — `[100%] Built target vins_estimator`, без `error`.

### 1.4. ROS в каждом новом терминале

Каждый **новый** терминал не знает про ROS, пока не выполните:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/Downloads/cursor/Projects/fire-dron/scripts/project.sh
vnav_setup_ros
```

**Когда нужно:** если видите `command not found` для `roslaunch`, `rosbag`, `rosrun`.

---

## 2. Live RViz — как в [демo VINS-Mono](https://www.youtube.com/watch?v=yRPl4zy9z1c)

**Что произойдёт:** скрипт `run_vins_mars_live.sh` поднимает roscore, VINS, публикатор RTK и **RViz**. С флагом `--play` ещё и проигрывает bag с записью полёта.

**Что увидите:**

- Окно **RViz** (3D-сетка, траектории, картинки слева)
- В терминале — логи VINS (`Initialization finish!`, иногда `failure detection`)

**Скрипт не сохраняет метрики** — только просмотр. Для ATE/PDF используйте раздел 4.

### 2.1. Быстрая проверка (~1.5 мин) — рекомендуется для первого раза

```bash
cd ~/Downloads/cursor/Projects/fire-dron
./scripts/run_vins_mars_live.sh 1.0 50 --play 90
```

По умолчанию скрипт также:

- открывает **второй терминал** с `rostopic echo /vins_estimator/odometry` — как в [демо VINS-Mono](https://www.youtube.com/watch?v=yRPl4zy9z1c) справа снизу;
- публикует **RTK только для RViz** (красный `rtk_path`), **не** в odom.

**Чистый VINS без RTK в RViz:**

```bash
./scripts/run_vins_mars_live.sh 1.0 50 --play 90 --no-rtk
```

**Без окна odom** (только RViz):

```bash
./scripts/run_vins_mars_live.sh 1.0 50 --play 90 --no-echo-odom
```

| Аргумент | Значение |
|----------|----------|
| `1.0` | Скорость bag: 1 секунда записи = 1 секунда реального времени |
| `50` | Пропустить первые 50 с bag (до этого VINS часто не инициализируется) |
| `--play` | Bag запускается автоматически (без второго терминала) |
| `90` | Играть 90 секунд и остановить |

**Порядок действий:**

1. Вставить команду в терминал, Enter
2. Подождать **10–15 с** — откроется RViz
3. Смотреть картинки и 3D (см. §2.6)
4. Когда в терминале «Bag закончен» — **Ctrl+C**

### 2.2. Полный полёт (~15 мин)

```bash
cd ~/Downloads/cursor/Projects/fire-dron
./scripts/run_vins_mars_live.sh 0.3 50 --play
```

`0.3` — bag в 3 раза медленнее реального времени (~15 мин wall time на весь остаток записи). Будет много reboot VINS — это нормально для надира.

### 2.3. Меньше сбоев на коротком окне

```bash
cd ~/Downloads/cursor/Projects/fire-dron
./scripts/run_vins_mars_live.sh 0.5 55 --play 60
```

Старт с **55 с** — дрон уже на высоте, меньше `IMU excitation not enough`. 60 с записи — короткое окно без длинных участков с поворотами камеры.

### 2.4. Без автоматического bag (два терминала)

Удобно, если нужно **пауза** перед стартом bag или ручной контроль скорости.

**Терминал 1** — VINS + RViz (без `--play`):

```bash
cd ~/Downloads/cursor/Projects/fire-dron
./scripts/run_vins_mars_live.sh 0.3 50
```

Скрипт напечатает команду для терминала 2 и будет ждать.

**Терминал 2** — после появления RViz:

```bash
source ~/Downloads/cursor/Projects/fire-dron/scripts/project.sh
vnav_setup_ros
rosbag play --clock -r 0.3 -s 50 ~/Downloads/cursor/Projects/fire-dron/data/mars/mars_hkairport03.bag
```

`--clock` обязателен — синхронизирует время ROS с записью.

### 2.5. Остановка

**Ctrl+C** в терминале 1 — завершит VINS, RViz и RTK publisher.

После окончания bag сообщения `Connection refused` — нормально, просто нажмите Ctrl+C.

### 2.6. Что смотреть в RViz

| Панель (Displays слева) | Что это | Когда появляется |
|-------------------------|---------|------------------|
| **tracked image** | Кадр с цветными точками трекинга | После `Initialization finish!` |
| **raw_image** | Сырой кадр с камеры | Сразу при play bag |
| **loop_match_image** | Два кадра + линии loop closure | При срабатывании pose_graph |
| **VIO → Path** | Зелёная 3D-траектория VINS | После инициализации |
| **rtk_path** | Красный пунктир — RTK эталон | Когда VINS публикует odom |
| **pose_graph** | Уточнённая траектория после loop | Периодически |

**Управление 3D:** ЛКМ — вращение, колёсико — zoom, СКМ — сдвиг.

**Если зелёные линии «стреляют вверх»** — это **reboot** VINS (сброс координат). На надирном MARS бывает часто; это не ошибка запуска.

### 2.8. Окно odometry (как справа в видеo VINS-Mono)

При запуске `run_vins_mars_live.sh` по умолчанию открывается **второй терминал** с `rostopic echo /vins_estimator/odometry`.

| Поле | Смысл |
|------|--------|
| `pose.position x,y,z` | Позиция в локальной СК VINS (`world`) |
| `pose.orientation` | Ориентация |
| `twist` | Скорости |

**Чистый VINS** — RTK в odom не входит. `/mars/rtk_path` — только для RViz. Fusion — offline (`fuse_vins_rtk.py`).

```bash
./scripts/run_vins_mars_live.sh 1.0 50 --play 90 --no-rtk
./scripts/run_vins_mars_live.sh 1.0 50 --play 90 --no-echo-odom
```

### 2.9. Сообщения в терминале (не паниковать)

| Сообщение | Объяснение |
|-----------|------------|
| `throw img, only should happen at the beginning` | Кадры в начале отбрасываются — норма |
| `IMU excitation not enough!` | Мало движения для калибровки IMU в начале |
| `Initialization finish!` | VINS **запустился** — скоро появятся треки |
| `failure detection!` / `system reboot!` | VINS перезапустился из-за скачка — ожидаемо на MARS |
| `misalign visual structure with IMU` | Визуал и IMU разошлись — reboot |
| `Bag закончен` | Запись доиграна — Ctrl+C |

---

## 3. Готовое видео и отчёт (без RViz)

Если RViz «ломаный» или нужен результат для отчёта — смотрите заранее собранные файлы.

### MARS HK — 150 с

Три панели: VINS точки, DSO точки, KLT-треки. Полёт над Гонконгом.

```bash
cd ~/Downloads/cursor/Projects/fire-dron
xdg-open results/combined_run8e_fused/mars_viz/mars_hk_flight.avi
```

Полный путь (если забыли `cd`):

```bash
xdg-open ~/Downloads/cursor/Projects/fire-dron/results/combined_run8e_fused/mars_viz/mars_hk_flight.avi
```

Файл ~450 МБ, откроется в VLC/mpv.

### Поворот_коптер — 51 с

DSO на наборе без IMU — поворот камеры.

```bash
cd ~/Downloads/cursor/Projects/fire-dron
xdg-open results/combined_run8e_fused/povorot_viz/povorot_flight.avi
```

### PDF-отчёт

Итоговые метрики run8e, графики траекторий, сравнение с RTK.

```bash
cd ~/Downloads/cursor/Projects/fire-dron
xdg-open ОТЧЁТ_ОБЪЕДИНЁННЫЙ_АЛГОРИТМ.pdf
```

---

## 4. Batch-прогон (метрики, без GUI)

Используйте, когда нужны **числа** (ATE, покрытие, CSV) и **артефакты** (PDF, видео), а не live-просмотр.

### 4.1. Полный пайплайн MARS run8e

```bash
cd ~/Downloads/cursor/Projects/fire-dron

# Шаг 1: VINS (~15 мин wall time при rate 0.3)
./scripts/run_vins_mars.sh 0.3 50 run8e_parallax

# Шаг 2: DSO на MARS (~минуты)
./scripts/run_dso_mars.sh 0 run8e_fused

# Шаг 3: RTK fusion + evo ATE
./scripts/production_mars.sh results/vins_mars_run8e_parallax.csv run8e_fused

# Шаг 4: Диспетчер, пересборка видео, PDF
./scripts/finalize_run8.sh run8e_parallax run8e_fused
```

| Шаг | Результат | Где лежит |
|-----|-----------|-----------|
| 1 | Траектория VINS (2734 поз) | `results/vins_mars_run8e_parallax.csv` |
| 2 | Траектория DSO | `results/dso_mars_run8e_fused.tum` |
| 3 | Fused TUM + ATE | `results/eval_mars_run8e_fused/` |
| 4 | Видео, PDF, dispatcher JSON | `results/combined_run8e_fused/`, корень PDF |

**Аргументы `run_vins_mars.sh`:** `[скорость] [старт_сек] [метка] [длительность_сек]`

Пример: `0.3 50 run8e_parallax` — rate 0.3, старт 50 с, метка для имён файлов, до конца bag.

### 4.2. Быстрая проверка VINS на 90 с

Не пересчитывает весь bag — только короткое окно:

```bash
cd ~/Downloads/cursor/Projects/fire-dron
./scripts/run_vins_mars.sh 1.0 50 test_q 90
```

Сравнение с эталоном на том же окне:

```bash
python3 tools/compare_segment.py \
    --gt data/mars/mars_hkairport03_gt.tum \
    --start 50 --duration 90 --fuse \
    results/vins_mars_run8e_parallax.csv \
    results/vins_mars_test_q.csv
```

---

## 5. Поворот_коптер (только DSO)

Набор **без IMU** — VINS не запускается. DSO + диспетчер.

```bash
cd ~/Downloads/cursor/Projects/fire-dron

./scripts/run_dso_povorot.sh 0 run8e_fused

python3 tools/dispatcher.py \
    /dev/null \
    results/dso_povorot_run8e_fused.tum \
    513 nadyr
```

| Аргумент dispatcher | Значение |
|---------------------|----------|
| `/dev/null` | VINS нет — пустой CSV |
| `513` | Число кадров в наборе |
| `nadyr` | Тип сцены (надир) |

**Результат:** JSON с выбором DSO и `health: low_confidence`. Абсолютного эталона нет — оценивается дрейф (~40% пути).

---

## 6. Проверка данных

Перед первым запуском убедитесь, что датасеты на месте:

```bash
cd ~/Downloads/cursor/Projects/fire-dron

# Bag MARS (IMU + камера для rosbag play)
ls -lh data/mars/mars_hkairport03.bag

# RTK эталон (timestamp tx ty tz ...)
head -3 data/mars/mars_hkairport03_gt.tum

# Кадры для офлайн-видео
ls data/mars/camera/images/ | head

# Поворот_коптер
ls data/povorot_kopter/images/ | head
```

**Ожидаемо:**

- bag — сотни МБ / несколько ГБ
- `mars_hkairport03_gt.tum` — тысячи строк, Z растёт при взлёте
- `camera/images/` — тысячи `.png` / `.jpg`

Если bag нет — его нужно скачать/собрать с [MARS-LVIG](https://mars.hku.hk/dataset.html).

---

## 7. Типичные ошибки

### «Нет bag» / «Нет такого файла или каталога»

**Причина:** команда из `~`, а не из `fire-dron`.

```bash
cd ~/Downloads/cursor/Projects/fire-dron
```

То же для `xdg-open results/...` — всегда сначала `cd` в проект.

### «catkin_make / roslaunch / rosbag: command not found»

**Причина:** ROS не подключён в этом терминале.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### Видео не открывается

**Причина:** путь `~/results/...` — такой папки нет. Видео **внутри проекта**:

```bash
xdg-open ~/Downloads/cursor/Projects/fire-dron/results/combined_run8e_fused/mars_viz/mars_hk_flight.avi
```

### RViz пустой / только сетка

1. Подождите 30–60 с после `Initialization finish!`
2. Проверьте, что bag играет (нет `--play` → нужен второй терминал)
3. В Displays включите **tracked image**, **VIO**, **pose_graph**

### «failure detection» / «system reboot» / линии вверх в 3D

**Причина:** надир + поворот камеры — VINS теряет трек. Ожидаемо на MARS.

Попробуйте короче и позже:

```bash
./scripts/run_vins_mars_live.sh 0.5 55 --play 60
```

Или готовое видео:

```bash
xdg-open results/combined_run8e_fused/mars_viz/mars_hk_flight.avi
```

### «Connection refused» после окончания bag

Bag закончился, часть нод ещё работает. **Ctrl+C** в терминале со скриптом.

### «Not enough features or parallax» / инициализация не сходится

**Причина:** старт bag слишком рано (дрон на земле, мало текстуры).

Используйте **`-s 50`** или **`-s 55`** в rosbag / `run_vins_mars*.sh`.

---

## 8. Справка по скриптам

| Скрипт | Когда использовать | Выход |
|--------|---------------------|-------|
| `run_vins_mars_live.sh` | Первый раз, демо, отладка в RViz | Окно RViz, логи в терминале |
| `run_vins_mars.sh` | Получить траекторию VINS | `results/vins_mars_*.csv` |
| `run_dso_mars.sh` | DSO на MARS | `results/dso_mars_*.tum` |
| `run_dso_povorot.sh` | DSO на Поворот_коптер | `results/dso_povorot_*.tum` |
| `production_mars.sh` | RTK fusion + ATE | `results/eval_mars_*/vins_fused.tum` |
| `finalize_run8.sh` | Видео + PDF + диспетчер | `combined_run8e_fused/`, PDF |

---

## 9. Что отправлять на GitHub

В репозиторий идут **код, config, docs** (в т.ч. этот файл).

**Не в git** (см. `.gitignore`): bag, кадры, большие `.avi` — через Git LFS или отдельное хранилище.

---

Подробнее: [README.md](../README.md) · [rviz_live.md](rviz_live.md) · [architecture.md](architecture.md)
