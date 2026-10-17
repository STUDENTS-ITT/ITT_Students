# Theory of Work — документация проекта

Среда документации по принципу работы проекта: как устроена система, как ей
пользоваться и по каким алгоритмам она считает. Отделяет **руководства пользователя**
и **описание алгоритма** от проектной переписки в `docs/`.

## Структура

```
Theory_of_Work/
├── README.md        — этот файл: оглавление и навигация
├── user/            — руководства пользователя
│   ├── FIRST_RUN.md            — первый запуск GUI на готовых данных
│   ├── FIRST_RUN_GENERATE.md   — первый прогон с генерацией данных
│   ├── USER_GUIDE.md           — руководство пользователя GUI
│   ├── NAVIGATION.md           — карта интерфейса (кнопки, вкладки, меню)
│   └── DATA_FORMATS.md         — форматы входных/выходных файлов
├── algorithm/       — руководство по алгоритму (по коду ядра) — см. ниже
└── theory/          — теоретическая справка: БИНС, ESKF, СНС — см. ниже
```

### Руководство по алгоритму (`algorithm/`)

Справочник по коду `esfcore` и консоли: все файлы, классы, функции, переменные.
Начинать с [`algorithm/README.md`](algorithm/README.md) — обзор, поток данных и
порядок такта; далее по модулям:

| Документ | Содержание |
|---|---|
| [`algorithm/utils.md`](algorithm/utils.md) | типы, константы, пути, INI-парсер |
| [`algorithm/math_lib.md`](algorithm/math_lib.md) | матрицы, статистика, преобразования СК |
| [`algorithm/ins.md`](algorithm/ins.md) | ориентация, ИМУ, фильтр Калмана 15-го порядка |
| [`algorithm/navigation.md`](algorithm/navigation.md) | счисление, такт `step()`, прогон |
| [`algorithm/navigation_alignment.md`](algorithm/navigation_alignment.md) | начальная выставка |
| [`algorithm/navigation_tilt.md`](algorithm/navigation_tilt.md) | коррекция тангажа/крена после манёвра |
| [`algorithm/navigation_gnss_guard.md`](algorithm/navigation_gnss_guard.md) | защита от отказов СНС |
| [`algorithm/navigation_settings.md`](algorithm/navigation_settings.md) | настройки фильтра (`settings.ini`) |
| [`algorithm/data_io.md`](algorithm/data_io.md) | чтение/запись файлов, генератор данных |
| [`algorithm/console.md`](algorithm/console.md) | консольный `main`, сборка |

### Теоретическая справка (`theory/`)

Теория, на которую опирается код: вывод формул, реализованных в `esfcore`.
Начинать с [`theory/README.md`](theory/README.md) — обозначения и карта; далее:

| Документ | Содержание |
|---|---|
| [`theory/coordinate_frames.md`](theory/coordinate_frames.md) | СК, углы Эйлера–Крылова, матрица `C_б^н`, кинематика углов |
| [`theory/ins_equations.md`](theory/ins_equations.md) | уравнения счисления, вредные ускорения, модель Земли |
| [`theory/alignment.md`](theory/alignment.md) | начальная выставка (аксель + вращение Земли) |
| [`theory/estimation.md`](theory/estimation.md) | ESKF 15 состояний, `F`/`Q`/`R`, NIS/NEES, reset |
| [`theory/gnss.md`](theory/gnss.md) | модель измерений СНС, отсев аномальных кадров |

## Навигация

| Хочу… | Документ |
|---|---|
| Впервые запустить приложение | [`user/FIRST_RUN.md`](user/FIRST_RUN.md) |
| Сгенерировать данные и прогнать без внешних файлов | [`user/FIRST_RUN_GENERATE.md`](user/FIRST_RUN_GENERATE.md) |
| Разобраться в функциях и сценариях GUI | [`user/USER_GUIDE.md`](user/USER_GUIDE.md) |
| Найти конкретную кнопку или вкладку | [`user/NAVIGATION.md`](user/NAVIGATION.md) |
| Понять форматы `imu.dat`/`gps.dat`/`angle.dat` и выходных файлов | [`user/DATA_FORMATS.md`](user/DATA_FORMATS.md) |
| Понять, как работает фильтр и навигация | [`algorithm/README.md`](algorithm/README.md) |
| Найти конкретный файл/функцию ядра | таблица в [`algorithm/README.md`](algorithm/README.md) |
| Освежить теорию (БИНС, углы, оценивание) | [`theory/README.md`](theory/README.md) |

Смежное: карта всего репозитория — `../PROJECT_MAP.md`; архитектура кода и отладка —
`../DEVELOPER_GUIDE.md`; правила работы с ядром — `../AGENTS.md`.

## Статус

- Руководства пользователя перенесены в `user/` (2026-10-17) и являются единственным
  каноническим экземпляром.
- Руководство по алгоритму — `algorithm/` (2026-10-17): обзор (`README.md`) плюс
  справочники по модулям `utils`, `math_lib`, `ins`, `navigation`
  (включая `alignment`, `tilt`, `gnss_guard`, `settings`), `data_io`, `console`.
  Описаны все файлы, классы, функции и ключевые переменные ядра `esfcore`.
- Теоретическая справка — `theory/` (2026-10-17): `README.md` плюс документы по
  системам координат, уравнениям БИНС, выставке, оцениванию (ESKF) и СНС.
  Формулы привязаны к файлам и функциям ядра.
