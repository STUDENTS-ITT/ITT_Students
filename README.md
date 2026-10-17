# Навигация (SINS) — ITT_Students (Qt-версия)

Комплексирование видео, навигационной системы и БИНС. Консольный `imitator` и Qt-GUI `imitator_gui` работают на общем Qt-независимом ядре ESKF (`esfcore`).

Данные полётов: https://cloud.mail.ru/public/kcvi/SZwC1emkR

## Состав
- `src/core/` — ядро ESKF (C++17, STL, **без Qt**), статическая библиотека `esfcore`.
- `src/console/` — консольная цель `imitator`.
- `src/gui/` — Qt 6 (QtWidgets, QtConcurrent, QSettings, QtSql, QtPdf), цель `imitator_gui`; графика на `QPainter`.

## Сборка
```bash
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH="C:/QT6.10/6.12.0/mingw_64"
cmake --build build
```

## Запуск и тесты
```bash
./build/imitator.exe        # консоль (данные в ../data/raw/)
./build/imitator_gui.exe    # GUI
ctest --test-dir build --output-on-failure   # QT_QPA_PLATFORM=offscreen при необходимости
```

## Документация
- `Theory_of_Work/README.md` — среда документации: руководства пользователя, алгоритм, теория.
- `Theory_of_Work/user/FIRST_RUN.md` — **с чего начать**: руководство по первому запуску, сквозной проход по всему функционалу GUI.
- `Theory_of_Work/user/FIRST_RUN_GENERATE.md` — пошаговый первый прогон с **генерацией** новых данных (без внешних файлов).
- `Theory_of_Work/user/USER_GUIDE.md` — руководство пользователя GUI.
- `Theory_of_Work/user/NAVIGATION.md` — подробная навигация по интерфейсу (кнопки, вкладки, меню, сценарии).
- `Theory_of_Work/user/DATA_FORMATS.md` — форматы входных/выходных данных.
- `Theory_of_Work/algorithm/README.md` — руководство по алгоритму: все файлы, классы и функции ядра (`esfcore`) и консоли.
- `Theory_of_Work/theory/README.md` — теория, на которую опирается код: СК и углы, уравнения БИНС, выставка, ESKF, СНС.
- `DEVELOPER_GUIDE.md` — архитектура, ключевые классы, сборка, отладка.
- `docs/DEVELOPMENT_PLAN.md` — план развития.
- `AGENTS.md` — контекст для ИИ по проекту.
- `PROJECT_MAP.md` — карта репозитория: что где лежит, что вне git и что генерируется.
