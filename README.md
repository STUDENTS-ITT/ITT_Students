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
- `USER_GUIDE.md` — руководство пользователя GUI (также `docs/user_manual/`, `dist/`).
- `docs/user_manual/NAVIGATION.md` — подробная навигация по интерфейсу (кнопки, вкладки, меню, сценарии).
- `DEVELOPER_GUIDE.md` — архитектура, ключевые классы, сборка, отладка.
- `docs/user_manual/README.txt` — форматы входных/выходных данных.
- `docs/DEVELOPMENT_PLAN.md` — план развития.
- `AGENTS.md` — контекст для ИИ по проекту.
