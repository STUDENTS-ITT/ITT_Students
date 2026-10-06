# Руководство для программиста DEVELOPER_GUIDE.md

## 1. Цель
Настоящий документ описывает архитектуру, правила разработки, процесс сборки, тестирования и развёртывания Qt-версии проекта imitator_gui. Предназначен для разработчиков, вносящих изменения в кодовую базу.

## 2. Общая информация
- Рабочий проект: C:\ITT_Students_Qt_Version
- Эталонные исходники (только для чтения): C:\ITT_Students
- Цель: Qt 6.12.0 GUI-обвязка поверх существующего ESKF/БИНС-СНС комплекса
- Ядро должно оставаться Qt-независимым. Весь Qt-код – только в src/gui/

## 3. Архитектура
Согласно docs/qt_integration_proposal.md проект разделён на слои:

- src/core (Qt-free): ins/, nav/, math_lib/, utils/, data_io/. Статическая библиотека esfcore (C++17, STL). Запрещены #include <Q...>
- src/console: консольный main.cpp, цель imitator. Линкуется с esfcore. Сохраняет обратную совместимость
- src/gui (Qt): QtWidgets-приложение imitator_gui. Использует QtConcurrent, QSettings, QtSql. Графики – только QPainter (без QtCharts/QtGraphs/QtQuick3D)

## 4. Конвенции core
- ESKF – 15 состояний (не менять без обоснования)
- Матрицы хранятся построчно (row-major)
- Явная передача числа столбцов: multiply_matrix(A,B,cols_A,cols_B), transpose_m(A,cols), multiply_m(A,v,cols_A), at(A,i,j,cols)
- Углы в ядре – радианы. На выходе – градусы (RAD_TO_DEG)
- Комментарии на русском языке, без избыточных восклицательных знаков
- Сохранение бит-в-бит идентичности консольного вывода (result.txt, reference.txt, errors.txt) при корректном рабочем каталоге

## 5. Структура
`
C:\ITT_Students_Qt_Version\
  src/
    core/      # Qt-free ядро
    console/   # консольная версия
    gui/       # GUI на Qt
  tests/       # unit-тесты
  docs/        # документация
  dist/        # портативная сборка
  CMakeLists.txt
  USER_GUIDE.md
  DEVELOPER_GUIDE.md
`

## 6. Окружение сборки
- Windows 10/11 x64
- Qt 6.12.0 mingw_64: C:/QT6.10/6.12.0/mingw_64
- MinGW GCC 16.1.0: C:/Users/anton/.mingw64/mingw64/bin
- CMake >= 3.16
- Ninja (рекомендуется)

## 7. Конфигурация и сборка
`ash
# Конфигурация
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH="C:/QT6.10/6.12.0/mingw_64"

# Полная сборка
cmake --build build

# Отдельные цели
cmake --build build --target esfcore
cmake --build build --target imitator
cmake --build build --target imitator_gui
`

## 8. Тестирование
`ash
# Запуск всех тестов (GUI-тесты в offscreen)
set QT_QPA_PLATFORM=offscreen
ctest --test-dir build --output-on-failure
`

Тестовый набор (11):
- core_has_no_qt – проверка отсутствия Qt-зависимостей в core/
- gui_smoke – базовый smoke-тест GUI
- test_matrix_ops, test_transformations, test_statistics, test_ins_filter
- test_data_io, test_settings, test_simulation
- test_plots – кэш PlotView, курсор, инвалидация
- test_pdf_report – PDF-сводка, восстановление canvas, сигнатура

Все должны быть пройдены.

## 9. PlotView (графики и кэш)
Файлы: src/gui/views/plots/plot_view.h/.cpp
- Ключ кэша включает: версию состояния данных, размер виджета, диапазоны cache_x0/cache_x1/cache_y0/cache_y1
- API: setCacheEnabled(bool), isCacheEnabled(), cacheHits(), cacheMisses(), invalidateCache(), isCacheValid(), rebuildCache()
- Поведение при cache_enabled=false: прямая отрисовка, QPixmap не создаётся/не используется, кэш очищается, счётчики не инкрементируются
- Курсор мыши (mouseMove/hover) дорисовывается поверх кэшированного кадра при первом промахе кэша – не попадает в QPixmap
- Инвалидация при любых изменениях, влияющих на содержимое/диапазоны (смена данных, добавление/удаление рядов, масштаб/скролл, resize)

## 10. PDF-отчёт
src/gui/export/pdf_report.h/.cpp
- QPdfWriter, формат A4
- PdfReport::summarize() – сводка NIS/NEES, RMS/максимумы по 15 компонентам
- PdfReport::build() – генерация титульного листа, таблиц, графиков
- Вставка графиков через QWidget::renderTo() (либо через PlotBlock с QVector<PlotSeries> + sync_markers)
- Сохранение и восстановление состояния canvas: список рядов и sync_markers (после экспорта GUI не меняется)
- dof = 9 (ins::KF_MEAS)

Интеграция: MainWindow::onExportReport(), buildReportBlocks(), exportReportTo(...)

## 11. Модель истории и фильтрация
- src/gui/models/history_model.h/.cpp – QAbstractTableModel
- src/gui/views/params_panel.h/.cpp – QSortFilterProxyModel поверх HistoryModel
- Фильтр: по имени/дате/каталогу
- Сортировка: по дате по умолчанию
- mapToSource() используется корректно при выборе записей (особенно множественный выбор)

## 12. Self-test GUI
src/gui/main.cpp: при старте создаёт временный PDF, проверяет наличие %PDF. При успехе приложение запускается (exit 0 в тестовом сценарии не требуется для нормального запуска, но логика проверки реализована).

## 13. Портативная сборка (dist) и deploy
Таргет deploy в корневом CMakeLists.txt:
- Копирует imitator_gui.exe, imitator.exe в dist/
- Запускает windeployqt (копирует Qt DLL, плагины platforms/qwindows.dll,qoffscreen.dll,qminimal.dll, sqldrivers, imageformats, tls и др.)
- Копирует MinGW runtime: libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll
- Копирует USER_GUIDE.md в dist/

Сборка:
`ash
cmake --build build --target deploy
`
Проверка: запустить dist\imitator_gui.exe с чистым PATH (двойным щелчком) – self-test проходит, приложение работает.

## 14. Паритет с консольной версией
- Причина прежних расхождений – рабочий каталог (cwd), а не регрессия
- В оригинальном Aligner.cpp при недоступном ../data/processed/Aligner.dat – ранний return (обнуление начальных углов/ba)
- При корректном cwd (есть доступ к ../data/processed, типично build-каталог) result.txt/reference.txt/errors.txt побайтово совпадают между оригинальным imitator и Qt-версией и с verified baseline
- Для одновременного запуска – **разные** рабочие каталоги (иначе выходные файлы в tools/ перезаписываются)

## 15. Правила разработки
- core/ – только STL/C++17. Без Qt. core_has_no_qt должен проходить
- Сохранять алгоритмы ESKF и матричные конвенции (row-major + явное число столбцов)
- Углы: радианы внутри, градусы на выходе
- Графика GUI – только QPainter
- Минимальные изменения. Соблюдать существующий стиль кода
- Комментарии – на русском
- После правок – прогнать ctest (особенно test_plots, test_pdf_report при правках GUI)
- Не коммитить без явного запроса пользователя
- Сборка в Release, без ошибок и предупреждений

## 16. Полезные файлы для навигации
- docs/qt_integration_proposal.md – детальный план интеграции
- AGENTS.md – технический контекст по ESKF
- USER_GUIDE.md – руководство для пользователя
- src/gui/main.cpp – точка входа + self-test
- src/gui/views/plots/plot_view.* – графики и кэш
- src/gui/export/pdf_report.* – экспорт PDF
- src/gui/views/params_panel.* – история/фильтр
- src/core/navigation/simulation.* – Qt-free файловый раннер
- src/core/data_io/data_reader.* – файловые ридеры

## 17. Типичные сценарии отладки
- Сломался кэш графика: проверить invalidateCache во всех мутаторах, влияющих на данные/диапазоны
- PDF пустой/неправильный: проверить восстановление canvas (ряды + sync_markers) после renderTo()
- Фильтр истории работает странно: проверить mapToSource() при множественном выборе
- Расхождение с консолью: первым делом сверить рабочий каталог запуска