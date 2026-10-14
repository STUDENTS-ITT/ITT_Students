# imitator_gui — Qt 6.12.0 приложение поверх ядра esfcore.
#
# Используются только LGPL-модули Qt: Core, Gui, Widgets, Concurrent, Sql,
# PrintSupport, Svg. Графики рисуются собственным кодом на QPainter
# (без QtCharts/QtGraphs/QtQuick3D).
#
# Развёртывание: windeployqt build/imitator_gui.exe — Qt6::Core/Gui/Widgets
# статически линкуются, переносить нужно плагины (platforms, imageformats,
# sqldrivers) и Qt6Sql.dll / Qt6Concurrent.dll.

qt_standard_project_setup()

qt_add_executable(imitator_gui
    main.cpp
    MainWindow.cpp
    MainWindow.h

    core_bridge/run_store.cpp
    core_bridge/run_store.h
    core_bridge/input_tree.cpp
    core_bridge/input_tree.h
    core_bridge/telemetry_buffer.cpp
    core_bridge/telemetry_buffer.h

    workers/simulation_worker.cpp
    workers/simulation_worker.h

    models/nav_table_model.cpp
    models/nav_table_model.h
    models/params_table_model.cpp
    models/params_table_model.h
    models/history_model.cpp
    models/history_model.h

    settings/app_settings.cpp
    settings/app_settings.h
    settings/dataset_truth.cpp
    settings/dataset_truth.h
    settings/preset_meta.cpp
    settings/preset_meta.h

    views/plots/plot_view.cpp
    views/plots/plot_view.h
    views/plots/time_series_plot.cpp
    views/plots/time_series_plot.h
    views/plots/trajectory_plot.cpp
    views/plots/trajectory_plot.h

    views/control_panel.cpp
    views/control_panel.h
    views/log_panel.cpp
    views/log_panel.h
    views/state_panel.cpp
    views/state_panel.h
    views/data_panel.cpp
    views/data_panel.h
    views/params_panel.cpp
    views/params_panel.h
    views/tuning_panel.cpp
    views/tuning_panel.h

    export/plot_exporter.cpp
    export/plot_exporter.h
    export/matrices_exporter.cpp
    export/matrices_exporter.h
    export/animation_exporter.cpp
    export/animation_exporter.h
    export/gif_writer.cpp
    export/gif_writer.h
    export/pdf_report.cpp
    export/pdf_report.h
    export/tuning_report.cpp
    export/tuning_report.h
)

target_link_libraries(imitator_gui PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Concurrent
    Qt6::Sql
    Qt6::PrintSupport
    Qt6::Svg
    esfcore
)

target_include_directories(imitator_gui PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(MSVC)
    target_compile_options(imitator_gui PRIVATE /utf-8)
endif()

# ---------------------------------------------------------------------------
# Headless-прогон всего GUI (окно, графики, SQLite). Перед прогоном тестовый
# набор синтезируется генератором ядра во временный каталог, поэтому smoke-тест
# не зависит от data/raw в репозитории.
# --self-test не показывает окно, выполняет расчёт и выходит с кодом 0/1/2,
# отчёт с логом пишется в файл (консольный вывод GUI-приложения на Windows
# при перенаправлении теряется).
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    set(ITT_GUI_SMOKE_DIR ${CMAKE_CURRENT_BINARY_DIR}/smoke)

    add_test(NAME gui_smoke_generate COMMAND imitator_gui
        --self-test-generate=${ITT_GUI_SMOKE_DIR}/data
        --self-test-timeout=120)

    add_test(NAME gui_smoke COMMAND imitator_gui
        --self-test
        --self-test-timeout=300
        --self-test-report=${ITT_GUI_SMOKE_DIR}/report.txt
        --data ${ITT_GUI_SMOKE_DIR}/data
        --output ${ITT_GUI_SMOKE_DIR}/out)

    set_tests_properties(gui_smoke_generate PROPERTIES
        FIXTURES_SETUP gui_smoke_data
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
        TIMEOUT 200)

    set_tests_properties(gui_smoke PROPERTIES
        FIXTURES_REQUIRED gui_smoke_data
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
        TIMEOUT 400)

    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
        set_tests_properties(gui_smoke_generate gui_smoke PROPERTIES ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
    endif()
endif()