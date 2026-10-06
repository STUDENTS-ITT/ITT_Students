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

    export/plot_exporter.cpp
    export/plot_exporter.h
    export/pdf_report.cpp
    export/pdf_report.h
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
# Headless-прогон всего GUI (окно, графики, SQLite) на штатном датасете.
# --self-test не показывает окно, выполняет расчёт и выходит с кодом 0/1/2,
# отчёт с логом пишется в файл (консольный вывод GUI-приложения на Windows
# при перенаправлении теряется).
# ---------------------------------------------------------------------------
if(BUILD_TESTING AND EXISTS ${CMAKE_SOURCE_DIR}/data/raw/imu.dat)
    set(ITT_GUI_SMOKE_DIR ${CMAKE_CURRENT_BINARY_DIR}/smoke)

    add_test(NAME gui_smoke COMMAND imitator_gui
        --self-test
        --self-test-timeout=300
        --self-test-report=${ITT_GUI_SMOKE_DIR}/report.txt
        --data ${CMAKE_SOURCE_DIR}/data/raw
        --output ${ITT_GUI_SMOKE_DIR})

    set_tests_properties(gui_smoke PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
        TIMEOUT 400)

    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
        set_tests_properties(gui_smoke PROPERTIES ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
    endif()
endif()