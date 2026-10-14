# tests — модульные тесты ядра (esfcore) на QtTest.
#
# Ядро не зависит от Qt, но тесты используют QtTest (LGPL, Qt 6.12):
# он даёт готовый раннер, сравнения и отчёт по упавшим проверкам.
# Тесты подключаются к ctest: `ctest --test-dir build`.

find_package(Qt6 REQUIRED COMPONENTS Test)

# AUTOMOC для #include "<test>.moc" в конце каждого файла теста.
qt_standard_project_setup()

set(ITT_TESTS
    test_matrix_ops
    test_transformations
    test_statistics
    test_ins_filter
    test_data_io
    test_data_source
    test_settings
    test_simulation
    test_dataset_generator
)

foreach(test_name IN LISTS ITT_TESTS)
    qt_add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE Qt6::Test esfcore)
    target_include_directories(${test_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
    # Qt и рантайм MinGW лежат вне каталога сборки — добавляем их в PATH теста.
    # Две отдельные операции, чтобы не вставлять ';' внутрь значения.
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
        set_tests_properties(${test_name} PROPERTIES ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
    endif()
endforeach()

# Этап 12: sanity-прогон на реальном наборе нового формата (.data).
# Тест пропускается (QSKIP), если каталога нет на диске.
target_compile_definitions(test_data_source PRIVATE
    ITT_LINE_DATASET_DIR="${CMAKE_SOURCE_DIR}/ITT_Students-Katya/imitator_sq/test/line_dataset")

# Графики на QPainter живут в Qt-слое, поэтому их тест собирает исходники
# напрямую (esfcore к графикам отношения не имеет).
qt_add_executable(test_plots
    test_plots.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/plot_view.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/time_series_plot.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/trajectory_plot.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/export/plot_exporter.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/export/animation_exporter.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/export/gif_writer.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/models/nav_table_model.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/core_bridge/telemetry_buffer.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/core_bridge/run_store.cpp
)
target_link_libraries(test_plots PRIVATE Qt6::Test Qt6::Widgets Qt6::Svg Qt6::PrintSupport Qt6::Sql esfcore)
target_include_directories(test_plots PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}
)
add_test(NAME test_plots COMMAND test_plots)
set_tests_properties(test_plots PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 120)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    set_tests_properties(test_plots PROPERTIES ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
endif()

# Хранилище истории прогонов живёт в Qt-слое: собираем исходники напрямую
# (как test_plots), тест проверяет снимок параметров и миграцию схемы.
qt_add_executable(test_run_store
    test_run_store.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/core_bridge/run_store.cpp
)
target_link_libraries(test_run_store PRIVATE
    Qt6::Test Qt6::Sql esfcore)
target_include_directories(test_run_store PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}
)
add_test(NAME test_run_store COMMAND test_run_store)
set_tests_properties(test_run_store PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 60)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    set_tests_properties(test_run_store PROPERTIES ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
endif()

# Сводный PDF-отчёт — тоже код Qt-слоя, поэтому собираем его исходники
# напрямую вместе с графиками (esfcore нужен для ins::KF_MEAS и RunSeries).
qt_add_executable(test_pdf_report
    test_pdf_report.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/plot_view.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/time_series_plot.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/export/pdf_report.cpp
)
target_link_libraries(test_pdf_report PRIVATE
    Qt6::Test Qt6::Widgets Qt6::PrintSupport Qt6::Svg Qt6::Sql esfcore)
target_include_directories(test_pdf_report PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}
)
add_test(NAME test_pdf_report COMMAND test_pdf_report)
set_tests_properties(test_pdf_report PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 60)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    set_tests_properties(test_pdf_report PROPERTIES ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
endif()

# PDF-отчёт о подборе — тоже код Qt-слоя; собираем его исходники напрямую
# вместе с графиком сходимости (TimeSeriesPlot). Сам подбор тест не запускает:
# рядом с синтетическим конфигом раскладываются best_params.txt и eval.json.
qt_add_executable(test_tuning_report
    test_tuning_report.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/plot_view.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/time_series_plot.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/export/tuning_report.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/settings/preset_meta.cpp
)
target_link_libraries(test_tuning_report PRIVATE
    Qt6::Test Qt6::Widgets Qt6::PrintSupport Qt6::Svg esfcore)
target_include_directories(test_tuning_report PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}
)
add_test(NAME test_tuning_report COMMAND test_tuning_report)
set_tests_properties(test_tuning_report PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 60)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    set_tests_properties(test_tuning_report PROPERTIES ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
endif()

# Экспорт статических матриц фильтра (Q/R/P0/H) — код GUI-слоя, собираем
# исходник напрямую (как test_run_store); QtCore достаточно.
qt_add_executable(test_matrices_exporter
    test_matrices_exporter.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/export/matrices_exporter.cpp
)
target_link_libraries(test_matrices_exporter PRIVATE
    Qt6::Test Qt6::Core esfcore)
target_include_directories(test_matrices_exporter PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}
)
add_test(NAME test_matrices_exporter COMMAND test_matrices_exporter)
set_tests_properties(test_matrices_exporter PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 60)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    set_tests_properties(test_matrices_exporter PROPERTIES ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
endif()

# Пути по умолчанию GUI-настроек (appRootDir и каталог отчётов reports/) —
# код Qt-слоя, собираем исходник напрямую (как test_matrices_exporter).
qt_add_executable(test_app_settings
    test_app_settings.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/settings/app_settings.cpp
)
target_link_libraries(test_app_settings PRIVATE
    Qt6::Test Qt6::Widgets)
target_include_directories(test_app_settings PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}
)
add_test(NAME test_app_settings COMMAND test_app_settings)
set_tests_properties(test_app_settings PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 60)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    set_tests_properties(test_app_settings PROPERTIES ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
endif()

# Проверка <data_dir>/truth.txt (вкладка «Подбор» умеет сравнивать с истиной
# только при корректном файле) — код Qt-слоя, собираем исходник напрямую.
qt_add_executable(test_dataset_truth
    test_dataset_truth.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/settings/dataset_truth.cpp
)
target_link_libraries(test_dataset_truth PRIVATE Qt6::Test Qt6::Core)
target_include_directories(test_dataset_truth PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}
)
add_test(NAME test_dataset_truth COMMAND test_dataset_truth)
set_tests_properties(test_dataset_truth PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 60)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    set_tests_properties(test_dataset_truth PROPERTIES ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
endif()

# Паспорт набора в пресете (секция [meta]): сверка имён «набор ↔ пресет» —
# код Qt-слоя, собираем исходник напрямую (как test_dataset_truth).
qt_add_executable(test_preset_meta
    test_preset_meta.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/settings/preset_meta.cpp
)
target_link_libraries(test_preset_meta PRIVATE Qt6::Test Qt6::Core)
target_include_directories(test_preset_meta PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/gui
    ${CMAKE_CURRENT_SOURCE_DIR}
)
add_test(NAME test_preset_meta COMMAND test_preset_meta)
set_tests_properties(test_preset_meta PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
    TIMEOUT 60)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
    set_tests_properties(test_preset_meta PROPERTIES ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
endif()

# Паритет консоль vs GUI (этап 6 плана): test_parity.cpp запускает imitator и
# imitator_gui на одних данных с одним явным settings.ini и сверяет бит-в-бит
# result.txt/reference.txt/errors.txt (ими пользуются тюнер и GUI-графики).
# Данные — data/raw; при чистом checkout без imu.dat тест не собирается.
set(ITT_PARITY_DATA_DIR "${CMAKE_SOURCE_DIR}/data/raw")
if(EXISTS "${ITT_PARITY_DATA_DIR}/imu.dat")
    qt_add_executable(test_parity test_parity.cpp)
    target_link_libraries(test_parity PRIVATE Qt6::Test esfcore)
    target_include_directories(test_parity PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_compile_definitions(test_parity PRIVATE
        ITT_IMITATOR_EXE="$<TARGET_FILE:imitator>"
        ITT_IMITATOR_GUI_EXE="$<TARGET_FILE:imitator_gui>"
        ITT_PARITY_DATA_DIR="${ITT_PARITY_DATA_DIR}"
    )
    add_test(NAME test_parity COMMAND test_parity)
    set_tests_properties(test_parity PROPERTIES
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
        TIMEOUT 600)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
        set_tests_properties(test_parity PROPERTIES ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt6::Core>;PATH=path_list_prepend:${ITT_TOOLCHAIN_BIN}")
    endif()
endif()