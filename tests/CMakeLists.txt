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
    test_settings
    test_simulation
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

# Графики на QPainter живут в Qt-слое, поэтому их тест собирает исходники
# напрямую (esfcore к графикам отношения не имеет).
qt_add_executable(test_plots
    test_plots.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/plot_view.cpp
    ${CMAKE_SOURCE_DIR}/src/gui/views/plots/time_series_plot.cpp
)
target_link_libraries(test_plots PRIVATE Qt6::Test Qt6::Widgets Qt6::Svg esfcore)
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