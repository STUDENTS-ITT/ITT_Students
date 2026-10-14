cmake_minimum_required(VERSION 3.16)

project(ITT_Students LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Почтовый каталог компилятора: собственные разделяемые библиотеки (libstdc++-6)
# должны подставляться в PATH тестов РАНЬШЕ тех, что приходят вместе с Qt
# (Qt 6.12.0 MinGW собран GCC 15.1.0, смешение ABI роняет std::iostream).
get_filename_component(ITT_TOOLCHAIN_BIN "${CMAKE_CXX_COMPILER}" DIRECTORY)

# Qt 6, только LGPL-модули (Core, Gui, Widgets, Concurrent, Sql, PrintSupport,
# Svg, Test). Находим один раз: используют imitator_gui и тесты.
find_package(Qt6 REQUIRED COMPONENTS
    Core Gui Widgets Concurrent Sql PrintSupport Svg Test)

# BUILD_TESTING (option, по умолчанию ON) и регистрация тестов через ctest.
include(CTest)

add_subdirectory(src/core)   # esfcore — ядро без Qt
add_subdirectory(src/console) # imitator — консольный запуск (обратная совместимость)
add_subdirectory(src/gui)    # imitator_gui + gui_smoke
add_subdirectory(tests)      # модульные тесты QtTest

# Правило проекта: в src/core не должно быть ни одного #include <Q...>.
# Проверяется отдельным ctest-тестом (ядро должно оставаться Qt-независимым).
if(BUILD_TESTING)
    add_test(NAME core_has_no_qt
        COMMAND ${CMAKE_COMMAND}
            -DITT_CORE_DIR=${CMAKE_SOURCE_DIR}/src/core
            -P ${CMAKE_SOURCE_DIR}/tools/check_no_qt_in_core.cmake)
endif()