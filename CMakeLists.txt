cmake_minimum_required(VERSION 3.16)

project(ITT_Students LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Консольный проект БИНС/СНС с фильтром Калмана 15-го порядка.
# Это версия без Qt: только штатный запуск `imitator <каталог_данных> [params.ini]`.
# Ядро (Qt-независимый код) лежит плоско в src/: data_io, ins, math_lib,
# navigation, utils. Ни одного включения Qt: сборка чистым C++17 (STL).
#
# Движок прогона (открытие файлов, выставка, основной цикл, запись) целиком
# в src/main.cpp — как в исходной консольной версии.
#
# Исходники:
#   src/main.cpp                 — движок прогона: аргументы, выставка, цикл, запись
#   src/navigation/...           — выставка, интегрирование, комплекс. по СНС
#   src/data_io/...              — чтение imu/gps/angle, запись результатов
#   src/utils/...                — константы, типы, пути
add_executable(imitator
    src/main.cpp
    src/data_io/data_reader.cpp
    src/data_io/data_writer.cpp
    src/navigation/Aligner.cpp
    src/navigation/averaging.cpp
    src/navigation/bias_log.cpp
    src/navigation/bias_report.cpp
)

# Заголовки подключаются от корня src (например, "navigation/trajectory.h").
target_include_directories(imitator PRIVATE src)

if(MSVC)
    target_compile_options(imitator PRIVATE /utf-8)
endif()
