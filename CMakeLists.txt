cmake_minimum_required(VERSION 3.15)

project(ITT_Students LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Исполнимый файл: БИНС с фильтром Калмана 15-го порядка.
# Исходники:
#   main.cpp              — точка входа, цикл счисления
#   Aligner.cpp           — автономная выставка (Median + EMA фильтры)
#   data_reader.cpp       — чтение imu.dat, gps.dat, angle.dat
#   data_writer.cpp       — запись траектории и ошибок фильтра
add_executable(imitator
    src/main.cpp
    src/navigation/Aligner.cpp
    src/data_io/data_reader.cpp
    src/data_io/data_writer.cpp
)

target_include_directories(imitator PRIVATE src)

if(MSVC)
    target_compile_options(imitator PRIVATE /utf-8)
endif()
