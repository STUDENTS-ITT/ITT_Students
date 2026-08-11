cmake_minimum_required(VERSION 3.15)

project(ITT_Students LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(imitator
    src/main.cpp
    src/data_io/data_reader.cpp
    src/data_io/data_writer.cpp
)

target_include_directories(imitator PRIVATE src)

if(MSVC)
    target_compile_options(imitator PRIVATE /utf-8)
endif()
