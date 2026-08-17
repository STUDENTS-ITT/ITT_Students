cmake_minimum_required(VERSION 3.12)
project(ITT_STUDENTS CXX)

# Cтандарт C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Библиотека навигации
add_library(navigation_lib 
		src/navigation/aligner.cpp
		src/navigation/aligner.hpp
)

# Указываем путь, где компилятор должен искать заголовочные файлы
target_include_directories(navigation_lib PUBLIC 
		${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Создаем исполняемый файл
add_executable(main 
		src/main.cpp
)

# Подключаем всё необходимое к исполняемому файлу
target_link_libraries(main PRIVATE navigation_lib)