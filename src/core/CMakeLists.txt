# esfcore — Qt-независимое ядро БИНС/СНС комплексирования.
#
# Собирается как статическая библиотека C++17 (STL, без Qt).
# Правило проекта: в этом каталоге не должно быть ни одного
# `#include <Q...>` — проверяется скриптом tools/check_no_qt_in_core.ps1.

add_library(esfcore STATIC
    data_io/data_reader.cpp
    data_io/data_writer.cpp
    navigation/Aligner.cpp
    navigation/filter_settings.cpp
    navigation/simulation.cpp
    utils/ini_parser.cpp
)

target_include_directories(esfcore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_compile_features(esfcore PUBLIC cxx_std_17)

set_target_properties(esfcore PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

if(MSVC)
    target_compile_options(esfcore PRIVATE /utf-8)
endif()