# imitator — консольный запуск (обратная совместимость).
#
# Использование:
#   imitator [каталог_данных] [settings.ini]
#   каталог_данных по умолчанию — ../data/raw
#   result.txt/reference.txt/errors.txt пишутся в каталог tools/
#   рядом с исполняемым файлом (см. utils::toolsDir).

add_executable(imitator main.cpp)

target_link_libraries(imitator PRIVATE esfcore)

target_include_directories(imitator PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_compile_features(imitator PRIVATE cxx_std_17)

if(MSVC)
    target_compile_options(imitator PRIVATE /utf-8)
endif()