# check_no_qt_in_core.cmake — Проверка, что в ядро (src/core) не протек Qt.
#
# Ядро собирается как статическая библиотека без Qt. Единственный допустимый
# Qt-зависимый заголовок в каталоге core/constraints отсутствует — здесь просто
# ищем любые #include <Q...> / "Q...".
#
# Запуск:
#   cmake -DITT_CORE_DIR=<путь к src/core> -P tools/check_no_qt_in_core.cmake

if(NOT DEFINED ITT_CORE_DIR)
    message(FATAL_ERROR "ITT_CORE_DIR не задан")
endif()

file(GLOB_RECURSE core_files
    "${ITT_CORE_DIR}/*.h"
    "${ITT_CORE_DIR}/*.hpp"
    "${ITT_CORE_DIR}/*.cpp"
    "${ITT_CORE_DIR}/*.cxx"
)

set(bad "")
foreach(f IN LISTS core_files)
    file(STRINGS "${f}" lines REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"](Q[A-Za-z0-9_/]*)[>\"]")
    if(lines)
        list(APPEND bad "${f}: ${lines}")
    endif()
    list(LENGTH lines n)
    if(n GREATER 0)
        message(SEND_ERROR "Qt в ядре: ${f}")
    endif()
endforeach()

if(bad)
    foreach(b IN LISTS bad)
        message(SEND_ERROR "${b}")
    endforeach()
    message(FATAL_ERROR "В src/core найдены Qt-заголовки. Ядро должно оставаться Qt-независимым.")
endif()

list(LENGTH core_files total)
message(STATUS "Проверено файлов в src/core: ${total} — Qt не найден.")