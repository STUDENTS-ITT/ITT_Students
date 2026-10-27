cmake_minimum_required(VERSION 3.16)
project(vo_cpp VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(VO_BUILD_APPS  "Собирать vo_final и run_all_hkairport03" ON)
option(VO_BUILD_TESTS "Собирать smoke_test (синтетическая проверка)" ON)
option(VO_WARNINGS    "Включить расширенные предупреждения компилятора" ON)

# OpenCV 4.7+ (проверено на 5.0). Модули берутся все, что есть в сборке: в 5.x
# часть модулей переименована (features2d -> features, calib3d -> 3d/calib),
# поэтому список компонентов жёстко не задаётся.
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV ${OpenCV_VERSION} (${OpenCV_DIR})")

# FREAK, SURF, (в 5.x также BRISK/KAZE/AKAZE) лежат в opencv_contrib.
list(FIND OpenCV_LIBS opencv_xfeatures2d _vo_xf_idx)
if(_vo_xf_idx GREATER -1)
  set(VO_HAVE_XFEATURES2D 1)
  message(STATUS "opencv_xfeatures2d найден: FREAK/SURF доступны")
else()
  set(VO_HAVE_XFEATURES2D 0)
  message(WARNING "opencv_xfeatures2d не найден: связки с FREAK/SURF недоступны. "
                  "Нужна сборка OpenCV с opencv_contrib.")
endif()

set(VO_SOURCES
  src/config.cpp
  src/util.cpp
  src/calibration.cpp
  src/sequence.cpp
  src/altitude.cpp
  src/preprocess.cpp
  src/features.cpp
  src/matching.cpp
  src/motion.cpp
  src/selection.cpp
  src/tracking.cpp
  src/viewer.cpp
  src/plot.cpp
  src/pipeline.cpp
  src/body_yaw.cpp
  tools/export_for_ins.cpp
)

add_library(vo_core STATIC ${VO_SOURCES})
target_include_directories(vo_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vo_core PUBLIC ${OpenCV_LIBS})
target_compile_definitions(vo_core PUBLIC VO_HAVE_XFEATURES2D=${VO_HAVE_XFEATURES2D})

if(VO_WARNINGS)
  if(MSVC)
    target_compile_options(vo_core PRIVATE /W3 /utf-8)
  else()
    target_compile_options(vo_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
  endif()
endif()
if(MINGW)
  # ISO-совместимый printf (%zu и т.п.) в MinGW-w64
  target_compile_definitions(vo_core PUBLIC __USE_MINGW_ANSI_STDIO=1)
endif()
if(MSVC)
  target_compile_options(vo_core PUBLIC /utf-8)
  target_compile_definitions(vo_core PUBLIC _USE_MATH_DEFINES NOMINMAX)
endif()

if(VO_BUILD_APPS)
  add_executable(vo_final apps/vo_final.cpp)
  target_link_libraries(vo_final PRIVATE vo_core)

  add_executable(run_all_hkairport03 apps/run_all_hkairport03.cpp)
  target_link_libraries(run_all_hkairport03 PRIVATE vo_core)

  install(TARGETS vo_final run_all_hkairport03 RUNTIME DESTINATION bin)
endif()

