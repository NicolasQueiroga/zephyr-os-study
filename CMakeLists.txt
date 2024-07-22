cmake_minimum_required(VERSION 3.13.1)
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/custom_pins.overlay)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarcados-zephyr)

set(GCC_COVERAGE_COMPILE_FLAGS "-g -Og -Wall -pedantic -O3")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(distance ${CMAKE_CURRENT_SOURCE_DIR}/src/distance/distance.c)
target_include_directories(distance PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/distance
    ${ZEPHYR_BASE}/include
)
target_link_libraries(distance PRIVATE zephyr)

add_library(utils ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/utils.c)
target_include_directories(utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src/distance
    ${ZEPHYR_BASE}/include
)
target_link_libraries(utils PRIVATE zephyr distance)



target_link_libraries(app PRIVATE utils distance)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)
