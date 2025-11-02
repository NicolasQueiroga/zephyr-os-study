cmake_minimum_required(VERSION 3.20)
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/custom_pins.overlay)

# Add micro-ROS module
list(APPEND ZEPHYR_EXTRA_MODULES $ENV{ZEPHYR_BASE}/../modules/micro_ros_zephyr_module/modules/libmicroros)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(embarcados-zephyr)

# Get micro-ROS package list and add include directories
set(MICROROS_DIR $ENV{ZEPHYR_BASE}/../modules/micro_ros_zephyr_module/modules/libmicroros)
execute_process(
    COMMAND make --no-print-directory -f libmicroros.mk get_package_names COMPONENT_PATH=${MICROROS_DIR}
    WORKING_DIRECTORY ${MICROROS_DIR}
    OUTPUT_VARIABLE MICROROS_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
# Add each package include directory
foreach(pkg ${MICROROS_PACKAGES})
    include_directories(${MICROROS_DIR}/include/${pkg})
endforeach()

# Compiler flags for C code
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic")

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

add_library(uros ${CMAKE_CURRENT_SOURCE_DIR}/src/uros/uros.c)
target_include_directories(uros PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/uros
    ${ZEPHYR_BASE}/include
)
# Link with microros - it provides include directories via INTERFACE
target_link_libraries(uros PUBLIC microros)
target_link_libraries(uros PRIVATE zephyr)
add_dependencies(uros microros)

target_link_libraries(app PRIVATE utils distance uros)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c)
