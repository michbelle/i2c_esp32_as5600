# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

#add_library(my scr/as5600_i2c_esp32.c)

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/test.c ${CMAKE_SOURCE_DIR}/src/as5600_i2c_esp32.c)

idf_component_register(SRCS ${app_sources})
