cmake_minimum_required(VERSION 3.0)
project(segway)

add_executable(segway main.cpp ev3dev.h ev3dev.cpp sensors.h sensors.cpp utils.h utils.cpp resource_guard.h)
target_compile_options(segway PRIVATE -Wall -Wextra -rdynamic -pg)
