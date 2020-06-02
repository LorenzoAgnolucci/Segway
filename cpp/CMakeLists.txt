cmake_minimum_required(VERSION 3.7)
project(segway)

add_executable(segway main.cpp ev3dev.h ev3dev.cpp sensors.h sensors.cpp utils.h utils.cpp resource_guard.h model_parameters.h model_parameters.cpp)
target_compile_options(segway PRIVATE -Wall -Wextra -O3 -rdynamic -g)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(segway Threads::Threads)
