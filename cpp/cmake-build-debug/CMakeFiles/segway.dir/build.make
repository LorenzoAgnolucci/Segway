# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/giova/PycharmProjects/segway/cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/giova/PycharmProjects/segway/cpp/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/segway.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/segway.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/segway.dir/flags.make

CMakeFiles/segway.dir/main.cpp.o: CMakeFiles/segway.dir/flags.make
CMakeFiles/segway.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giova/PycharmProjects/segway/cpp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/segway.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segway.dir/main.cpp.o -c /home/giova/PycharmProjects/segway/cpp/main.cpp

CMakeFiles/segway.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segway.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giova/PycharmProjects/segway/cpp/main.cpp > CMakeFiles/segway.dir/main.cpp.i

CMakeFiles/segway.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segway.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giova/PycharmProjects/segway/cpp/main.cpp -o CMakeFiles/segway.dir/main.cpp.s

CMakeFiles/segway.dir/ev3dev.cpp.o: CMakeFiles/segway.dir/flags.make
CMakeFiles/segway.dir/ev3dev.cpp.o: ../ev3dev.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giova/PycharmProjects/segway/cpp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/segway.dir/ev3dev.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segway.dir/ev3dev.cpp.o -c /home/giova/PycharmProjects/segway/cpp/ev3dev.cpp

CMakeFiles/segway.dir/ev3dev.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segway.dir/ev3dev.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giova/PycharmProjects/segway/cpp/ev3dev.cpp > CMakeFiles/segway.dir/ev3dev.cpp.i

CMakeFiles/segway.dir/ev3dev.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segway.dir/ev3dev.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giova/PycharmProjects/segway/cpp/ev3dev.cpp -o CMakeFiles/segway.dir/ev3dev.cpp.s

CMakeFiles/segway.dir/sensors.cpp.o: CMakeFiles/segway.dir/flags.make
CMakeFiles/segway.dir/sensors.cpp.o: ../sensors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giova/PycharmProjects/segway/cpp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/segway.dir/sensors.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segway.dir/sensors.cpp.o -c /home/giova/PycharmProjects/segway/cpp/sensors.cpp

CMakeFiles/segway.dir/sensors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segway.dir/sensors.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giova/PycharmProjects/segway/cpp/sensors.cpp > CMakeFiles/segway.dir/sensors.cpp.i

CMakeFiles/segway.dir/sensors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segway.dir/sensors.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giova/PycharmProjects/segway/cpp/sensors.cpp -o CMakeFiles/segway.dir/sensors.cpp.s

CMakeFiles/segway.dir/utils.cpp.o: CMakeFiles/segway.dir/flags.make
CMakeFiles/segway.dir/utils.cpp.o: ../utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giova/PycharmProjects/segway/cpp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/segway.dir/utils.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segway.dir/utils.cpp.o -c /home/giova/PycharmProjects/segway/cpp/utils.cpp

CMakeFiles/segway.dir/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segway.dir/utils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giova/PycharmProjects/segway/cpp/utils.cpp > CMakeFiles/segway.dir/utils.cpp.i

CMakeFiles/segway.dir/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segway.dir/utils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giova/PycharmProjects/segway/cpp/utils.cpp -o CMakeFiles/segway.dir/utils.cpp.s

CMakeFiles/segway.dir/model_parameters.cpp.o: CMakeFiles/segway.dir/flags.make
CMakeFiles/segway.dir/model_parameters.cpp.o: ../model_parameters.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/giova/PycharmProjects/segway/cpp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/segway.dir/model_parameters.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/segway.dir/model_parameters.cpp.o -c /home/giova/PycharmProjects/segway/cpp/model_parameters.cpp

CMakeFiles/segway.dir/model_parameters.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/segway.dir/model_parameters.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/giova/PycharmProjects/segway/cpp/model_parameters.cpp > CMakeFiles/segway.dir/model_parameters.cpp.i

CMakeFiles/segway.dir/model_parameters.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/segway.dir/model_parameters.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/giova/PycharmProjects/segway/cpp/model_parameters.cpp -o CMakeFiles/segway.dir/model_parameters.cpp.s

# Object files for target segway
segway_OBJECTS = \
"CMakeFiles/segway.dir/main.cpp.o" \
"CMakeFiles/segway.dir/ev3dev.cpp.o" \
"CMakeFiles/segway.dir/sensors.cpp.o" \
"CMakeFiles/segway.dir/utils.cpp.o" \
"CMakeFiles/segway.dir/model_parameters.cpp.o"

# External object files for target segway
segway_EXTERNAL_OBJECTS =

segway: CMakeFiles/segway.dir/main.cpp.o
segway: CMakeFiles/segway.dir/ev3dev.cpp.o
segway: CMakeFiles/segway.dir/sensors.cpp.o
segway: CMakeFiles/segway.dir/utils.cpp.o
segway: CMakeFiles/segway.dir/model_parameters.cpp.o
segway: CMakeFiles/segway.dir/build.make
segway: CMakeFiles/segway.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/giova/PycharmProjects/segway/cpp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable segway"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/segway.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/segway.dir/build: segway

.PHONY : CMakeFiles/segway.dir/build

CMakeFiles/segway.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/segway.dir/cmake_clean.cmake
.PHONY : CMakeFiles/segway.dir/clean

CMakeFiles/segway.dir/depend:
	cd /home/giova/PycharmProjects/segway/cpp/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/giova/PycharmProjects/segway/cpp /home/giova/PycharmProjects/segway/cpp /home/giova/PycharmProjects/segway/cpp/cmake-build-debug /home/giova/PycharmProjects/segway/cpp/cmake-build-debug /home/giova/PycharmProjects/segway/cpp/cmake-build-debug/CMakeFiles/segway.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/segway.dir/depend

