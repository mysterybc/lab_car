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
CMAKE_COMMAND = /home/lovebc/Desktop/workspace/packs/cmake-3.17.5-Linux-x86_64/bin/cmake

# The command to remove a file.
RM = /home/lovebc/Desktop/workspace/packs/cmake-3.17.5-Linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo

# Include any dependencies generated for this target.
include examples/CMakeFiles/example-Geocentric.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example-Geocentric.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example-Geocentric.dir/flags.make

examples/CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.o: examples/CMakeFiles/example-Geocentric.dir/flags.make
examples/CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.o: /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46/examples/example-Geocentric.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.o"
	cd /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.o -c /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46/examples/example-Geocentric.cpp

examples/CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.i"
	cd /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46/examples/example-Geocentric.cpp > CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.i

examples/CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.s"
	cd /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46/examples/example-Geocentric.cpp -o CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.s

# Object files for target example-Geocentric
example__Geocentric_OBJECTS = \
"CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.o"

# External object files for target example-Geocentric
example__Geocentric_EXTERNAL_OBJECTS =

examples/example-Geocentric: examples/CMakeFiles/example-Geocentric.dir/example-Geocentric.cpp.o
examples/example-Geocentric: examples/CMakeFiles/example-Geocentric.dir/build.make
examples/example-Geocentric: src/libGeographic.so.17.0.0
examples/example-Geocentric: examples/CMakeFiles/example-Geocentric.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example-Geocentric"
	cd /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-Geocentric.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example-Geocentric.dir/build: examples/example-Geocentric

.PHONY : examples/CMakeFiles/example-Geocentric.dir/build

examples/CMakeFiles/example-Geocentric.dir/clean:
	cd /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/examples && $(CMAKE_COMMAND) -P CMakeFiles/example-Geocentric.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example-Geocentric.dir/clean

examples/CMakeFiles/example-Geocentric.dir/depend:
	cd /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46 /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46/examples /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/examples /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/examples/CMakeFiles/example-Geocentric.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example-Geocentric.dir/depend

