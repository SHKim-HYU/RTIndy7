# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xeno/Indy_ws/RTIndy7

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xeno/Indy_ws/RTIndy7/build

# Include any dependencies generated for this target.
include include/Robot/CMakeFiles/Robot.dir/depend.make

# Include the progress variables for this target.
include include/Robot/CMakeFiles/Robot.dir/progress.make

# Include the compile flags for this target's objects.
include include/Robot/CMakeFiles/Robot.dir/flags.make

include/Robot/CMakeFiles/Robot.dir/bullet_Indy7.cpp.o: include/Robot/CMakeFiles/Robot.dir/flags.make
include/Robot/CMakeFiles/Robot.dir/bullet_Indy7.cpp.o: ../include/Robot/bullet_Indy7.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xeno/Indy_ws/RTIndy7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object include/Robot/CMakeFiles/Robot.dir/bullet_Indy7.cpp.o"
	cd /home/xeno/Indy_ws/RTIndy7/build/include/Robot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robot.dir/bullet_Indy7.cpp.o -c /home/xeno/Indy_ws/RTIndy7/include/Robot/bullet_Indy7.cpp

include/Robot/CMakeFiles/Robot.dir/bullet_Indy7.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robot.dir/bullet_Indy7.cpp.i"
	cd /home/xeno/Indy_ws/RTIndy7/build/include/Robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xeno/Indy_ws/RTIndy7/include/Robot/bullet_Indy7.cpp > CMakeFiles/Robot.dir/bullet_Indy7.cpp.i

include/Robot/CMakeFiles/Robot.dir/bullet_Indy7.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robot.dir/bullet_Indy7.cpp.s"
	cd /home/xeno/Indy_ws/RTIndy7/build/include/Robot && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xeno/Indy_ws/RTIndy7/include/Robot/bullet_Indy7.cpp -o CMakeFiles/Robot.dir/bullet_Indy7.cpp.s

# Object files for target Robot
Robot_OBJECTS = \
"CMakeFiles/Robot.dir/bullet_Indy7.cpp.o"

# External object files for target Robot
Robot_EXTERNAL_OBJECTS =

include/Robot/libRobot.a: include/Robot/CMakeFiles/Robot.dir/bullet_Indy7.cpp.o
include/Robot/libRobot.a: include/Robot/CMakeFiles/Robot.dir/build.make
include/Robot/libRobot.a: include/Robot/CMakeFiles/Robot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xeno/Indy_ws/RTIndy7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libRobot.a"
	cd /home/xeno/Indy_ws/RTIndy7/build/include/Robot && $(CMAKE_COMMAND) -P CMakeFiles/Robot.dir/cmake_clean_target.cmake
	cd /home/xeno/Indy_ws/RTIndy7/build/include/Robot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Robot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
include/Robot/CMakeFiles/Robot.dir/build: include/Robot/libRobot.a

.PHONY : include/Robot/CMakeFiles/Robot.dir/build

include/Robot/CMakeFiles/Robot.dir/clean:
	cd /home/xeno/Indy_ws/RTIndy7/build/include/Robot && $(CMAKE_COMMAND) -P CMakeFiles/Robot.dir/cmake_clean.cmake
.PHONY : include/Robot/CMakeFiles/Robot.dir/clean

include/Robot/CMakeFiles/Robot.dir/depend:
	cd /home/xeno/Indy_ws/RTIndy7/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xeno/Indy_ws/RTIndy7 /home/xeno/Indy_ws/RTIndy7/include/Robot /home/xeno/Indy_ws/RTIndy7/build /home/xeno/Indy_ws/RTIndy7/build/include/Robot /home/xeno/Indy_ws/RTIndy7/build/include/Robot/CMakeFiles/Robot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : include/Robot/CMakeFiles/Robot.dir/depend

