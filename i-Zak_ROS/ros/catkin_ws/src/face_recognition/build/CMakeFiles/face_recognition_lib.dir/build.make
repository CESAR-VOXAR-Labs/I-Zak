# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/gabriel/catkin_ws/src/procrob_functional-catkin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gabriel/catkin_ws/src/procrob_functional-catkin/build

# Include any dependencies generated for this target.
include CMakeFiles/face_recognition_lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/face_recognition_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/face_recognition_lib.dir/flags.make

CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o: CMakeFiles/face_recognition_lib.dir/flags.make
CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o: ../src/face_recognition_lib.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/gabriel/catkin_ws/src/procrob_functional-catkin/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o -c /home/gabriel/catkin_ws/src/procrob_functional-catkin/src/face_recognition_lib.cpp

CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/gabriel/catkin_ws/src/procrob_functional-catkin/src/face_recognition_lib.cpp > CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.i

CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/gabriel/catkin_ws/src/procrob_functional-catkin/src/face_recognition_lib.cpp -o CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.s

CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o.requires:
.PHONY : CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o.requires

CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o.provides: CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o.requires
	$(MAKE) -f CMakeFiles/face_recognition_lib.dir/build.make CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o.provides.build
.PHONY : CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o.provides

CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o.provides.build: CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o

# Object files for target face_recognition_lib
face_recognition_lib_OBJECTS = \
"CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o"

# External object files for target face_recognition_lib
face_recognition_lib_EXTERNAL_OBJECTS =

devel/lib/libface_recognition_lib.so: CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o
devel/lib/libface_recognition_lib.so: CMakeFiles/face_recognition_lib.dir/build.make
devel/lib/libface_recognition_lib.so: CMakeFiles/face_recognition_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library devel/lib/libface_recognition_lib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/face_recognition_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/face_recognition_lib.dir/build: devel/lib/libface_recognition_lib.so
.PHONY : CMakeFiles/face_recognition_lib.dir/build

CMakeFiles/face_recognition_lib.dir/requires: CMakeFiles/face_recognition_lib.dir/src/face_recognition_lib.cpp.o.requires
.PHONY : CMakeFiles/face_recognition_lib.dir/requires

CMakeFiles/face_recognition_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/face_recognition_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/face_recognition_lib.dir/clean

CMakeFiles/face_recognition_lib.dir/depend:
	cd /home/gabriel/catkin_ws/src/procrob_functional-catkin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gabriel/catkin_ws/src/procrob_functional-catkin /home/gabriel/catkin_ws/src/procrob_functional-catkin /home/gabriel/catkin_ws/src/procrob_functional-catkin/build /home/gabriel/catkin_ws/src/procrob_functional-catkin/build /home/gabriel/catkin_ws/src/procrob_functional-catkin/build/CMakeFiles/face_recognition_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/face_recognition_lib.dir/depend

