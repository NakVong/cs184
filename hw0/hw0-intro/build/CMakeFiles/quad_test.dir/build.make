# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.28.1/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.28.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/build

# Include any dependencies generated for this target.
include CMakeFiles/quad_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/quad_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/quad_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quad_test.dir/flags.make

CMakeFiles/quad_test.dir/quad_test.cpp.o: CMakeFiles/quad_test.dir/flags.make
CMakeFiles/quad_test.dir/quad_test.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/quad_test.cpp
CMakeFiles/quad_test.dir/quad_test.cpp.o: CMakeFiles/quad_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/quad_test.dir/quad_test.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/quad_test.dir/quad_test.cpp.o -MF CMakeFiles/quad_test.dir/quad_test.cpp.o.d -o CMakeFiles/quad_test.dir/quad_test.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/quad_test.cpp

CMakeFiles/quad_test.dir/quad_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/quad_test.dir/quad_test.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/quad_test.cpp > CMakeFiles/quad_test.dir/quad_test.cpp.i

CMakeFiles/quad_test.dir/quad_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/quad_test.dir/quad_test.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/quad_test.cpp -o CMakeFiles/quad_test.dir/quad_test.cpp.s

# Object files for target quad_test
quad_test_OBJECTS = \
"CMakeFiles/quad_test.dir/quad_test.cpp.o"

# External object files for target quad_test
quad_test_EXTERNAL_OBJECTS =

quad_test: CMakeFiles/quad_test.dir/quad_test.cpp.o
quad_test: CMakeFiles/quad_test.dir/build.make
quad_test: CGL/src/libCGL_osx.a
quad_test: CGL/deps/glew/libglew.a
quad_test: CGL/deps/glfw/src/libglfw3.a
quad_test: /opt/homebrew/lib/libfreetype.dylib
quad_test: CMakeFiles/quad_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable quad_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quad_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quad_test.dir/build: quad_test
.PHONY : CMakeFiles/quad_test.dir/build

CMakeFiles/quad_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quad_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quad_test.dir/clean

CMakeFiles/quad_test.dir/depend:
	cd /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/build /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/build /Users/kiriratanakvong/Desktop/cs184/hw0/hw0-intro/build/CMakeFiles/quad_test.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/quad_test.dir/depend
