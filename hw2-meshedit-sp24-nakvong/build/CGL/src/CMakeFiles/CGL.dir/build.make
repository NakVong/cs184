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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.28.3/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.28.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build

# Include any dependencies generated for this target.
include CGL/src/CMakeFiles/CGL.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CGL/src/CMakeFiles/CGL.dir/compiler_depend.make

# Include the progress variables for this target.
include CGL/src/CMakeFiles/CGL.dir/progress.make

# Include the compile flags for this target's objects.
include CGL/src/CMakeFiles/CGL.dir/flags.make

CGL/src/CMakeFiles/CGL.dir/vector2D.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/vector2D.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector2D.cpp
CGL/src/CMakeFiles/CGL.dir/vector2D.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CGL/src/CMakeFiles/CGL.dir/vector2D.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/vector2D.cpp.o -MF CMakeFiles/CGL.dir/vector2D.cpp.o.d -o CMakeFiles/CGL.dir/vector2D.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector2D.cpp

CGL/src/CMakeFiles/CGL.dir/vector2D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/vector2D.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector2D.cpp > CMakeFiles/CGL.dir/vector2D.cpp.i

CGL/src/CMakeFiles/CGL.dir/vector2D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/vector2D.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector2D.cpp -o CMakeFiles/CGL.dir/vector2D.cpp.s

CGL/src/CMakeFiles/CGL.dir/vector3D.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/vector3D.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector3D.cpp
CGL/src/CMakeFiles/CGL.dir/vector3D.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CGL/src/CMakeFiles/CGL.dir/vector3D.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/vector3D.cpp.o -MF CMakeFiles/CGL.dir/vector3D.cpp.o.d -o CMakeFiles/CGL.dir/vector3D.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector3D.cpp

CGL/src/CMakeFiles/CGL.dir/vector3D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/vector3D.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector3D.cpp > CMakeFiles/CGL.dir/vector3D.cpp.i

CGL/src/CMakeFiles/CGL.dir/vector3D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/vector3D.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector3D.cpp -o CMakeFiles/CGL.dir/vector3D.cpp.s

CGL/src/CMakeFiles/CGL.dir/vector4D.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/vector4D.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector4D.cpp
CGL/src/CMakeFiles/CGL.dir/vector4D.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CGL/src/CMakeFiles/CGL.dir/vector4D.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/vector4D.cpp.o -MF CMakeFiles/CGL.dir/vector4D.cpp.o.d -o CMakeFiles/CGL.dir/vector4D.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector4D.cpp

CGL/src/CMakeFiles/CGL.dir/vector4D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/vector4D.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector4D.cpp > CMakeFiles/CGL.dir/vector4D.cpp.i

CGL/src/CMakeFiles/CGL.dir/vector4D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/vector4D.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/vector4D.cpp -o CMakeFiles/CGL.dir/vector4D.cpp.s

CGL/src/CMakeFiles/CGL.dir/matrix3x3.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/matrix3x3.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/matrix3x3.cpp
CGL/src/CMakeFiles/CGL.dir/matrix3x3.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CGL/src/CMakeFiles/CGL.dir/matrix3x3.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/matrix3x3.cpp.o -MF CMakeFiles/CGL.dir/matrix3x3.cpp.o.d -o CMakeFiles/CGL.dir/matrix3x3.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/matrix3x3.cpp

CGL/src/CMakeFiles/CGL.dir/matrix3x3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/matrix3x3.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/matrix3x3.cpp > CMakeFiles/CGL.dir/matrix3x3.cpp.i

CGL/src/CMakeFiles/CGL.dir/matrix3x3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/matrix3x3.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/matrix3x3.cpp -o CMakeFiles/CGL.dir/matrix3x3.cpp.s

CGL/src/CMakeFiles/CGL.dir/matrix4x4.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/matrix4x4.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/matrix4x4.cpp
CGL/src/CMakeFiles/CGL.dir/matrix4x4.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CGL/src/CMakeFiles/CGL.dir/matrix4x4.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/matrix4x4.cpp.o -MF CMakeFiles/CGL.dir/matrix4x4.cpp.o.d -o CMakeFiles/CGL.dir/matrix4x4.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/matrix4x4.cpp

CGL/src/CMakeFiles/CGL.dir/matrix4x4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/matrix4x4.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/matrix4x4.cpp > CMakeFiles/CGL.dir/matrix4x4.cpp.i

CGL/src/CMakeFiles/CGL.dir/matrix4x4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/matrix4x4.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/matrix4x4.cpp -o CMakeFiles/CGL.dir/matrix4x4.cpp.s

CGL/src/CMakeFiles/CGL.dir/quaternion.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/quaternion.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/quaternion.cpp
CGL/src/CMakeFiles/CGL.dir/quaternion.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CGL/src/CMakeFiles/CGL.dir/quaternion.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/quaternion.cpp.o -MF CMakeFiles/CGL.dir/quaternion.cpp.o.d -o CMakeFiles/CGL.dir/quaternion.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/quaternion.cpp

CGL/src/CMakeFiles/CGL.dir/quaternion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/quaternion.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/quaternion.cpp > CMakeFiles/CGL.dir/quaternion.cpp.i

CGL/src/CMakeFiles/CGL.dir/quaternion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/quaternion.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/quaternion.cpp -o CMakeFiles/CGL.dir/quaternion.cpp.s

CGL/src/CMakeFiles/CGL.dir/complex.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/complex.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/complex.cpp
CGL/src/CMakeFiles/CGL.dir/complex.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CGL/src/CMakeFiles/CGL.dir/complex.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/complex.cpp.o -MF CMakeFiles/CGL.dir/complex.cpp.o.d -o CMakeFiles/CGL.dir/complex.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/complex.cpp

CGL/src/CMakeFiles/CGL.dir/complex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/complex.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/complex.cpp > CMakeFiles/CGL.dir/complex.cpp.i

CGL/src/CMakeFiles/CGL.dir/complex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/complex.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/complex.cpp -o CMakeFiles/CGL.dir/complex.cpp.s

CGL/src/CMakeFiles/CGL.dir/color.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/color.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/color.cpp
CGL/src/CMakeFiles/CGL.dir/color.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CGL/src/CMakeFiles/CGL.dir/color.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/color.cpp.o -MF CMakeFiles/CGL.dir/color.cpp.o.d -o CMakeFiles/CGL.dir/color.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/color.cpp

CGL/src/CMakeFiles/CGL.dir/color.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/color.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/color.cpp > CMakeFiles/CGL.dir/color.cpp.i

CGL/src/CMakeFiles/CGL.dir/color.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/color.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/color.cpp -o CMakeFiles/CGL.dir/color.cpp.s

CGL/src/CMakeFiles/CGL.dir/osdtext.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/osdtext.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/osdtext.cpp
CGL/src/CMakeFiles/CGL.dir/osdtext.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CGL/src/CMakeFiles/CGL.dir/osdtext.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/osdtext.cpp.o -MF CMakeFiles/CGL.dir/osdtext.cpp.o.d -o CMakeFiles/CGL.dir/osdtext.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/osdtext.cpp

CGL/src/CMakeFiles/CGL.dir/osdtext.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/osdtext.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/osdtext.cpp > CMakeFiles/CGL.dir/osdtext.cpp.i

CGL/src/CMakeFiles/CGL.dir/osdtext.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/osdtext.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/osdtext.cpp -o CMakeFiles/CGL.dir/osdtext.cpp.s

CGL/src/CMakeFiles/CGL.dir/osdfont.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/osdfont.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/osdfont.cpp
CGL/src/CMakeFiles/CGL.dir/osdfont.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CGL/src/CMakeFiles/CGL.dir/osdfont.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/osdfont.cpp.o -MF CMakeFiles/CGL.dir/osdfont.cpp.o.d -o CMakeFiles/CGL.dir/osdfont.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/osdfont.cpp

CGL/src/CMakeFiles/CGL.dir/osdfont.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/osdfont.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/osdfont.cpp > CMakeFiles/CGL.dir/osdfont.cpp.i

CGL/src/CMakeFiles/CGL.dir/osdfont.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/osdfont.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/osdfont.cpp -o CMakeFiles/CGL.dir/osdfont.cpp.s

CGL/src/CMakeFiles/CGL.dir/viewer.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/viewer.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/viewer.cpp
CGL/src/CMakeFiles/CGL.dir/viewer.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CGL/src/CMakeFiles/CGL.dir/viewer.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/viewer.cpp.o -MF CMakeFiles/CGL.dir/viewer.cpp.o.d -o CMakeFiles/CGL.dir/viewer.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/viewer.cpp

CGL/src/CMakeFiles/CGL.dir/viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/viewer.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/viewer.cpp > CMakeFiles/CGL.dir/viewer.cpp.i

CGL/src/CMakeFiles/CGL.dir/viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/viewer.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/viewer.cpp -o CMakeFiles/CGL.dir/viewer.cpp.s

CGL/src/CMakeFiles/CGL.dir/base64.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/base64.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/base64.cpp
CGL/src/CMakeFiles/CGL.dir/base64.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CGL/src/CMakeFiles/CGL.dir/base64.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/base64.cpp.o -MF CMakeFiles/CGL.dir/base64.cpp.o.d -o CMakeFiles/CGL.dir/base64.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/base64.cpp

CGL/src/CMakeFiles/CGL.dir/base64.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/base64.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/base64.cpp > CMakeFiles/CGL.dir/base64.cpp.i

CGL/src/CMakeFiles/CGL.dir/base64.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/base64.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/base64.cpp -o CMakeFiles/CGL.dir/base64.cpp.s

CGL/src/CMakeFiles/CGL.dir/tinyxml2.cpp.o: CGL/src/CMakeFiles/CGL.dir/flags.make
CGL/src/CMakeFiles/CGL.dir/tinyxml2.cpp.o: /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/tinyxml2.cpp
CGL/src/CMakeFiles/CGL.dir/tinyxml2.cpp.o: CGL/src/CMakeFiles/CGL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CGL/src/CMakeFiles/CGL.dir/tinyxml2.cpp.o"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CGL/src/CMakeFiles/CGL.dir/tinyxml2.cpp.o -MF CMakeFiles/CGL.dir/tinyxml2.cpp.o.d -o CMakeFiles/CGL.dir/tinyxml2.cpp.o -c /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/tinyxml2.cpp

CGL/src/CMakeFiles/CGL.dir/tinyxml2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/CGL.dir/tinyxml2.cpp.i"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/tinyxml2.cpp > CMakeFiles/CGL.dir/tinyxml2.cpp.i

CGL/src/CMakeFiles/CGL.dir/tinyxml2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/CGL.dir/tinyxml2.cpp.s"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src/tinyxml2.cpp -o CMakeFiles/CGL.dir/tinyxml2.cpp.s

# Object files for target CGL
CGL_OBJECTS = \
"CMakeFiles/CGL.dir/vector2D.cpp.o" \
"CMakeFiles/CGL.dir/vector3D.cpp.o" \
"CMakeFiles/CGL.dir/vector4D.cpp.o" \
"CMakeFiles/CGL.dir/matrix3x3.cpp.o" \
"CMakeFiles/CGL.dir/matrix4x4.cpp.o" \
"CMakeFiles/CGL.dir/quaternion.cpp.o" \
"CMakeFiles/CGL.dir/complex.cpp.o" \
"CMakeFiles/CGL.dir/color.cpp.o" \
"CMakeFiles/CGL.dir/osdtext.cpp.o" \
"CMakeFiles/CGL.dir/osdfont.cpp.o" \
"CMakeFiles/CGL.dir/viewer.cpp.o" \
"CMakeFiles/CGL.dir/base64.cpp.o" \
"CMakeFiles/CGL.dir/tinyxml2.cpp.o"

# External object files for target CGL
CGL_EXTERNAL_OBJECTS =

CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/vector2D.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/vector3D.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/vector4D.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/matrix3x3.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/matrix4x4.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/quaternion.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/complex.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/color.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/osdtext.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/osdfont.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/viewer.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/base64.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/tinyxml2.cpp.o
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/build.make
CGL/src/libCGL.a: CGL/src/CMakeFiles/CGL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Linking CXX static library libCGL.a"
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && $(CMAKE_COMMAND) -P CMakeFiles/CGL.dir/cmake_clean_target.cmake
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CGL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CGL/src/CMakeFiles/CGL.dir/build: CGL/src/libCGL.a
.PHONY : CGL/src/CMakeFiles/CGL.dir/build

CGL/src/CMakeFiles/CGL.dir/clean:
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src && $(CMAKE_COMMAND) -P CMakeFiles/CGL.dir/cmake_clean.cmake
.PHONY : CGL/src/CMakeFiles/CGL.dir/clean

CGL/src/CMakeFiles/CGL.dir/depend:
	cd /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/CGL/src /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src /Users/kiriratanakvong/Desktop/cs184/hw2/hw2-meshedit-sp24-nakvong/build/CGL/src/CMakeFiles/CGL.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CGL/src/CMakeFiles/CGL.dir/depend

