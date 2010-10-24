# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /app/bullet/bullet-2.77

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /app/bullet/bullet-2.77

# Include any dependencies generated for this target.
include src/LinearMath/CMakeFiles/LinearMath.dir/depend.make

# Include the progress variables for this target.
include src/LinearMath/CMakeFiles/LinearMath.dir/progress.make

# Include the compile flags for this target's objects.
include src/LinearMath/CMakeFiles/LinearMath.dir/flags.make

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o: src/LinearMath/btAlignedAllocator.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /app/bullet/bullet-2.77/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LinearMath.dir/btAlignedAllocator.o -c /app/bullet/bullet-2.77/src/LinearMath/btAlignedAllocator.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btAlignedAllocator.i"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /app/bullet/bullet-2.77/src/LinearMath/btAlignedAllocator.cpp > CMakeFiles/LinearMath.dir/btAlignedAllocator.i

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btAlignedAllocator.s"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /app/bullet/bullet-2.77/src/LinearMath/btAlignedAllocator.cpp -o CMakeFiles/LinearMath.dir/btAlignedAllocator.s

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.requires:
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.requires

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.provides: src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.requires
	$(MAKE) -f src/LinearMath/CMakeFiles/LinearMath.dir/build.make src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.provides.build
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.provides

src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.provides.build: src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.provides.build

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o: src/LinearMath/btConvexHull.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /app/bullet/bullet-2.77/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LinearMath.dir/btConvexHull.o -c /app/bullet/bullet-2.77/src/LinearMath/btConvexHull.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btConvexHull.i"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /app/bullet/bullet-2.77/src/LinearMath/btConvexHull.cpp > CMakeFiles/LinearMath.dir/btConvexHull.i

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btConvexHull.s"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /app/bullet/bullet-2.77/src/LinearMath/btConvexHull.cpp -o CMakeFiles/LinearMath.dir/btConvexHull.s

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.requires:
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.requires

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.provides: src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.requires
	$(MAKE) -f src/LinearMath/CMakeFiles/LinearMath.dir/build.make src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.provides.build
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.provides

src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.provides.build: src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.provides.build

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o: src/LinearMath/btGeometryUtil.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /app/bullet/bullet-2.77/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LinearMath.dir/btGeometryUtil.o -c /app/bullet/bullet-2.77/src/LinearMath/btGeometryUtil.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btGeometryUtil.i"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /app/bullet/bullet-2.77/src/LinearMath/btGeometryUtil.cpp > CMakeFiles/LinearMath.dir/btGeometryUtil.i

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btGeometryUtil.s"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /app/bullet/bullet-2.77/src/LinearMath/btGeometryUtil.cpp -o CMakeFiles/LinearMath.dir/btGeometryUtil.s

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.requires:
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.requires

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.provides: src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.requires
	$(MAKE) -f src/LinearMath/CMakeFiles/LinearMath.dir/build.make src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.provides.build
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.provides

src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.provides.build: src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.provides.build

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o: src/LinearMath/btQuickprof.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /app/bullet/bullet-2.77/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LinearMath.dir/btQuickprof.o -c /app/bullet/bullet-2.77/src/LinearMath/btQuickprof.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btQuickprof.i"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /app/bullet/bullet-2.77/src/LinearMath/btQuickprof.cpp > CMakeFiles/LinearMath.dir/btQuickprof.i

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btQuickprof.s"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /app/bullet/bullet-2.77/src/LinearMath/btQuickprof.cpp -o CMakeFiles/LinearMath.dir/btQuickprof.s

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.requires:
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.requires

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.provides: src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.requires
	$(MAKE) -f src/LinearMath/CMakeFiles/LinearMath.dir/build.make src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.provides.build
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.provides

src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.provides.build: src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.provides.build

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o: src/LinearMath/CMakeFiles/LinearMath.dir/flags.make
src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o: src/LinearMath/btSerializer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /app/bullet/bullet-2.77/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LinearMath.dir/btSerializer.o -c /app/bullet/bullet-2.77/src/LinearMath/btSerializer.cpp

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LinearMath.dir/btSerializer.i"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /app/bullet/bullet-2.77/src/LinearMath/btSerializer.cpp > CMakeFiles/LinearMath.dir/btSerializer.i

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LinearMath.dir/btSerializer.s"
	cd /app/bullet/bullet-2.77/src/LinearMath && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /app/bullet/bullet-2.77/src/LinearMath/btSerializer.cpp -o CMakeFiles/LinearMath.dir/btSerializer.s

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.requires:
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.requires

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.provides: src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.requires
	$(MAKE) -f src/LinearMath/CMakeFiles/LinearMath.dir/build.make src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.provides.build
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.provides

src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.provides.build: src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.provides.build

# Object files for target LinearMath
LinearMath_OBJECTS = \
"CMakeFiles/LinearMath.dir/btAlignedAllocator.o" \
"CMakeFiles/LinearMath.dir/btConvexHull.o" \
"CMakeFiles/LinearMath.dir/btGeometryUtil.o" \
"CMakeFiles/LinearMath.dir/btQuickprof.o" \
"CMakeFiles/LinearMath.dir/btSerializer.o"

# External object files for target LinearMath
LinearMath_EXTERNAL_OBJECTS =

src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/build.make
src/LinearMath/libLinearMath.a: src/LinearMath/CMakeFiles/LinearMath.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libLinearMath.a"
	cd /app/bullet/bullet-2.77/src/LinearMath && $(CMAKE_COMMAND) -P CMakeFiles/LinearMath.dir/cmake_clean_target.cmake
	cd /app/bullet/bullet-2.77/src/LinearMath && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LinearMath.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/LinearMath/CMakeFiles/LinearMath.dir/build: src/LinearMath/libLinearMath.a
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/build

src/LinearMath/CMakeFiles/LinearMath.dir/requires: src/LinearMath/CMakeFiles/LinearMath.dir/btAlignedAllocator.o.requires
src/LinearMath/CMakeFiles/LinearMath.dir/requires: src/LinearMath/CMakeFiles/LinearMath.dir/btConvexHull.o.requires
src/LinearMath/CMakeFiles/LinearMath.dir/requires: src/LinearMath/CMakeFiles/LinearMath.dir/btGeometryUtil.o.requires
src/LinearMath/CMakeFiles/LinearMath.dir/requires: src/LinearMath/CMakeFiles/LinearMath.dir/btQuickprof.o.requires
src/LinearMath/CMakeFiles/LinearMath.dir/requires: src/LinearMath/CMakeFiles/LinearMath.dir/btSerializer.o.requires
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/requires

src/LinearMath/CMakeFiles/LinearMath.dir/clean:
	cd /app/bullet/bullet-2.77/src/LinearMath && $(CMAKE_COMMAND) -P CMakeFiles/LinearMath.dir/cmake_clean.cmake
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/clean

src/LinearMath/CMakeFiles/LinearMath.dir/depend:
	cd /app/bullet/bullet-2.77 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /app/bullet/bullet-2.77 /app/bullet/bullet-2.77/src/LinearMath /app/bullet/bullet-2.77 /app/bullet/bullet-2.77/src/LinearMath /app/bullet/bullet-2.77/src/LinearMath/CMakeFiles/LinearMath.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/LinearMath/CMakeFiles/LinearMath.dir/depend

