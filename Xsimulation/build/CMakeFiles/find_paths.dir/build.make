# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/build

# Include any dependencies generated for this target.
include CMakeFiles/find_paths.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/find_paths.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/find_paths.dir/flags.make

CMakeFiles/find_paths.dir/main.cpp.o: CMakeFiles/find_paths.dir/flags.make
CMakeFiles/find_paths.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/find_paths.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/find_paths.dir/main.cpp.o -c /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/main.cpp

CMakeFiles/find_paths.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/find_paths.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/main.cpp > CMakeFiles/find_paths.dir/main.cpp.i

CMakeFiles/find_paths.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/find_paths.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/main.cpp -o CMakeFiles/find_paths.dir/main.cpp.s

# Object files for target find_paths
find_paths_OBJECTS = \
"CMakeFiles/find_paths.dir/main.cpp.o"

# External object files for target find_paths
find_paths_EXTERNAL_OBJECTS =

find_paths: CMakeFiles/find_paths.dir/main.cpp.o
find_paths: CMakeFiles/find_paths.dir/build.make
find_paths: /opt/local/lib/libboost_system-mt.dylib
find_paths: /opt/local/lib/libboost_iostreams-mt.dylib
find_paths: /opt/local/lib/libboost_filesystem-mt.dylib
find_paths: /opt/local/lib/libboost_thread-mt.dylib
find_paths: /opt/local/lib/libboost_timer-mt.dylib
find_paths: /opt/local/lib/libboost_regex-mt.dylib
find_paths: /opt/local/lib/libboost_chrono-mt.dylib
find_paths: /opt/local/lib/libboost_date_time-mt.dylib
find_paths: /opt/local/lib/libboost_atomic-mt.dylib
find_paths: CMakeFiles/find_paths.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable find_paths"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/find_paths.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/find_paths.dir/build: find_paths

.PHONY : CMakeFiles/find_paths.dir/build

CMakeFiles/find_paths.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/find_paths.dir/cmake_clean.cmake
.PHONY : CMakeFiles/find_paths.dir/clean

CMakeFiles/find_paths.dir/depend:
	cd /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/build /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/build /Users/Moti/Documents/GitHub/temp-ridesharing/Xsimulation/build/CMakeFiles/find_paths.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/find_paths.dir/depend
