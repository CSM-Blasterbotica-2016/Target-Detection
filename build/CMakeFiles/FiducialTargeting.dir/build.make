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
CMAKE_SOURCE_DIR = /home/blasterbotica/opencv

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/blasterbotica/opencv/build

# Include any dependencies generated for this target.
include CMakeFiles/FiducialTargeting.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FiducialTargeting.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FiducialTargeting.dir/flags.make

CMakeFiles/FiducialTargeting.dir/main.cpp.o: CMakeFiles/FiducialTargeting.dir/flags.make
CMakeFiles/FiducialTargeting.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/blasterbotica/opencv/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/FiducialTargeting.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/FiducialTargeting.dir/main.cpp.o -c /home/blasterbotica/opencv/main.cpp

CMakeFiles/FiducialTargeting.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FiducialTargeting.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/blasterbotica/opencv/main.cpp > CMakeFiles/FiducialTargeting.dir/main.cpp.i

CMakeFiles/FiducialTargeting.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FiducialTargeting.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/blasterbotica/opencv/main.cpp -o CMakeFiles/FiducialTargeting.dir/main.cpp.s

CMakeFiles/FiducialTargeting.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/FiducialTargeting.dir/main.cpp.o.requires

CMakeFiles/FiducialTargeting.dir/main.cpp.o.provides: CMakeFiles/FiducialTargeting.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/FiducialTargeting.dir/build.make CMakeFiles/FiducialTargeting.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/FiducialTargeting.dir/main.cpp.o.provides

CMakeFiles/FiducialTargeting.dir/main.cpp.o.provides.build: CMakeFiles/FiducialTargeting.dir/main.cpp.o

# Object files for target FiducialTargeting
FiducialTargeting_OBJECTS = \
"CMakeFiles/FiducialTargeting.dir/main.cpp.o"

# External object files for target FiducialTargeting
FiducialTargeting_EXTERNAL_OBJECTS =

FiducialTargeting: CMakeFiles/FiducialTargeting.dir/main.cpp.o
FiducialTargeting: CMakeFiles/FiducialTargeting.dir/build.make
FiducialTargeting: /usr/local/lib/libopencv_xphoto.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_xobjdetect.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_ximgproc.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_xfeatures2d.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_tracking.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_text.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_surface_matching.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_structured_light.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_stereo.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_saliency.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_rgbd.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_reg.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_plot.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_optflow.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_line_descriptor.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_fuzzy.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_face.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_dpm.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_dnn.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_datasets.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_ccalib.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_bioinspired.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_bgsegm.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_aruco.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_viz.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_videostab.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_videoio.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_video.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_superres.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_stitching.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_shape.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_photo.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_objdetect.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_ml.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_imgproc.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_highgui.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_flann.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_features2d.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_core.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_calib3d.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_text.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_face.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_ximgproc.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_xfeatures2d.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_shape.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_video.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_objdetect.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_calib3d.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_features2d.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_ml.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_highgui.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_videoio.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_imgproc.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_flann.so.3.1.0
FiducialTargeting: /usr/local/lib/libopencv_core.so.3.1.0
FiducialTargeting: CMakeFiles/FiducialTargeting.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable FiducialTargeting"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FiducialTargeting.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FiducialTargeting.dir/build: FiducialTargeting
.PHONY : CMakeFiles/FiducialTargeting.dir/build

CMakeFiles/FiducialTargeting.dir/requires: CMakeFiles/FiducialTargeting.dir/main.cpp.o.requires
.PHONY : CMakeFiles/FiducialTargeting.dir/requires

CMakeFiles/FiducialTargeting.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FiducialTargeting.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FiducialTargeting.dir/clean

CMakeFiles/FiducialTargeting.dir/depend:
	cd /home/blasterbotica/opencv/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/blasterbotica/opencv /home/blasterbotica/opencv /home/blasterbotica/opencv/build /home/blasterbotica/opencv/build /home/blasterbotica/opencv/build/CMakeFiles/FiducialTargeting.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FiducialTargeting.dir/depend
