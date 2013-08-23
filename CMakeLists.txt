cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(LabicKinect)

find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(/usr/local/include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (labickinect main.cpp labickinect.h kinect_old.cpp cv.cpp cv_matcher.cpp cv_matcher.h pcl.cpp pcl.h)
add_executable (labickinect main.cpp LabicKinect.cpp LabicKinect.h LabicCV.cpp LabicCV.h LabicMatcher.cpp LabicMatcher.h LabicPCL.cpp LabicPCL.h libfreenect.h)

# to supress pcl libraries warnings
if ( CMAKE_COMPILER_IS_GNUCC )
    set_property( TARGET labickinect APPEND_STRING PROPERTY COMPILE_FLAGS -Wno-deprecated\ -Wno-invalid-offsetof\ -Qunused-arguments )
endif ( CMAKE_COMPILER_IS_GNUCC )

set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")

target_link_libraries (labickinect ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} freenect)
