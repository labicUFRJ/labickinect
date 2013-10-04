cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(LabicKinect)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(/usr/local/include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (labickinect main.cpp arg.cpp arg.h RGBDImage.cpp RGBDImage.h KinectController.cpp KinectController.h LabicCV.cpp LabicCV.h Reconstructor.cpp Reconstructor.h RANSACAligner.cpp RANSACAligner.h LabicPCL.cpp LabicPCL.h libfreenect.h)

#add_executable (reconstructor testereconstructor.cpp KinectController.cpp KinectController.h LabicCV.cpp LabicCV.h LabicReconstructor.cpp LabicReconstructor.h RANSACAligner.cpp RANSACAligner.h LabicPCL.cpp LabicPCL.h libfreenect.h)

# to supress pcl libraries warnings
SET(GCC_COVERAGE_COMPILE_FLAGS "-Wall -Wno-deprecated -Wno-invalid-offsetof ")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

if(APPLE)
	set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
	target_link_libraries (labickinect freenect ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
	#target_link_libraries (reconstructor freenect ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
else()
	find_package(Threads REQUIRED)
	include_directories(${USB_INCLUDE_DIRS})
	link_directories(/usr/local/lib64)
	target_link_libraries(labickinect freenect ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB})
	#target_link_libraries(reconstructor freenect ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${MATH_LIB})
endif()
