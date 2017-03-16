cmake_minimum_required(VERSION 2.8)

project(HMAR)

add_compile_options(-std=c++11)

set(OpenCV_LIBS_DIR "/usr/local/src/opencv-3.1/build/lib")
set(OpenCV_INCLUDE_DIR "/usr/local/include/opencv2")
set(GSL_LIBS_DIR "/home/chen/gsl/lib")
set(GSL_INCLUDE_DIR "/home/chen/gsl/include")

set(GCC_GDB_COMPILE_FLAGS "-g")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_GDB_COMPILE_FLAGS}")

#add_compile_options(-std=c++11)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories( 
	/usr/local/src/XVision2/include 
	/usr/local/intel/ipp/5.2/em64t/include 
	/usr/local/src/OpenNI2/Include 
	${GSL_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR})

link_directories( 
	/usr/lib64 
	/usr/local/src/XVision2/lib 
	/usr/local/intel/ipp/5.2/em64t/sharedlib 
	/usr/local/src/OpenNI2/Bin/x64-Release 
	${GSL_LIBS_DIR}
	${OpenCV_LIBS_DIR})

add_executable(HMAR
	./src/Graph.cpp
	./src/dbscan.cpp
	./src/util.cpp
	./src/vtkExtra.cpp
	./src/algo.cpp
  	./src/readWriteFile.cpp 
  	./src/labeling.cpp
  	./src/prediction.cpp
	./src/util2.cpp 
	./src/main.cpp)

target_link_libraries( HMAR 
	png Xext X11 pthread 
	opencv_calib3d opencv_core opencv_videoio opencv_highgui opencv_imgproc opencv_imgcodecs opencv_objdetect
	XVSeg XVCons XVTrack XVDevs XVTools XVImages 
	ippccem64t ippcvem64t ippiem64t 
	OpenNI2
	${VTK_LIBRARIES}
	gsl gslcblas)


#message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")
#message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
#message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
#message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
#message("CMAKE_CXX_FLAGS_MINSIZEREL is ${CMAKE_CXX_FLAGS_MINSIZEREL}")
