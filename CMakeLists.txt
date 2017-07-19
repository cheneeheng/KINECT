cmake_minimum_required(VERSION 2.8)

project(HMAR)

add_compile_options(-std=c++11)

set(OpenCV_LIBS_DIR "/usr/local/src/opencv-3.1/build/lib")
set(OpenCV_INCLUDE_DIR "/usr/local/include/opencv2")
set(DBOT_INCLUDE_DIR "/home/chen/projects/tracking/src/dbot/source")
set(DBOT_LIBS_DIR "/home/chen/projects/tracking/src/dbot/build")
set(FL_INCLUDE_DIR "/home/chen/projects/tracking/src/fl/include")
set(FL_LIBS_DIR "/home/chen/projects/tracking/src/fl/build")

set(GCC_GDB_COMPILE_FLAGS "-g")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_GDB_COMPILE_FLAGS}")

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(OpenGL REQUIRED)

# Compile external dependencies 
add_subdirectory (external)

# On Visual 2005 and above, this module can set the debug working directory
cmake_policy(SET CMP0026 OLD)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/rpavlik-cmake-modules-fe2273")
include(CreateLaunchers)
include(MSVCMultipleProcessCompile) # /MP

if(INCLUDE_DISTRIB)
	add_subdirectory(distrib)
endif(INCLUDE_DISTRIB)

include_directories(
	external/AntTweakBar-1.16/include/
	external/glfw-3.1.2/include/GLFW/
	external/glm-0.9.7.1/
	external/glew-1.13.0/include/
	external/assimp-3.0.1270/include/
	external/bullet-2.81-rev2613/src/
	.
)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
	-DDATA2
)

include_directories( 
	/usr/include/eigen3
	/usr/local/src/XVision2/include 
	/usr/local/intel/ipp/5.2/em64t/include 
	/usr/local/src/OpenNI2/Include 
	/home/chen/gsl/include
	/home/chen/TTS/festival/src/include
	/home/chen/TTS/speech_tools/include
	${FL_INCLUDE_DIR}
	${DBOT_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIR})

link_directories( 
	/home/chen/gsl/lib
	/usr/lib64 
	/usr/local/src/XVision2/lib 
	/usr/local/intel/ipp/5.2/em64t/sharedlib 
	/usr/local/src/OpenNI2/Bin/x64-Release 
	/home/chen/TTS/festival/src/lib
	/home/chen/TTS/speech_tools/lib
	${FL_LIBS_DIR}
	${DBOT_LIBS_DIR}
	${OpenCV_LIBS_DIR})

add_executable(HMAR
	common/graphparser.hpp
	common/graphparser.cpp
	common/shader.cpp
	common/shader.hpp
	common/objectloader.cpp
	common/objectloader.hpp
	common/controls.hpp
	common/controls.cpp
	common/location_parser.cpp
	common/location_parser.hpp
	common/quaternion_utils.cpp
	common/quaternion_utils.hpp
	common/TransformVertexShader.vertexshader
	common/ColorFragmentShader.fragmentshader
  	./src/VTKExtra.cpp 
  	./src/COS.cpp 
  	./src/CKB.cpp 
  	./src/CAS.cpp 
  	./src/CData.cpp 
  	./src/CGraph.cpp 
  	./src/algo.cpp 
  	./src/DataFilter.cpp
  	./src/print.cpp
  	./src/core.cpp
  	./src/TestCase.cpp 
  	./src/Evaluate.cpp 
  	./src/ReadFile.cpp 
  	./src/WriteFile.cpp 
  	./src/DataParser.cpp 
  	./src/NBayesClassifier.cpp 
  	./src/Test.cpp 
  	./src/Train.cpp 
  	./src/TrainLA.cpp 
  	./src/TrainSM.cpp 
  	./src/ObjectPrediction.cpp  
  	./src/ActionPrediction.cpp 
  	./src/ActionParser.cpp 
  	./src/DBSCAN.cpp 
	./src/OpenGLViewer.cpp 
	./src/PFTracker.cpp 
	./src/ObjectDetection.cpp 
	./src/main.cpp)

target_link_libraries( HMAR 
	png Xext X11 pthread 
	opencv_calib3d opencv_core opencv_videoio opencv_highgui opencv_imgproc
	opencv_imgcodecs opencv_objdetect
	XVSeg XVCons XVTrack XVDevs XVTools XVImages 
	ippccem64t ippcvem64t ippiem64t 
	OpenNI2
	dbot_gpu dbot
	${VTK_LIBRARIES}
	gsl gslcblas
	Festival estools estbase eststring ncurses
	${OPENGL_LIBRARY} glfw GLEW_1130 ANTTWEAKBAR_116_OGLCORE_GLFW 
)


#message("CMAKE_CXX_FLAGS is ${CMAKE_CXX_FLAGS}")
#message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
#message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
#message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
#message("CMAKE_CXX_FLAGS_MINSIZEREL is ${CMAKE_CXX_FLAGS_MINSIZEREL}")
