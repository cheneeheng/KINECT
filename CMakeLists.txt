cmake_minimum_required(VERSION 2.8)

################################################################################
# Compiler option                                                              #
################################################################################
add_compile_options(-std=c++11)

################################################################################
# Find source files                                                            #
################################################################################
file(GLOB HMAR_SOURCES 
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
  	)

file(GLOB COMMON_SOURCES 
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
  	)


################################################################################
# Dependencies                                                                 #
################################################################################

set(HMAR_INCLUDE_DIRS 
	/usr/include/eigen3
	/usr/local/src/XVision2/include 
	/usr/local/intel/ipp/5.2/em64t/include 
	/usr/local/src/OpenNI2/Include # kinect
	/home/chen/gsl/include
	/home/chen/TTS/festival/src/include
	/home/chen/TTS/speech_tools/include
	/usr/local/include/opencv2
	/home/chen/projects/tracking/src/dbot/source
	/home/chen/projects/tracking/src/fl/include
)

set(HMAR_LIB_DIRS 
	/home/chen/gsl/lib
	/usr/lib64 
	/usr/local/src/XVision2/lib 
	/usr/local/intel/ipp/5.2/em64t/sharedlib 
	/usr/local/src/OpenNI2/Bin/x64-Release 
	/home/chen/TTS/festival/src/lib
	/home/chen/TTS/speech_tools/lib
	/usr/local/src/opencv-3.1/build/lib
	/home/chen/projects/tracking/src/dbot/build
	/home/chen/projects/tracking/src/fl/build
)

################################################################################
# Libraries                                                                    #
################################################################################

set(OPENCV_LIBS 
	opencv_calib3d
	opencv_core
	opencv_videoio
	opencv_highgui
	opencv_imgproc
	opencv_imgcodecs
	opencv_objdetect
)

set(XV_LIBS 
	XVSeg
	XVCons
	XVTrack
	XVDevs
	XVTools
	XVImages 
)

set(TTS_LIBS 
	Festival
	estools
	estbase
	eststring
	ncurses
)

set(VIEWER_LIBS 
	glfw
	GLEW_1130
	ANTTWEAKBAR_116_OGLCORE_GLFW 
)

set(IPPI_LIBS 
	ippccem64t
	ippcvem64t
	ippiem64t 
)

set(HMAR_LIBS 
	png Xext X11 pthread 
	OpenNI2
	gsl gslcblas
	dbot dbot_gpu
)
list(APPEND HMAR_LIBS ${OPENCV_LIBS})
list(APPEND HMAR_LIBS ${XV_LIBS})
list(APPEND HMAR_LIBS ${TTS_LIBS})
list(APPEND HMAR_LIBS ${VIEWER_LIBS})
list(APPEND HMAR_LIBS ${IPPI_LIBS})

################################################################################
# Build project                                                                #
################################################################################
project(HMAR)

set(GCC_GDB_COMPILE_FLAGS "-g")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_GDB_COMPILE_FLAGS}")

########################################
# OPENGL based viewer library          #
########################################
find_package(OpenGL REQUIRED)

# Compile external dependencies 
add_subdirectory (external)

# On Visual 2005 and above, this module can set the debug working directory
cmake_policy(SET CMP0026 OLD)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/rpavlik-cmake-modules-fe2273")
include(CreateLaunchers)
include(MSVCMultipleProcessCompile) # /MP

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
)

########################################
# HMAR project                         #
########################################
add_definitions( -DDATA2 )

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories( ${HMAR_INCLUDE_DIRS} )

link_directories( ${HMAR_LIB_DIRS} )

add_executable( HMAR
    ${COMMON_SOURCES}
    ${HMAR_SOURCES}
	./src/main.cpp
)

target_link_libraries( HMAR
	${HMAR_LIBS}
	${VTK_LIBRARIES}
	${OPENGL_LIBRARY} 
)

