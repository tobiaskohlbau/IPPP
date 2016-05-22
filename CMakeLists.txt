cmake_minimum_required (VERSION 2.8.10)
project (RobotMotionPlanning)
set(CMAKE_AUTOMOC ON)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package( OpenCV REQUIRED COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d)
#include_directories( ${OpenCV_INCLUDE_DIRS} )

set(VREP_DIR "/opt/v-rep" CACHE PATH "v-rep directory")
set(VREP_REMOTE ${VREP_DIR}/programming/remoteApi)
set(VREP_INCLUDE ${VREP_DIR}/programming/include)
set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wno-write-strings")
include_directories (
    ${VREP_DIR}/programming/common
    ${VREP_DIR}/programming/include
    ${VREP_DIR}/programming/remoteApi)
link_directories(
    ${VREP_DIR}/programming/v_repExtRemoteApi/lib)

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/planner)

#source
set(SRCS
    main.cpp)

#header
set(HDRS
    CollisionDetection.h
    Edge.h
    Drawing.h
    Graph.h
    KDNode.h
    KDTree.h
    Node.h
    planner/NormalRRTPlanner.h
    planner/Planner.h
    planner/RRTPlanner.h
    planner/StarRRTPlanner.h
    Sampling.h
    TrajectoryPlanner.h
    Vec.h
    VrepHelper.h)

set(VREP
    ${VREP_REMOTE}/extApiPlatform.c
    ${VREP_REMOTE}/extApi.c
    ${VREP_INCLUDE}/v_repConst.h)

add_executable(${PROJECT_NAME} ${SRCS} ${HDRS} ${VREP})
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} pthread)

# copy the workspace images
configure_file(${CMAKE_SOURCE_DIR}/examples/spaces/freeWorkspace.png
    ${CMAKE_CURRENT_BINARY_DIR}/examples/spaces/freeWorkspace.png COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/examples/spaces/obstacleWorkspace.png
    ${CMAKE_CURRENT_BINARY_DIR}/examples/spaces/obstacleWorkspace.png COPYONLY)

# copy the dll files to the executable at windows
IF (WIN32)
# DEBUG
configure_file(${OpenCV_DIR}/bin/Debug/opencv_highgui300d.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Debug/opencv_highgui300d.dll COPYONLY)
configure_file(${OpenCV_DIR}/bin/Debug/opencv_core300d.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Debug/opencv_core300d.dll COPYONLY)
configure_file(${OpenCV_DIR}/bin/Debug/opencv_imgproc300d.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Debug/opencv_imgproc300d.dll COPYONLY)
configure_file(${OpenCV_DIR}/bin/Debug/opencv_imgcodecs300d.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Debug/opencv_imgcodecs300d.dll COPYONLY)
configure_file(${OpenCV_DIR}/bin/Debug/opencv_videoio300d.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Debug/opencv_videoio300d.dll COPYONLY)

# RELEASE
configure_file(${OpenCV_DIR}/bin/Release/opencv_highgui300.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Release/opencv_highgui300.dll COPYONLY)
configure_file(${OpenCV_DIR}/bin/Release/opencv_core300.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Release/opencv_core300.dll COPYONLY)
configure_file(${OpenCV_DIR}/bin/Release/opencv_imgproc300.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Release/opencv_imgproc300.dll COPYONLY)
configure_file(${OpenCV_DIR}/bin/Release/opencv_imgcodecs300.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Release/opencv_imgcodecs300.dll COPYONLY)
configure_file(${OpenCV_DIR}/bin/Release/opencv_videoio300.dll
    ${CMAKE_CURRENT_BINARY_DIR}/Release/opencv_videoio300.dll COPYONLY)
ENDIF()
