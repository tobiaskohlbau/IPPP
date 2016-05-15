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
    NormalRRTPlanner.h
    RRTPlanner.h
    Sampling.h
    SearchNearestNeighbor.h
    StarRRTPlanner.h
    TrajectoryPlanner.h
    Vec.h)

add_executable(${PROJECT_NAME} ${SRCS} ${HDRS})
# link the libraries to the executable
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${ADDITIONAL_LIBRARIES})

# copy the workspace images
configure_file(${CMAKE_SOURCE_DIR}/spaces/freeWorkspace.png
    ${CMAKE_CURRENT_BINARY_DIR}/spaces/freeWorkspace.png COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/spaces/obstacleWorkspace.png
    ${CMAKE_CURRENT_BINARY_DIR}/spaces/obstacleWorkspace.png COPYONLY)

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
