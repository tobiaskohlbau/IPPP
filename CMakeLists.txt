cmake_minimum_required (VERSION 2.8.10)
project (RobotMotionPlanningLib)
set(CMAKE_AUTOMOC ON)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package( OpenCV REQUIRED COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d)

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

#set(SRCS)

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
    ${VREP_REMOTE}/extApiPlatform.h
    ${VREP_REMOTE}/extApi.c
    ${VREP_REMOTE}/extApi.h
    ${VREP_INCLUDE}/v_repConst.h)

add_library(${PROJECT_NAME} STATIC  ${HDRS} ${VREP}) #${SRCS}
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} pthread)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib)
install(FILES ${HDRS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include)
install(FILES ${VREP} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include)
