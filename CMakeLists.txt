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

set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wno-write-strings")

include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include/vrep)

set(SRCS
    core/CollisionDetection.cpp
    ui/Drawing.cpp
    core/Edge.cpp
    core/Graph.cpp
    core/Node.cpp
    core/Sampling.cpp
    core/TrajectoryPlanner.cpp
    planner/NormalRRTPlanner.cpp
    planner/Planner.cpp
    planner/RRTPlanner.cpp
    planner/StarRRTPlanner.cpp
    include/core/KDNode.hpp
    include/core/KDTree.hpp
    include/core/Vec.hpp
    vrep/extApi.c
    vrep/extApiPlatform.c
    vrep/Helper.cpp
)

add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} pthread)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib)
install(DIRECTORY include DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
