cmake_minimum_required (VERSION 2.8.10)
project (testVec)
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
set(RMP_DIR "${CMAKE_SOURCE_DIR}" CACHE PATH "Robot Motion Planner DIR")
link_directories(${RMP_DIR}/lib})
include_directories(${RMP_DIR}/include)

set(SRCS
    main.cpp)

set(HDRS)

add_executable(${PROJECT_NAME} ${SRCS}) #${HDRS}
target_link_libraries( ${PROJECT_NAME} RobotMotionPlanningLib ${OpenCV_LIBS} pthread)
