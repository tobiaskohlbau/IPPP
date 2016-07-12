cmake_minimum_required (VERSION 2.8.10)
project (RobotMotionPlanning)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package( OpenCV REQUIRED COMPONENTS opencv_core opencv_highgui opencv_imgproc)
set(RMP_DIR "/home/sascha/projects/robotMotion_build" CACHE PATH "Robot Motion Planner DIR")
link_directories(${RMP_DIR}/lib)
include_directories(${RMP_DIR}/include)
include_directories(${RMP_DIR}/eigen3)
include_directories(${RMP_DIR})

set(PQP_DIR "/home/sascha/projects/pqp" CACHE PATH "PQP DIR")
include_directories(${PQP_DIR}/include)
link_directories(${PQP_DIR}/lib)

#QT
#find_package(Qt5Widgets)
#find_package(Qt5Core)
#find_package(Qt5Gui)
#find_package(Qt5Quick)
#find_package(Qt5Qml)
#qt5_add_resources(RESOURCES ${CMAKE_SOURCE_DIR}/ui/qml.qrc)

set(SRCS
    main.cpp
    ui/Drawing.cpp)

set(HDRS
    ui/Drawing.h)

add_executable(planner ${SRCS} ${HDRS})
target_link_libraries( planner RobotMotionPlanningLib ${OpenCV_LIBS} pthread PQP)
    #Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Quick Qt5::Qml

# copy the workspaces and meshes
file(COPY ${CMAKE_SOURCE_DIR}/spaces DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_SOURCE_DIR}/meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
