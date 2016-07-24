cmake_minimum_required (VERSION 2.8.10)
project (unitTest)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wno-write-strings")

#Setup CMake to run tests
enable_testing()

#Prep ourselves for compiling boost
find_package(Boost COMPONENTS unit_test_framework REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_TEST_DYN_LINK)

set(RMP_DIR "/home/sascha/projects/robotMotion_build" CACHE PATH "Robot Motion Planner DIR")
link_directories(${RMP_DIR}/lib)
include_directories(${RMP_DIR})
include_directories(${RMP_DIR}/include)

#I like to keep test files in a separate source directory called test
file(GLOB TEST_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

#Run through each source
foreach(testSrc ${TEST_SRCS})
        #Extract the filename without an extension (NAME_WE)
        get_filename_component(testName ${testSrc} NAME_WE)

        #Add compile target
        add_executable(${testName} ${testSrc})

        #link to Boost libraries AND your targets and dependencies
        target_link_libraries(${testName} RobotMotionPlanningLib ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

        #I like to move testing binaries into a testBin directory
        set_target_properties(${testName} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_CURRENT_BINARY_DIR}/testBin)

        #Finally add it to test execution -
        #Notice the WORKING_DIRECTORY and COMMAND
        add_test(NAME ${testName}
                 WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/testBin
                 COMMAND ${CMAKE_CURRENT_BINARY_DIR}/testBin/${testName} UnitTest
                 )
endforeach(testSrc)
