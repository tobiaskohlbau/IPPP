#-------------------------------------------------------------------------//
#
# Copyright 2016 Sascha Kaden
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#-------------------------------------------------------------------------//

find_package(Boost COMPONENTS unit_test_framework REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_TEST_DYN_LINK)
SET(LIBRARIES RobotMotionPlanning ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

ADD_DEFINITIONS("-DBOOST_TEST_MAIN")

MACRO(ADD_RMPL_TEST TEST_NAME ADDITIONAL_LIBS)
    SET(EXE_NAME Test_${MODULE}_${TEST_NAME})

    INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
    ADD_EXECUTABLE(${EXE_NAME} ${TEST_NAME}.cpp)
    TARGET_LINK_LIBRARIES(${EXE_NAME} ${LIBRARIES} ${ADDITIONAL_LIBS})

    SET(TEST_NAME ${MODULE}_${TEST_NAME})
    ADD_TEST(${TEST_NAME} ${EXE_NAME} COMMAND Test)

    SET_TESTS_PROPERTIES(${TEST_NAME} PROPERTIES
            FAIL_REGULAR_EXPRESSION "FAILED")

ENDMACRO(ADD_RMPL_TEST TEST_NAME)

# from: http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            SET(dirlist ${dirlist} ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()