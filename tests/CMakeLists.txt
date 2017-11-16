#-------------------------------------------------------------------------//
#
# Copyright 2017 Sascha Kaden
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

find_package(GTest REQUIRED)
SET(LIBRARIES IPPP ${GTEST_BOTH_LIBRARIES})

MACRO(ADD_IPPP_TEST TEST_NAME BRANCH)
    SET(EXE_NAME Test_${BRANCH}_${TEST_NAME})
    
    add_executable(${EXE_NAME} ${TEST_NAME}.cpp)
    set_property(TARGET ${EXE_NAME} PROPERTY CXX_STANDARD 11)
    target_link_libraries(${EXE_NAME} ${LIBRARIES})
    target_include_directories(${EXE_NAME} PUBLIC ${GTEST_INCLUDE_DIRS})
    add_custom_command(TARGET ${EXE_NAME} POST_BUILD
         COMMAND ${CMAKE_COMMAND} -E copy_directory
             ${CMAKE_SOURCE_DIR}/tests/assets
             ${CMAKE_CURRENT_BINARY_DIR}/assets/)

    ADD_TEST(UnitTests ${EXE_NAME})
ENDMACRO(ADD_IPPP_TEST)

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
list(REMOVE_ITEM SUBDIRS "assets")

FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()