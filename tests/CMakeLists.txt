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

if(CMAKE_VERSION VERSION_LESS 3.5)
    add_library(GTest::GTest UNKNOWN IMPORTED ${GTEST_LIBRARIES})
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
    if(EXISTS "${GTEST_LIBRARIES}")
        set_target_properties(GTest::GTest PROPERTIES IMPORTED_LOCATION "${GTEST_LIBRARIES}")
    endif()

    add_library(GTest::Main UNKNOWN IMPORTED ${GTEST_MAIN_LIBRARIES})
    set_target_properties(GTest::Main PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
    if(EXISTS "${GTEST_MAIN_LIBRARIES}")
        set_target_properties(GTest::Main PROPERTIES IMPORTED_LOCATION "${GTEST_MAIN_LIBRARIES}")
    endif()
endif()

macro(ADD_IPPP_TEST TEST_NAME BRANCH)
    set(EXE_NAME Test_${BRANCH}_${TEST_NAME})
    
    add_executable(${EXE_NAME} ${TEST_NAME}.cpp)
    target_link_libraries(${EXE_NAME} IPPP GTest::GTest GTest::Main)

    add_test(${TEST_NAME} ${EXE_NAME})
endmacro(ADD_IPPP_TEST)

add_subdirectory(dataObj)
add_subdirectory(main)
add_subdirectory(module)
add_subdirectory(utility)
