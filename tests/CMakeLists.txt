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

add_library(GTest::GTest ALIAS gtest)
add_library(GTest::Main ALIAS gtest_main)

macro(ADD_IPPP_TEST TEST_NAME BRANCH)
    set(EXE_NAME Test_${BRANCH}_${TEST_NAME})
    
    add_executable(${EXE_NAME} ${TEST_NAME}.cpp)
    target_link_libraries(${EXE_NAME} 
        PRIVATE
             IPPP
             GTest::GTest
             GTest::Main
             ${ARGN})

    add_test(${TEST_NAME} ${EXE_NAME})
endmacro(ADD_IPPP_TEST)

add_subdirectory(dataObj)
add_subdirectory(main)
add_subdirectory(module)
#add_subdirectory(ui)
add_subdirectory(utility)
