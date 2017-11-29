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

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../third_party/gflags
    ${CMAKE_CURRENT_BINARY_DIR}/../third_party/gflags
    EXCLUDE_FROM_ALL)

macro(ADD_IPPP_EXAMPLE name source)
    add_executable(${name} ${source})
    target_link_libraries(${name}
        PRIVATE
            IPPP
            gflags
            ${ARGN})
endmacro()

add_ippp_example(GettingStarted GettingStarted.cpp)
add_ippp_example(JacoRRT JacoRRT.cpp)

add_subdirectory(ui)
add_subdirectory(Planner2D)
#add_subdirectory(gui2D)
add_subdirectory(ParasolBenchmarks)

