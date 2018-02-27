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

find_package(OpenCV REQUIRED QUIET)
if (OpenCV_VERSION VERSION_LESS "2.4.0")
    message (FATAL_ERROR "ERROR: can|t find OpenCV 2.4.0+")
elseif(OpenCV_VERSION VERSION_LESS "3.0.0")
    find_package(OpenCV REQUIRED QUIET COMPONENTS opencv_core opencv_highgui opencv_imgproc)
else()
    find_package(OpenCV REQUIRED QUIET COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs)
endif()

macro(ADD_IPPP_EXAMPLE name source)
    add_executable(${name} ${source})
    target_link_libraries(${name}
        PRIVATE
            IPPP
            ${ARGN})
endmacro()

add_subdirectory(ui)

add_ippp_example(GettingStarted GettingStarted.cpp gflags)
add_ippp_example(Kuka Kuka.cpp gflags)
if(OpenCV_FOUND)
	add_ippp_example(Planner2D Planner2D.cpp ${OpenCV_LIBS} ui gflags)
    add_ippp_example(Constraint2D Constraint2D.cpp ${OpenCV_LIBS} ui gflags)
    if(MSVC)
        target_compile_options(Planner2D PUBLIC -bigobj)
        target_compile_options(Constraint2D PUBLIC -bigobj)
    endif()
endif()
add_ippp_example(ParasolBenchmarks ParasolBenchmarks.cpp ui)
add_ippp_example(ConstraintIiwa ConstraintIiwa.cpp gflags)

#add_subdirectory(gui2D)
