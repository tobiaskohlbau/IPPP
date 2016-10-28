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

set(source
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_SOURCE_DIR}/examples/ui/Drawing.h
    ${CMAKE_SOURCE_DIR}/examples/ui/Drawing.cpp)

find_package( OpenCV REQUIRED COMPONENTS opencv_core opencv_highgui opencv_imgproc)

set(ADDITIONAL_LIBRARIES
    ${OpenCV_LIBS})

ADD_RMPL_EXAMPLE(Planner2D "${source}" "${ADDITIONAL_LIBRARIES}")

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/spaces/ DESTINATION Release/spaces)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/spaces/ DESTINATION Debug/spaces)
