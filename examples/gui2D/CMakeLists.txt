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

find_package(Boost COMPONENTS system REQUIRED)
find_package( OpenCV REQUIRED COMPONENTS opencv_core opencv_highgui opencv_imgproc)
find_package(Qt5Widgets)
find_package(Qt5Core)
find_package(Qt5Gui)

include_directories(${Boost_INCLUDE_DIRS})

QT5_WRAP_UI(UIS_HDRS
        ${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.ui)

set(source
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.h
    ${CMAKE_SOURCE_DIR}/examples/ui/Drawing2D.hpp
    ${UIS_HDRS})

set(ADDITIONAL_LIBRARIES
    ${OpenCV_LIBS})

ADD_RMPL_EXAMPLE(Gui2D "${source}" "${ADDITIONAL_LIBRARIES}")
qt5_use_modules(Gui2D Widgets Gui)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/spaces/ DESTINATION Release/spaces)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/spaces/ DESTINATION Debug/spaces)
