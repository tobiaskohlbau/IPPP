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
if(OpenCV_VERSION VERSION_LESS "2.4.0")
    message (FATAL_ERROR "ERROR: can|t find OpenCV 2.4.0+")
elseif(OpenCV_VERSION VERSION_LESS "3.0.0")
    find_package(OpenCV REQUIRED QUIET COMPONENTS opencv_core opencv_highgui opencv_imgproc)
else()
    find_package(OpenCV REQUIRED QUIET COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs)
endif()

find_package(Qt5Core)
if(Qt5Core_FOUND)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)

    find_package(Qt5Widgets REQUIRED)
    find_package(Qt5Gui REQUIRED)

    QT5_WRAP_UI(UIS_HDRS
            ${CMAKE_CURRENT_SOURCE_DIR}/mainwindow.ui)

    add_ippp_example(Gui2D main.cpp mainwindow.cpp ui Qt5::Core Qt5::Widgets Qt5::Gui ${OPENCV_LIBS})
    target_compile_definitions(Gui2D
        PRIVATE
            ${QT_DEFINITIONS})

    # needed for clang-cl, fixed in qt 5.9 which is not available from vcpkg right now
    if (Qt5Gui_VERSION VERSION_EQUAL 5.8.0 AND ${CMAKE_CXX_COMPILER} MATCHES ".*clang-cl.*")
        target_compile_definitions(Gui2D
            PRIVATE
                _HAS_RVALUE_REFERENCES)
    endif()
endif()
