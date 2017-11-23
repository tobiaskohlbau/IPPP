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

find_package(gflags REQUIRED)

macro(ADD_IPPP_EXAMPLE name source)
    add_executable(${name} ${source})
    target_link_libraries(${name}
        PRIVATE
            IPPP
            gflags
            ${ARGN})
endmacro()

add_subdirectory(ui)

subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
list(REMOVE_ITEM SUBDIRS "ui")
list(REMOVE_ITEM SUBDIRS "assets")
list(REMOVE_ITEM SUBDIRS "gui2D")

foreach(subdir ${SUBDIRS})
    add_subdirectory(${subdir})
endforeach()

