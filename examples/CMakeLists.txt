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

SET(LIBRARIES
        IPPP
        ${PQP_LIBRARIES}
        ${ASSIMP_LIBRARIES}
        ${fcl_LIBRARIES}
        ${CCD_LIBRARIES})

MACRO(ADD_IPPP_EXAMPLE name source ADDITIONAL_LIBRARIES)
    ADD_EXECUTABLE(${name} ${source})
	set_property(TARGET ${name} PROPERTY CXX_STANDARD 11)
    TARGET_LINK_LIBRARIES(${name} ${LIBRARIES} ${ADDITIONAL_LIBRARIES})
    TARGET_INCLUDE_DIRECTORIES(${name} PUBLIC
        ${CMAKE_SOURCE_DIR}/examples
        ${CMAKE_CURRENT_SOURCE_DIR})
    add_custom_command(TARGET ${name} POST_BUILD
         COMMAND ${CMAKE_COMMAND} -E copy_directory
             ${CMAKE_SOURCE_DIR}/examples/assets
             ${CMAKE_CURRENT_BINARY_DIR}/assets/)
ENDMACRO()

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
list(REMOVE_ITEM SUBDIRS "ui")
list(REMOVE_ITEM SUBDIRS "assets")

FOREACH(subdir ${SUBDIRS})
    ADD_SUBDIRECTORY(${subdir})
ENDFOREACH()

