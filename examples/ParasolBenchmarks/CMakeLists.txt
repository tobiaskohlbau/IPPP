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

find_package(Boost COMPONENTS filesystem system REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/examples)
        #/usr/include)

set(source
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
        ${CMAKE_SOURCE_DIR}/examples/ui/Writer.hpp
        ${CMAKE_SOURCE_DIR}/examples/ui/BenchmarkReader.h
        ${CMAKE_SOURCE_DIR}/examples/ui/BenchmarkReader.cpp)

#set(ADDITIONAL_LIBRARIES profiler tcmalloc)

#link_directories(/usr/lib)

ADD_RMPL_EXAMPLE(ParasolBenchmarks "${source}" "${ADDITIONAL_LIBRARIES}")
