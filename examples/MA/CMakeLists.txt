#-------------------------------------------------------------------------//
#
# Copyright 2018 Sascha Kaden
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

add_library(ConfigurationMA STATIC ConfigurationMA.cpp)
target_link_libraries(ConfigurationMA
    PRIVATE
        IPPP)
set_target_properties(ConfigurationMA PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}")

if(OpenCV_FOUND)
    add_ippp_example(ConfigSpace configSpace.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(Planning2dFigures planning2dFigures.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationConstraint evaluationConstraint.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationConstraintObs evaluationConstraintObs.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationRRTs evaluationRRTs.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationRRTOptimization evaluationRRTOptimization.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationSampler evaluationSampler.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationStepSize evaluationStepSize.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationThreads evaluationThreads.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationConstStepSize evaluationConstStepSize.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationConstThreads evaluationConstThreads.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(EvaluationConstTest evaluationConstTest.cpp ui ConfigurationMA gflags)
    add_ippp_example(EvaluationEndTestConst evaluationEndTestConst.cpp ui ConfigurationMA gflags)
    add_ippp_example(EvaluationEndTestConst2 evaluationEndTestConst2.cpp ui ConfigurationMA gflags)
    add_ippp_example(EvaluationEndTest evaluationEndTest.cpp ui ConfigurationMA gflags)
    add_ippp_example(SaveEnvironments saveEnvironments.cpp ${OpenCV_LIBS} ui ConfigurationMA gflags)
    add_ippp_example(const ConstraintIiwa.cpp ui ConfigurationMA gflags)
endif()