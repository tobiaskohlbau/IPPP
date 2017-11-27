//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

#ifndef UTILENVIRONMENT_HPP
#define UTILENVIRONMENT_HPP

#include <ippp/environment/Environment.h>

namespace ippp {
namespace util {

/*!
*  \brief      Check the summation of the robot dimensions to the planner dimension (template), return true if valid.
*  \author     Sascha Kaden
*  \param[in]  environment pointer
*  \param[out] dimension validty
*  \date       2017-05-25
*/
template <unsigned int dim>
bool checkDimensions(const std::shared_ptr<Environment> &environment) {
    unsigned int robotDims = 0;
    for (auto &robot : environment->getRobots()) {
        robotDims += robot->getDim();
    }
    return robotDims == dim;
}

} /* namespace util */

} /* namespace ippp */

#endif /* ENVIRONMENT_H */
