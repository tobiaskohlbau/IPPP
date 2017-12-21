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

#ifndef DHPARAMETER_HPP
#define DHPARAMETER_HPP

namespace ippp {

/*!
* \brief   DH parameter class contains the four parameter alpha, theta, d offset and a offset from the parameter definition.
* \author  Sascha Kaden
* \date    2017-12-13
*/
class DhParameter {
  public:
    DhParameter(double alpha = 0, double a = 0, double d = 0, double theta = 0);

    double alpha = 0; /*!< offset angle (x axis) */
    double a = 0;     /*!< distance offset along the z axis */
    double d = 0;     /*!< distance offset along the x axis */
    double theta = 0; /*!< joint angle (z axis) */
};

} /* namespace ippp */

#endif /* DHPARAMETER_HPP */
