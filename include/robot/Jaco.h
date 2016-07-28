//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef JACO_H_
#define JACO_H_

#include <robot/RobotBase.h>

namespace rmpl {

/*!
* \brief   Class for the jaco robot
* \author  Sascha Kaden
* \date    2016-06-30
*/
class Jaco : public RobotBase
{
public:
    Jaco();
    Vec<REAL> directKinematic(const Vec<REAL> &angles);
    std::vector<Eigen::Matrix4f> getTransformations(const Vec<REAL> &angles);

private:
    Vec<REAL> convertRealToDH(const Vec<REAL> &realAngles);
};

} /* namespace rmpl */

#endif /* JACO_H_ */
