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

#ifndef JACO_H
#define JACO_H

#include <ippp/environment/robot/SerialRobot.h>

namespace ippp {

/*!
* \brief   Class for the Jaco robot
* \author  Sascha Kaden
* \date    2017-12-15
*/
class Jaco : public SerialRobot {
  public:
    Jaco(unsigned int dim, const std::vector<Joint> &joints, const std::vector<DofType> &dofTypes);
    std::vector<Transform> getJointTrafos(const VectorX &angles) const override;
    virtual std::vector<Transform> getLinkTrafos(const VectorX &angles) const override;
    virtual std::pair<std::vector<Transform>, Transform> getLinkAndToolTrafos(const VectorX &angles) const override;

  private:
    Vector6 convertRealToDH(const Vector6 &realAngles) const;
};

} /* namespace ippp */

#endif /* JACO_H */
