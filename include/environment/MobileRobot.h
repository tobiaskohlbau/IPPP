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

#ifndef MOBILEROBOT_H
#define MOBILEROBOT_H

#include <environment/RobotBase.hpp>

namespace ippp {

template <unsigned int dim>
class MobileRobot : public RobotBase<dim> {
  public:
    MobileRobot(Vector<dim> minBoundary, Vector<dim> maxBoundary)
        : RobotBase<dim>("MobileRobot", RobotType::mobile, minBoundary, maxBoundary){};
};

} /* namespace ippp */

#endif    // MOBILEROBOT_H
