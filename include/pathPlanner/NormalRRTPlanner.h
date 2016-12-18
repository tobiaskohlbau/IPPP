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

#ifndef NORMALRRTPLANNER_H_
#define NORMALRRTPLANNER_H_

#include <mutex>

#include "RRTPlanner.h"

namespace rmpl {

/*!
* \brief   Class of the NormalRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class NormalRRTPlanner : public RRTPlanner {
  public:
    NormalRRTPlanner(const std::shared_ptr<RobotBase> &robot, const RRTOptions &options)
        : RRTPlanner("Normal RRT Planner", robot, options) {
    }

    bool connectGoalNode(Vec<float> goal);

  protected:
    void computeRRTNode(const Vec<float> &randVec, std::shared_ptr<Node> &newNode);

    std::mutex m_mutex;
};

} /* namespace rmpl */

#endif /* NORMALRRTPLANNER_H_ */
