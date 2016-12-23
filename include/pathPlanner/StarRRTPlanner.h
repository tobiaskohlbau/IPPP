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

#ifndef STARRRTPLANNER_H_
#define STARRRTPLANNER_H_

#include <mutex>

#include "RRTPlanner.h"

namespace rmpl {

/*!
* \brief   Class of the StarRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class StarRRTPlanner : public RRTPlanner {
  public:
    StarRRTPlanner(const std::shared_ptr<RobotBase> &robot, const RRTOptions &options)
        : RRTPlanner("RRT* Planner", robot, options) {
    }
    bool connectGoalNode(Eigen::VectorXf goal);

  protected:
    void computeRRTNode(const Eigen::VectorXf &randVec, std::shared_ptr<Node> &newNode);
    void chooseParent(std::shared_ptr<Node> &newNode, std::shared_ptr<Node> &nearestNode,
                      std::vector<std::shared_ptr<Node>> &nearNodes);
    void reWire(std::shared_ptr<Node> &newNode, std::shared_ptr<Node> &nearestNode,
                std::vector<std::shared_ptr<Node>> &nearNodes);

    std::mutex m_mutex;
};

} /* namespace rmpl */

#endif /* STARRRTPLANNER_H_ */
