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

#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "Planner.h"
#include <pathPlanner/options/RRTOptions.h>

namespace rmpl {

/*!
* \brief   Super class of the RRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class RRTPlanner : public Planner {
  public:
    RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase> &robot, const RRTOptions &options);

    bool computePath(Eigen::VectorXf start, Eigen::VectorXf goal, unsigned int numNodes, unsigned int numThreads);
    bool setInitNode(Eigen::VectorXf start);
    bool computeTree(unsigned int nbOfNodes, unsigned int nbOfThreads = 1);
    virtual bool connectGoalNode(Eigen::VectorXf goal) = 0;

    std::vector<std::shared_ptr<Node>> getPathNodes();
    std::vector<Eigen::VectorXf> getPath(float trajectoryStepSize, bool smoothing = true);
    std::shared_ptr<Node> getInitNode();
    std::shared_ptr<Node> getGoalNode();

  protected:
    bool controlConstraints();
    void computeTreeThread(unsigned int nbOfNodes);
    virtual void computeRRTNode(const Eigen::VectorXf &randVec, std::shared_ptr<Node> &newNode) = 0;
    Eigen::VectorXf computeNodeNew(const Eigen::VectorXf &randNode, const Eigen::VectorXf &nearestNode);

    // variables
    float m_stepSize;
    std::shared_ptr<Node> m_initNode;
    std::shared_ptr<Node> m_goalNode;
};

} /* namespace rmpl */

#endif /* RRTPLANNER_H_ */
