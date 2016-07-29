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

namespace rmpl {

/*!
* \brief   Super class of the RRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class RRTPlanner : public Planner
{
public:
    RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase> &robot, float stepSize, float trajectoryStepSize, TrajectoryMethod trajectory, SamplingMethod sampling);

    bool setInitNode(Node node);
    bool computeTree(int nbOfNodes, int nbOfThreades = 1);
    virtual bool connectGoalNode(Node goalNode) = 0;
    std::vector<std::shared_ptr<Node>> getPathNodes();
    std::vector<Vec<float>> getPath(float trajectoryStepSize);
    std::shared_ptr<Node> getInitNode();
    std::shared_ptr<Node> getGoalNode();
    Vec<float> getSamplePoint();

protected:
    bool controlConstraints();
    void computeTreeThread(int nbOfNodes);
    virtual void computeRRTNode(const Vec<float> &randVec, std::shared_ptr<Node> &newNode) = 0;
    Vec<float> computeNodeNew(const Vec<float> &randNode, const Vec<float> &nearestNode);

    // variables
    std::shared_ptr<Node>  m_initNode;
    std::shared_ptr<Node>  m_goalNode;
};

} /* namespace rmpl */

#endif /* RRTPLANNER_H_ */
