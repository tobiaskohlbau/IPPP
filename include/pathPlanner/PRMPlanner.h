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

#ifndef PRMPLANNER_H_
#define PRMPLANNER_H_

#include <pathPlanner/Planner.h>
#include <pathPlanner/options/PRMOptions.h>

namespace rmpl {

/*!
* \brief   Class PRMPlanner
* \author  Sascha Kaden
* \date    2016-08-09
*/
class PRMPlanner : public Planner {
  public:
    PRMPlanner(const std::shared_ptr<RobotBase> &robot, const PRMOptions &options);

    bool computePath(Eigen::VectorXf start, Eigen::VectorXf goal, unsigned int numNodes, unsigned int numThreads);

    void startSamplingPhase(unsigned int nbOfNodes, unsigned int nbOfThreads = 1);
    void startPlannerPhase(unsigned int nbOfThreads = 1);

    bool queryPath(Eigen::VectorXf start, Eigen::VectorXf goal);
    bool aStar(std::shared_ptr<Node> sourceNode, std::shared_ptr<Node> targetNode);
    void expandNode(std::shared_ptr<Node> currentNode);

    std::vector<std::shared_ptr<Node>> getPathNodes();
    std::vector<Eigen::VectorXf> getPath(float trajectoryStepSize, bool smoothing = true);

  protected:
    void samplingPhase(unsigned int nbOfNodes);
    void plannerPhase(unsigned int startNodeIndex, unsigned int endNodeIndex);
    std::shared_ptr<Node> connectNode(Eigen::VectorXf &node);

    float m_rangeSize;
    std::vector<std::shared_ptr<Node>> m_nodePath;
    std::vector<std::shared_ptr<Node>> m_openList, m_closedList;
};

} /* namespace rmpl */

#endif    // PRMPLANNER_H_
