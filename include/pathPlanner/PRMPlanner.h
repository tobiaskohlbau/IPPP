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

#include <core/Edge.h>
#include <pathPlanner/AStarList.h>
#include <pathPlanner/Planner.h>


namespace rmpl {

class PRMPlanner : public Planner {
  public:
    PRMPlanner(const std::shared_ptr<RobotBase> &robot, float rangeSize, float trajectoryStepSize, TrajectoryMethod trajectory,
               SamplingMethod sampling);

    void startQueryPhase(unsigned int nbOfNodes, unsigned int nbOfThreads = 1);
    void startPlannerPhase(unsigned int nbOfThreads = 1);

    bool computePath(Node startNode, Node goalNode);
    bool aStar(std::shared_ptr<Node> sourceNode, std::shared_ptr<Node> targetNode);
    void expandNode(std::shared_ptr<Node> currentNode);

    std::vector<std::shared_ptr<Node>> getPathNodes();
    std::vector<Vec<float>> getPath(float trajectoryStepSize, bool smoothing = true);

protected:
    void queryPhase(unsigned int nbOfNodes);
    void plannerPhase(unsigned int startNodeIndex, unsigned int endNodeIndex);
    std::shared_ptr<Node> connectNode(Node &node);

    float m_rangeSize;
    std::vector<std::shared_ptr<Node>> m_nodePath;
    AStarList m_openList, m_closedList;
};

} /* namespace rmpl */

#endif    // PRMPLANNER_H_
