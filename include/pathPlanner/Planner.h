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

#ifndef PLANNER_H_
#define PLANNER_H_

#include <Eigen/Core>

#include <core/dataObj/Graph.h>
#include <core/module/CollisionDetection.h>
#include <core/module/ModuleBase.h>
#include <core/module/Sampling.h>
#include <core/module/TrajectoryPlanner.h>
#include <pathPlanner/options/PlannerOptions.h>
#include <robot/RobotBase.h>

namespace rmpl {

/*!
* \brief   Super class of all path planners
* \author  Sascha Kaden
* \date    2016-05-27
*/
class Planner : public ModuleBase {
  public:
    ~Planner();

  protected:
    Planner(const std::string &name, const std::shared_ptr<RobotBase> &robot, const PlannerOptions &options);

  public:
    virtual bool computePath(Eigen::VectorXf start, Eigen::VectorXf goal, unsigned int numNodes, unsigned int numThreads) = 0;

    std::vector<std::shared_ptr<Node>> getGraphNodes();
    virtual std::vector<Eigen::VectorXf> getPath(float trajectoryStepSize, bool smoothing) = 0;
    virtual std::vector<std::shared_ptr<Node>> getPathNodes() = 0;
    std::vector<Eigen::VectorXf> getPathFromNodes(const std::vector<std::shared_ptr<Node>> &nodes, float trajectoryStepSize,
                                                  bool smoothing);

  protected:
    std::vector<std::shared_ptr<Node>> smoothPath(std::vector<std::shared_ptr<Node>> nodes);

    std::shared_ptr<TrajectoryPlanner> m_planner;
    std::shared_ptr<Sampling> m_sampler;
    std::shared_ptr<CollisionDetection> m_collision;
    std::shared_ptr<Graph> m_graph;
    std::shared_ptr<RobotBase> m_robot;

    const PlannerOptions m_options;
    bool m_pathPlanned;
};

} /* namespace rmpl */

#endif /* RRTPLANNER_H_ */
