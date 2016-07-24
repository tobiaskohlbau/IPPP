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

#include <core/Base.h>
#include <core/CollisionDetection.h>
#include <core/Graph.h>
#include <core/Sampling.h>
#include <core/TrajectoryPlanner.h>
#include <robot/RobotBase.h>

namespace rmpl {

/*!
* \brief   Super class of all planners
* \author  Sascha Kaden
* \date    2016-05-27
*/
class Planner : public Base
{
public:
    Planner(const std::string &name, const std::shared_ptr<RobotBase> &robot, float stepSize, float trajectoryStepSize, TrajectoryMethod trajectory, SamplingMethod sampling);

    std::vector<std::shared_ptr<Node>> getGraphNodes();
    virtual std::vector<Vec<float>> getPath() = 0;
    virtual std::vector<std::shared_ptr<Node>> getPathNodes() = 0;
    std::shared_ptr<Helper> getVrep();

protected:
    std::shared_ptr<TrajectoryPlanner>  m_planner;
    std::shared_ptr<Sampling>           m_sampler;
    std::shared_ptr<CollisionDetection> m_collision;
    std::shared_ptr<Graph>              m_graph;
    std::shared_ptr<Helper>             m_vrep;
    std::shared_ptr<RobotBase>          m_robot;

    // variables
    bool         m_pathPlanned;
    float        m_stepSize;
};

} /* namespace rmpl */

#endif /* RRTPLANNER_H_ */
