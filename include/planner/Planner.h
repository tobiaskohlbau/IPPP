#ifndef PLANNER_H_
#define PLANNER_H_

#include <Eigen/Core>

#include <core/Base.h>
#include <core/CollisionDetection.h>
#include <core/Graph.h>
#include <robot/RobotBase.h>
#include <core/Sampling.h>
#include <core/TrajectoryPlanner.h>

namespace rmpl {

/*!
* \brief   Super class of all planners
* \author  Sascha Kaden
* \date    2016-05-27
*/
class Planner : public Base
{
public:
    Planner(const std::string &name, const std::shared_ptr<RobotBase> &robot, const float &stepSize, TrajectoryMethod trajectory, SamplingMethod sampling);

    void set2DWorkspace(Eigen::MatrixXi space);
    std::vector<std::shared_ptr<Node>> getGraphNodes();
    virtual std::vector<Vec<float>> getPath() = 0;
    virtual std::vector<std::shared_ptr<Node>> getPathNodes() = 0;
    void setWorkspaceBoundaries(Vec<float> &minBoundary, Vec<float> &maxBoundary);
    std::shared_ptr<Helper> getVrep() const;

protected:
    bool controlConstraints();

    // modules
    std::shared_ptr<TrajectoryPlanner>  m_planner;
    std::shared_ptr<Sampling>           m_sampler;
    std::shared_ptr<CollisionDetection> m_collision;
    std::shared_ptr<Graph>              m_graph;
    std::shared_ptr<Helper>             m_vrep;
    std::shared_ptr<RobotBase>          m_robot;

    // variables
    bool         m_pathPlanned;
    float        m_stepSize;
    Eigen::MatrixXi m_workspace;
    Vec<float>   m_maxBoundary;
    Vec<float>   m_minBoundary;
};

} /* namespace rmpl */

#endif /* RRTPLANNER_H_ */
