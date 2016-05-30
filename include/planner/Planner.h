#ifndef PLANNER_H_
#define PLANNER_H_

#include <cstdint>
#include <iostream>

#include <core/CollisionDetection.h>
#include <core/Graph.h>
#include <core/Sampling.h>
#include <core/TrajectoryPlanner.h>

/*!
* \brief   Super class of all planners
* \author  Sascha Kaden
* \date    2016-05-27
*/
class Planner
{
public:
    Planner(const unsigned int &dim, const float &stepSize, TrajectoryMethod trajectory, SamplingMethod sampling);

    void set2DWorkspace(cv::Mat space);
    std::vector<Node> getPath();
    std::vector<std::shared_ptr<Node>> getTree();
    void setWorkspaceBoundaries(Vec<float> &minBoundary, Vec<float> &maxBoundary);

protected:
    bool controlConstraints();

    // modules
    TrajectoryPlanner  *m_planner;
    Sampling           *m_sampler;
    CollisionDetection *m_collision;
    Graph               m_graph;

    // variables
    unsigned int m_dim;
    float        m_stepSize;
    cv::Mat      m_workspace;
    Vec<float>   m_maxBoundary;
    Vec<float>   m_minBoundary;
};

#endif /* RRTPLANNER_H_ */
