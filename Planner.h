#ifndef PLANNER_H_
#define PLANNER_H_

#include <cstdint>

#include "CollisionDetection.h"
#include "Graph.h"
#include "Node.h"
#include "Sampling.h"
#include "TrajectoryPlanner.h"

template<uint16_t dim>
class Planner
{
public:
    Planner(float stepSize, TrajectoryPlanner::TrajectoryMethod trajectory, Sampling::SamplingMethod sampling);

    void setWorkspace(cv::Mat space);
    std::vector<Node<dim>> getPath();
    void getTree(std::vector<std::shared_ptr<Node<dim>>> &nodes) const;

protected:
    bool controlConstraints();

    // modules
    TrajectoryPlanner  *m_planner;
    Sampling           *m_sampler;
    CollisionDetection *m_collision;

    // variables
    float      m_stepSize;
    Graph<dim> m_graph;
    cv::Mat    m_workspace;
};

template<uint16_t dim>
Planner<dim>::Planner(float stepSize, TrajectoryPlanner::TrajectoryMethod trajectory, Sampling::SamplingMethod sampling) {
    m_stepSize = stepSize;
    m_sampler = new Sampling();
    m_collision = new CollisionDetection();
    m_planner = new TrajectoryPlanner(trajectory, m_collision);
}

template<uint16_t dim>
void Planner<dim>::setWorkspace(cv::Mat space) {
    m_workspace = space;
    m_collision->setWorkspace(m_workspace);
}

template<uint16_t dim>
void Planner<dim>::getTree(std::vector<std::shared_ptr<Node<dim>>> &nodes) const {
    nodes = m_graph.getNodes();
}

template<uint16_t dim>
bool Planner<dim>::controlConstraints() {
    if (dim == 2 && this->m_workspace.empty())
        return false;
    else
        return true;
}

#endif /* RRTPLANNER_H_ */
