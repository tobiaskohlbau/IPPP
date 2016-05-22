#ifndef PLANNER_H_
#define PLANNER_H_

#include <cstdint>
#include <iostream>

#include "CollisionDetection.h"
#include "Graph.h"
#include "Node.h"
#include "Sampling.h"
#include "TrajectoryPlanner.h"

template<unsigned int dim>
class Planner
{
public:
    Planner(const float &stepSize, TrajectoryMethod trajectory, SamplingMethod sampling);

    void set2DWorkspace(cv::Mat space);
    std::vector<Node<dim>> getPath();
    void getTree(std::vector<std::shared_ptr<Node<dim>>> &nodes) const;
    void setWorkspaceBoundaries(Vec<dim, float> &minBoundary, Vec<dim, float> &maxBoundary);

protected:
    bool controlConstraints();

    // modules
    TrajectoryPlanner  *m_planner;
    Sampling<dim>      *m_sampler;
    CollisionDetection *m_collision;
    Graph<dim>          m_graph;

    // variables
    float           m_stepSize;
    cv::Mat         m_workspace;
    Vec<dim, float> m_maxBoundary;
    Vec<dim, float> m_minBoundary;
};

template<unsigned int dim>
Planner<dim>::Planner(const float &stepSize, TrajectoryMethod trajectory, SamplingMethod sampling) {
    m_stepSize = stepSize;
    m_sampler = new Sampling<dim>();
    m_collision = new CollisionDetection();
    m_planner = new TrajectoryPlanner(trajectory, m_collision);
}

template<unsigned int dim>
void Planner<dim>::set2DWorkspace(cv::Mat space) {
    m_workspace = space;
    m_collision->set2DWorkspace(m_workspace);
}

template<unsigned int dim>
void Planner<dim>::setWorkspaceBoundaries(Vec<dim, float> &minBoundary, Vec<dim, float> &maxBoundary) {
    for (unsigned int i = 0; i < dim; ++i) {
        if (minBoundary[i] > maxBoundary[i]) {
            std::cout << "Wrong boudaries set" << std::endl;
            return;
        }
    }
    m_maxBoundary = maxBoundary;
    m_minBoundary = minBoundary;
    m_sampler->setBoundaries(maxBoundary, minBoundary);
}

template<unsigned int dim>
void Planner<dim>::getTree(std::vector<std::shared_ptr<Node<dim>>> &nodes) const {
    nodes = m_graph.getNodes();
}

template<unsigned int dim>
bool Planner<dim>::controlConstraints() {
    if (m_minBoundary.empty() || m_maxBoundary.empty()) {
        std::cout << "No boudaries set" << std::endl;
        return false;
    }
    if (dim == 2 && this->m_workspace.empty())
        return false;
    else
        return true;
}

#endif /* RRTPLANNER_H_ */
