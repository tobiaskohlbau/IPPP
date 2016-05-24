#ifndef PLANNER_H_
#define PLANNER_H_

#include <cstdint>
#include <iostream>

#include <CollisionDetection.h>
#include <Graph.h>
#include <Sampling.h>
#include <TrajectoryPlanner.h>

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

Planner::Planner(const unsigned int &dim, const float &stepSize, TrajectoryMethod trajectory, SamplingMethod sampling) {
    m_dim = dim;
    m_stepSize = stepSize;
    m_sampler = new Sampling(sampling);
    m_collision = new CollisionDetection();
    m_planner = new TrajectoryPlanner(trajectory, m_collision);
}

void Planner::set2DWorkspace(cv::Mat space) {
    m_workspace = space;
    m_collision->set2DWorkspace(m_workspace);
}

void Planner::setWorkspaceBoundaries(Vec<float> &minBoundary, Vec<float> &maxBoundary) {
    if (minBoundary.empty() || maxBoundary.empty()) {
        std::cout << "No boundaries set" << std::endl;
        return;
    }
    else if (minBoundary.getDim() != maxBoundary.getDim()) {
        std::cout << "Boudaries have different dimensions" << std::endl;
        return;
    }

    for (unsigned int i = 0; i < m_dim; ++i) {
        if (minBoundary[i] > maxBoundary[i]) {
            std::cout << "Min boundary is larger than max boundary" << std::endl;
            return;
        }
    }
    m_maxBoundary = maxBoundary;
    m_minBoundary = minBoundary;
    m_sampler->setBoundaries(minBoundary, maxBoundary);
}

std::vector<std::shared_ptr<Node>> Planner::getTree() {
    return m_graph.getNodes();
}

bool Planner::controlConstraints() {
    if (m_minBoundary.empty() || m_maxBoundary.empty()) {
        std::cout << "No boudaries set" << std::endl;
        return false;
    }
    if (m_dim == 2 && this->m_workspace.empty())
        return false;
    else
        return true;
}

#endif /* RRTPLANNER_H_ */
