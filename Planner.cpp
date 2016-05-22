#include "Planner.h"

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
            std::cout << "Wrong boudaries set" << std::endl;
            return;
        }
    }
    m_maxBoundary = maxBoundary;
    m_minBoundary = minBoundary;
    m_sampler->setBoundaries(minBoundary, maxBoundary);
}

std::vector<std::shared_ptr<Node>> &nodes Planner::getTree() const {
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
