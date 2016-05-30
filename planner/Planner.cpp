#include <planner/Planner.h>

/*!
*  \brief      Constructor of the class Planner
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[in]  TrajectoryMethod
*  \param[in]  SamplingMethod
*  \date       2016-05-27
*/
Planner::Planner(const unsigned int &dim, const float &stepSize, TrajectoryMethod trajectory, SamplingMethod sampling) {
    m_dim = dim;
    m_stepSize = stepSize;
    m_sampler = new Sampling(sampling);
    m_collision = new CollisionDetection();
    m_planner = new TrajectoryPlanner(trajectory, m_collision);
}

/*!
*  \brief      Set 2D workspace to Planner and CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  2D workspace
*  \date       2016-05-27
*/
void Planner::set2DWorkspace(cv::Mat space) {
    m_workspace = space;
    m_collision->set2DWorkspace(m_workspace);
}

/*!
*  \brief      Set configuration space boundaries
*  \author     Sascha Kaden
*  \param[in]  minimum Boudaries
*  \param[in]  maximum Boudaries
*  \date       2016-05-27
*/
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

/*!
*  \brief      Return all nodes from the Graph
*  \author     Sascha Kaden
*  \param[out] list of all nodes
*  \date       2016-05-27
*/
std::vector<std::shared_ptr<Node>> Planner::getTree() {
    return m_graph.getNodes();
}

/*!
*  \brief      Control all constraints of the Planner
*  \author     Sascha Kaden
*  \param[out] check flag
*  \date       2016-05-27
*/
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

