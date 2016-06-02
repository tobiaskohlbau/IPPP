#include <planner/Planner.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class Planner
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[in]  TrajectoryMethod
*  \param[in]  SamplingMethod
*  \date       2016-05-27
*/
Planner::Planner(const std::string &name, const unsigned int &dim, const float &stepSize, TrajectoryMethod trajectory, SamplingMethod sampling)
        : Base(name){
    m_pathPlanned = false;
    m_dim = dim;
    m_stepSize = stepSize;
    m_sampler = std::shared_ptr<Sampling>(new Sampling(sampling));
    m_vrep = std::shared_ptr<Helper>(new Helper(m_dim));
    m_collision = std::shared_ptr<CollisionDetection>(new CollisionDetection(m_vrep));
    m_planner = std::shared_ptr<TrajectoryPlanner>(new TrajectoryPlanner(trajectory, m_collision));
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
        this->sendMessage("No boundaries set");
        return;
    }
    else if (minBoundary.getDim() != maxBoundary.getDim()) {
        this->sendMessage("Boudaries have different dimensions");
        return;
    }

    for (unsigned int i = 0; i < m_dim; ++i) {
        if (minBoundary[i] > maxBoundary[i]) {
            this->sendMessage("Min boundary is larger than max boundary");
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
    return m_graph->getNodes();
}

std::shared_ptr<Helper> Planner::getVrep() {
    return m_vrep;
}

/*!
*  \brief      Control all constraints of the Planner
*  \author     Sascha Kaden
*  \param[out] check flag
*  \date       2016-05-27
*/
bool Planner::controlConstraints() {
    if (m_minBoundary.empty() || m_maxBoundary.empty()) {\
        this->sendMessage("Boundaries are Empty!");
        return false;
    }
    if (m_dim == 2 && this->m_workspace.empty())
        return false;
    else
        return true;
}
