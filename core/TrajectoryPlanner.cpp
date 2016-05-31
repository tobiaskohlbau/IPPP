#include <core/TrajectoryPlanner.h>

/*!
*  \brief      Constructor of the class TrajectoryPlanner
*  \author     Sascha Kaden
*  \param[in]  TrajectoryMethod
*  \param[in]  pointer to ColllisionDetection instance
*  \date       2016-05-25
*/
TrajectoryPlanner::TrajectoryPlanner(const TrajectoryMethod method, CollisionDetection *collision) {
    m_method = method;
    m_collision = collision;
}

/*!
*  \brief      Control the trajectory and return if possible or not
*  \author     Sascha Kaden
*  \param[in]  first Node
*  \param[in]  second Node
*  \param[in]  step size
*  \param[out] possibility of trajectory
*  \date       2016-05-31
*/
bool TrajectoryPlanner::controlTrajectory(const std::shared_ptr<Node> &source, const std::shared_ptr<Node> &target, const float &stepSize) {
    return controlTrajectory(source->getVec(), target->getVec(), stepSize);
}

/*!
*  \brief      Control the trajectory and return if possible or not
*  \author     Sascha Kaden
*  \param[in]  first Vec
*  \param[in]  second Vec
*  \param[in]  step size
*  \param[out] possibility of trajectory
*  \date       2016-05-31
*/
bool TrajectoryPlanner::controlTrajectory(const Vec<float> &source, const Vec<float> &target, const float &stepSize) {
    if (source.getDim() != target.getDim()) {
        std::cout << "TrajectoryPlanner: Nodes/Vecs have different dimensions" << std::endl;
        return false;
    }

    std::vector<Vec<float>> vecs = computeTrajectory(source, target, stepSize);

    if (source.getDim() == 2) {
        for (int i = 0; i < vecs.size(); ++i) {
            if (m_collision->controlCollision(vecs[i]))
                return false;
        }
    }

    return true;
}

/*!
*  \brief      Compute the trajectory and return the list of nodes
*  \author     Sascha Kaden
*  \param[in]  first Node
*  \param[in]  second Node
*  \param[in]  step size
*  \param[out] trajectory
*  \date       2016-05-31
*/
std::vector<Node> TrajectoryPlanner::computeTrajectory(const std::shared_ptr<Node> &source, const std::shared_ptr<Node> &target, const float &stepSize) {
    std::vector<Node> nodes;
    std::vector<Vec<float>> vecs = computeTrajectory(source->getVec(), target->getVec(), stepSize);
    for (int i = 0; i < vecs.size(); ++i)
        nodes.push_back(Node(vecs[i]));
    return nodes;
}

/*!
*  \brief      Compute the trajectory and return the list of vecs
*  \author     Sascha Kaden
*  \param[in]  first Vec
*  \param[in]  second Vec
*  \param[in]  step size
*  \param[out] trajectory
*  \date       2016-05-31
*/
std::vector<Vec<float>> TrajectoryPlanner::computeTrajectory(const Vec<float> &source, const Vec<float> &target, const float &stepSize) {
    std::vector<Vec<float>> vecs;

    if (source.getDim() != target.getDim()) {
        std::cout << "TrajectoryPlanner: Nodes/Vecs have different dimensions" << std::endl;
        return vecs;
    }
    Vec<float> u = target - source;
    Vec<float> temp(source);
    while (std::abs(temp.norm() - target.norm()) > 1) {
        vecs.push_back(temp);
        temp += u * stepSize;
    }
    return vecs;
}
