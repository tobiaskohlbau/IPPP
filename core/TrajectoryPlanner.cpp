#include <core/TrajectoryPlanner.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Constructor of the class TrajectoryPlanner
*  \author     Sascha Kaden
*  \param[in]  TrajectoryMethod
*  \param[in]  pointer to ColllisionDetection instance
*  \date       2016-05-25
*/
TrajectoryPlanner::TrajectoryPlanner(const TrajectoryMethod &method, const shared_ptr<CollisionDetection> &collision)
    : Base("TrajectoryPlanner") {
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
bool TrajectoryPlanner::controlTrajectory(const shared_ptr<Node> &source, const shared_ptr<Node> &target, const float &stepSize) {
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
        this->sendMessage("Nodes/Vecs have different dimensions");
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
std::vector<shared_ptr<Node>> TrajectoryPlanner::computeTrajectory(const shared_ptr<Node> &source, const shared_ptr<Node> &target, const float &stepSize) {
    std::vector<shared_ptr<Node>> nodes;
    std::vector<Vec<float>> vecs = computeTrajectory(source->getVec(), target->getVec(), stepSize);
    for (int i = 0; i < vecs.size(); ++i)
        nodes.push_back(shared_ptr<Node>(new Node(vecs[i])));
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
        this->sendMessage("Nodes/Vecs have different dimensions");
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
