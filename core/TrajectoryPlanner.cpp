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
TrajectoryPlanner::TrajectoryPlanner(const TrajectoryMethod &method, const float &stepSize, const shared_ptr<CollisionDetection> &collision)
    : Base("TrajectoryPlanner") {
    m_method = method;
    m_collision = collision;

    if (stepSize <= 0) {
        this->sendMessage("Step size has to be larger than 0!");
        m_stepSize = -1;
    }
    else {
        m_stepSize = stepSize;
    }
}

/*!
*  \brief      Control the trajectory and return if possible or not
*  \author     Sascha Kaden
*  \param[in]  first Vec
*  \param[in]  second Vec
*  \param[in]  step size
*  \param[out] possibility of trajectory, true if possible
*  \date       2016-05-31
*/
bool TrajectoryPlanner::controlTrajectory(const Vec<float> &source, const Vec<float> &target) {
    if (source.getDim() != target.getDim()) {
        this->sendMessage("Nodes/Vecs have different dimensions");
        return false;
    }
    else if (m_stepSize == -1) {
        this->sendMessage("Step size is not set!");
        return false;
    }

    if (m_collision->controlCollision(computeTrajectory(source, target)))
        return false;

    return true;
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
std::vector<Vec<float>> TrajectoryPlanner::computeTrajectory(const Vec<float> &source, const Vec<float> &target) {
    std::vector<Vec<float>> vecs;

    if (source.getDim() != target.getDim()) {
        this->sendMessage("Nodes/Vecs have different dimensions");
        return vecs;
    }
    else if (m_stepSize == -1) {
        this->sendMessage("Step size is not set!");
        return vecs;
    }

    Vec<float> u = target - source;
    Vec<float> uNorm = u / u.norm();
    Vec<float> temp(source);
    while (std::abs(temp.norm() - target.norm()) > 1) {
        vecs.push_back(temp);
        temp += uNorm * m_stepSize;
    }
    return vecs;
}

void TrajectoryPlanner::setStepSize(const float &stepSize) {
    if (stepSize <= 0) {
        this->sendMessage("Step size has to be larger than 0!");
    }
    else {
        m_stepSize = stepSize;
    }
}
