#ifndef STARRRTPLANNER_H_
#define STARRRTPLANNER_H_

#include <mutex>

#include "RRTPlanner.h"

namespace rmpl {

/*!
* \brief   Class of the StarRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class StarRRTPlanner : public RRTPlanner
{
public:
    StarRRTPlanner(const std::shared_ptr<RobotBase> &robot, float stepSize, float trajectoryStepSize, const TrajectoryMethod trajectory = TrajectoryMethod::linear, const SamplingMethod sampling = SamplingMethod::randomly)
    : RRTPlanner("RRT* Planner", robot, stepSize, trajectoryStepSize, trajectory, sampling) // Argumente an Basisklassenkonstruktor weiterleiten
    {
    }

protected:
    void computeRRTNode(const Vec<float> &randVec, std::shared_ptr<Node> &newNode);
    void chooseParent(std::shared_ptr<Node> &newNode, std::shared_ptr<Node> &nearestNode, std::vector<std::shared_ptr<Node>> &nearNodes);
    void reWire(std::shared_ptr<Node> &newNode, std::shared_ptr<Node> &nearestNode, std::vector<std::shared_ptr<Node>> &nearNodes);

private:
    std::mutex m_mutex;
};

} /* namespace rmpl */

#endif /* STARRRTPLANNER_H_ */
