#ifndef NORMALRRTPLANNER_H_
#define NORMALRRTPLANNER_H_

#include <mutex>

#include "RRTPlanner.h"

namespace rmpl {

/*!
* \brief   Class of the NormalRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class NormalRRTPlanner : public RRTPlanner
{
public:
    NormalRRTPlanner(const std::shared_ptr<RobotBase> &robot, float stepSize, float trajectoryStepSize, TrajectoryMethod trajectory = TrajectoryMethod::linear, SamplingMethod sampling = SamplingMethod::randomly)
    : RRTPlanner("Normal RRT Planner", robot, stepSize, trajectoryStepSize, trajectory, sampling)
    {
    }

protected:
    void computeRRTNode(const Vec<float> &randVec, std::shared_ptr<Node> &newNode);

private:
    std::mutex m_mutex;
};

} /* namespace rmpl */

#endif /* NORMALRRTPLANNER_H_ */
