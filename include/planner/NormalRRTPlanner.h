#ifndef NORMALRRTPLANNER_H_
#define NORMALRRTPLANNER_H_

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
    NormalRRTPlanner(const unsigned int &dim, const float &stepSize, TrajectoryMethod trajectory = TrajectoryMethod::linear, SamplingMethod sampling = SamplingMethod::randomly)
    : RRTPlanner("Normal RRT Planner", dim, stepSize, trajectory, sampling)
    {
    }

protected:
    void computeRRTNode(const Vec<float> &randVec, std::shared_ptr<Node> &newNode);
};

} /* namespace rmpl */

#endif /* NORMALRRTPLANNER_H_ */
