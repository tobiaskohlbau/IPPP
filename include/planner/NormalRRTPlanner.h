#ifndef NORMALRRTPLANNER_H_
#define NORMALRRTPLANNER_H_

#include "RRTPlanner.h"

using std::shared_ptr;

/*!
* \brief   Class of the NormalRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
class NormalRRTPlanner : public RRTPlanner
{
public:
    NormalRRTPlanner(const unsigned int &dim, float stepSize, TrajectoryMethod trajectory = TrajectoryMethod::linear, SamplingMethod sampling = SamplingMethod::randomly)
    : RRTPlanner(dim, stepSize, trajectory, sampling)
    {
    }

protected:
    void computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode);
};

#endif /* NORMALRRTPLANNER_H_ */
