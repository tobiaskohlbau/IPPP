#ifndef TRAJECTORYPLANNER_H_
#define TRAJECTORYPLANNER_H_

#include <core/Base.h>
#include <core/CollisionDetection.h>

namespace rmpl{

enum class TrajectoryMethod
{
    linear,
    spline,
};

/*!
* \brief   Class TrajectoryPlanner plans edges/paths to given nodes
* \author  Sascha Kaden
* \date    2016-05-25
*/
class TrajectoryPlanner : public Base
{
public:
    TrajectoryPlanner(const TrajectoryMethod &method, const float &stepSize, const std::shared_ptr<CollisionDetection> &collision);
    bool controlTrajectory(const Vec<float> &source, const Vec<float> &target);
    std::vector<Vec<float>> computeTrajectory(const Vec<float> &source, const Vec<float> &target);

private:
    TrajectoryMethod m_method;
    float m_stepSize;
    std::shared_ptr<CollisionDetection> m_collision;
};

} /* namespace rmpl */

#endif /* TRAJECTORYPLANNER_H_ */
