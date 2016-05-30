#ifndef TRAJECTORYPLANNER_H_
#define TRAJECTORYPLANNER_H_

#include <cstdint>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "CollisionDetection.h"
#include <core/Node.h>

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
class TrajectoryPlanner
{
public:
    TrajectoryPlanner(const TrajectoryMethod method, CollisionDetection *collision);
    bool computeTrajectory(const std::shared_ptr<Node> node1, const std::shared_ptr<Node> node2);

private:
    TrajectoryMethod m_method;
    CollisionDetection *m_collision;
};

#endif /* TRAJECTORYPLANNER_H_ */
