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
    bool controlTrajectory(const std::shared_ptr<Node> &source, const std::shared_ptr<Node> &target, const float &stepSize);
    bool controlTrajectory(const Vec<float> &source, const Vec<float> &target, const float &stepSize);
    std::vector<Node> computeTrajectory(const std::shared_ptr<Node> &source, const std::shared_ptr<Node> &target, const float &stepSize);
    std::vector<Vec<float>> computeTrajectory(const Vec<float> &source, const Vec<float> &target, const float &stepSize);

private:
    TrajectoryMethod m_method;
    CollisionDetection *m_collision;
};

#endif /* TRAJECTORYPLANNER_H_ */
