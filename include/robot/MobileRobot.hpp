//
// Created by sascha on 25.08.16.
//

#ifndef MOBILEROBOT_H_
#define MOBILEROBOT_H_

#include <robot/RobotBase.hpp>

namespace rmpl {

template <unsigned int dim>
class MobileRobot : public RobotBase<dim> {
  public:
    MobileRobot(std::string name, CollisionType type, const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary);
};

template <unsigned int dim>
MobileRobot<dim>::MobileRobot(std::string name, CollisionType type, const Vector<dim> &minBoundary,
                              const Vector<dim> &maxBoundary)
    : RobotBase<dim>(name, type, RobotType::mobile) {
    for (unsigned int i = 0; i < dim; ++i) {
        if (minBoundary[i] > maxBoundary[i]) {
            Logging::warning("Min boundary is larger than max boundary!", this);
            return;
        }
    }

    this->m_maxBoundary = maxBoundary;
    this->m_minBoundary = minBoundary;
}

} /* namespace rmpl */

#endif    // MOBILEROBOT_H_
