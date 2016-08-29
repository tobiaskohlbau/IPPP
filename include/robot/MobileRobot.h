//
// Created by sascha on 25.08.16.
//

#ifndef MOBILEROBOT_H_
#define MOBILEROBOT_H_

#include <robot/RobotBase.h>

namespace rmpl {

class MobileRobot : public RobotBase {
  public:
    MobileRobot(std::string name, CollisionType type, unsigned int dim, Vec<float> minBoundary, Vec<float> maxBoundary);
};

} /* namespace rmpl */

#endif    // MOBILEROBOT_H_
