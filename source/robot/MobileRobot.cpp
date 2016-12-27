//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

#include <robot/MobileRobot.h>

#include <core/utility/Logging.h>
#include <core/utility/Utility.h>

namespace rmpl {

MobileRobot::MobileRobot(std::string name, CollisionType type, unsigned int dim, const Eigen::VectorXf &minBoundary,
                         const Eigen::VectorXf &maxBoundary)
    : RobotBase(name, type, RobotType::mobile, dim) {
    if (utilVec::empty(minBoundary) || utilVec::empty(maxBoundary)) {
        Logging::error("Boundaries are empty", this);
        return;
    } else if (minBoundary.rows() != m_dim || maxBoundary.rows() != m_dim) {
        Logging::error("Boudaries have different dimensions from the robot!", this);
        return;
    }

    for (unsigned int i = 0; i < m_dim; ++i) {
        if (minBoundary[i] > maxBoundary[i]) {
            Logging::warning("Min boundary is larger than max boundary!", this);
            return;
        }
    }

    m_maxBoundary = maxBoundary;
    m_minBoundary = minBoundary;
}

} /* namespace rmpl */
