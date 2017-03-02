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

#ifndef COLLISIONDETECTIONPQPBENCHMARK_HPP
#define COLLISIONDETECTIONPQPBENCHMARK_HPP

#include <core/module/collisionDetection/CollisionDetectionPqp.hpp>

#include <mutex>

namespace rmpl {

/*!
* \brief   Class collision detection with the pqp library and extra benchmark methods.
* \author  Sascha Kaden
* \date    2017-02-27
*/
template <unsigned int dim>
class CollisionDetectionPqpBenchmark : public CollisionDetectionPqp<dim> {
  public:
    CollisionDetectionPqpBenchmark(const std::shared_ptr<RobotBase<dim>> &robot);
    bool controlVec(const Vector<dim> &vec) override;
    bool controlTrajectory(std::vector<Vector<dim>> &vec) override;

    int getCount() const;
    void resetCount();

  private:
    unsigned int m_count = 0;
    std::mutex m_mutex;
};

/*!
*  \brief      Standard constructor of the class CollisionDetectionPqpBenchmark
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2017-02-27
*/
template <unsigned int dim>
CollisionDetectionPqpBenchmark<dim>::CollisionDetectionPqpBenchmark(const std::shared_ptr<RobotBase<dim>> &robot)
    : CollisionDetectionPqp<dim>(robot) {
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-27
*/
template <unsigned int dim>
bool CollisionDetectionPqpBenchmark<dim>::controlVec(const Vector<dim> &vec) {
    m_mutex.lock();
    ++m_count;
    m_mutex.unlock();
    return CollisionDetectionPqp<dim>::controlVec(vec);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionPqpBenchmark<dim>::controlTrajectory(std::vector<Vector<dim>> &vecs) {
    if (vecs.size() == 0)
        return false;

    if (this->m_robot->getRobotType() == RobotType::mobile) {
        for (int i = 0; i < vecs.size(); ++i) {
            m_mutex.lock();
            ++m_count;
            m_mutex.unlock();
            if (CollisionDetectionPqp<dim>::checkMobileRobot(vecs[i])) {
                return true;
            }
        }
    } else {
        for (int i = 0; i < vecs.size(); ++i) {
            m_mutex.lock();
            ++m_count;
            m_mutex.unlock();
            if (CollisionDetectionPqp<dim>::checkSerialRobot(vecs[i])) {
                return true;
            }
        }
    }

    return false;
}

/*!
*  \brief      Return count of collision detection calls
*  \author     Sascha Kaden
*  \param[out] count
*  \date       2017-02-27
*/
template <unsigned int dim>
int CollisionDetectionPqpBenchmark<dim>::getCount() const {
    return m_count;
}

/*!
*  \brief      Reset collision detection count
*  \author     Sascha Kaden
*  \date       2017-02-27
*/
template <unsigned int dim>
void CollisionDetectionPqpBenchmark<dim>::resetCount() {
    m_count = 0;
}

} /* namespace rmpl */

#endif /* COLLISIONDETECTIONPQPBENCHMARK_HPP */
