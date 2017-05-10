//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
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

#include <chrono>
#include <mutex>

namespace ippp {

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

    void resetComputationTimes();
    std::chrono::duration<double> getMeanComputationTime() const;

  private:
    unsigned int m_count = 0;
    std::vector<std::chrono::duration<double>> m_computationTimes;
    std::mutex m_mutexCount;
    std::mutex m_mutexTime;
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
    m_mutexCount.lock();
    ++m_count;
    m_mutexCount.unlock();
    auto startTime = std::chrono::system_clock::now();
    bool result = CollisionDetectionPqp<dim>::controlVec(vec);
    m_mutexTime.lock();
    m_computationTimes.push_back(std::chrono::system_clock::now() - startTime);
    m_mutexTime.unlock();
    return result;
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
            m_mutexCount.lock();
            ++m_count;
            m_mutexCount.unlock();
            if (CollisionDetectionPqp<dim>::checkMobileRobot(vecs[i])) {
                return true;
            }
        }
    } else {
        for (int i = 0; i < vecs.size(); ++i) {
            m_mutexCount.lock();
            ++m_count;
            m_mutexCount.unlock();
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

/*!
*  \brief      Reset collision detection computation times
*  \author     Sascha Kaden
*  \date       2017-03-15
*/
template <unsigned int dim>
void CollisionDetectionPqpBenchmark<dim>::resetComputationTimes() {
    m_computationTimes.clear();
}

/*!
*  \brief      Return the mean time of all computations
*  \author     Sascha Kaden
*  \param[out] mean computation time
*  \date       2017-03-1
*/
template <unsigned int dim>
std::chrono::duration<double> CollisionDetectionPqpBenchmark<dim>::getMeanComputationTime() const {
    std::chrono::duration<double> duration;
    if (m_computationTimes.empty())
        return duration;

    duration = m_computationTimes[0];
    for (int i = 1; i < m_computationTimes.size(); ++i) {
        duration += m_computationTimes[i];
    }
    duration /= (double)m_computationTimes.size();
    return duration;
}

} /* namespace ippp */

#endif /* COLLISIONDETECTIONPQPBENCHMARK_HPP */
