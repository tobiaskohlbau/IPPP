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

#ifndef COLLISIONDETECTION_HPP
#define COLLISIONDETECTION_HPP

#include <Eigen/Core>

#include <ippp/dataObj/Node.hpp>
#include <ippp/environment/Environment.h>
#include <ippp/modules/collisionDetection/CollisionRequest.h>
#include <ippp/modules/collisionDetection/CollisionResult.h>

namespace ippp {

/*!
* \brief   Abstract class CollisionDetection, base class of all CollisionDetections
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetection : public Identifier {
  public:
    CollisionDetection(const std::string &name, const std::shared_ptr<Environment> &environment,
                       const CollisionRequest &request = CollisionRequest());
    virtual bool checkConfig(const Vector<dim> &config, CollisionRequest *request = nullptr,
                             CollisionResult *result = nullptr) = 0;
    virtual bool checkTrajectory(std::vector<Vector<dim>> &config) = 0;

    void setRobotBoundings(const std::pair<Vector<dim>, Vector<dim>> &robotBoundings);
    bool checkRobotBounding(const Vector<dim> &config) const;

  protected:
    const std::shared_ptr<Environment> m_environment;    /*!< Pointer to the Environment */
    std::pair<Vector<dim>, Vector<dim>> m_robotBounding; /*!< Boundaries of the robot, fetched from the Environment */
    const CollisionRequest m_request;                    /*!< Default request for single collision tests (not trajectories) */
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetection<dim>::CollisionDetection(const std::string &name, const std::shared_ptr<Environment> &environment,
                                            const CollisionRequest &request)
    : Identifier(name), m_environment(environment), m_request(request) {
    Logging::debug("Initialize", this);
}

/*!
*  \brief      Sets the robot boundings of all robots, dimension should be the same.
*  \author     Sascha Kaden
*  \param[in]  pair of min and max boundary Vector
*  \date       2017-11-14
*/
template <unsigned int dim>
void CollisionDetection<dim>::setRobotBoundings(const std::pair<Vector<dim>, Vector<dim>> &robotBoundings) {
    m_robotBounding = robotBoundings;
}

/*!
*  \brief      Checks the boundaries of the robot to the passed configuration, return true if valid.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity of boundary check.
*  \date       2017-11-14
*/
template <unsigned int dim>
bool CollisionDetection<dim>::checkRobotBounding(const Vector<dim> &config) const {
    for (unsigned int i = 0; i < dim; ++i) {
        if (config[i] <= m_robotBounding.first[i] || m_robotBounding.second[i] <= config[i]) {
            Logging::trace("Robot out of robot boundary", this);
            return true;
        }
    }
    return false;
}

} /* namespace ippp */

#endif /* COLLISIONDETECTION_HPP */
