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

#ifndef ROTATEATS_HPP
#define ROTATEATS_HPP

#include <ippp/core/trajectoryPlanner/TrajectoryPlanner.hpp>

namespace ippp {

/*!
* \brief   Class LinearTrajectory plans a path between the passed nodes/vecs. Start and end point aren't part of the path.
* \details The rotation in the path will be at the rotation point, percent value of the value.
 * The values of the rotation will be taken from the robot, which is saved in the environment.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class RotateAtS : public TrajectoryPlanner<dim> {
  public:
    RotateAtS(const std::shared_ptr<CollisionDetection<dim>> &collision,
              const std::shared_ptr<Environment> &environment, const double stepSize = 1, const double rotPoint = 0.5);

    std::vector<Vector<dim>> calcTrajectoryCont(const Vector<dim> &source, const Vector<dim> &target);
    std::vector<Vector<dim>> calcTrajectoryBin(const Vector<dim> &source, const Vector<dim> &target);

    void setRotationPoint(const double rotPoint);
    double getRotationPoint();

  private:
    void initMasks();

    double m_rotationPoint = 0.5;
    Vector<dim> m_posMask;
    Vector<dim> m_rotMask;

    using TrajectoryPlanner<dim>::m_collision;
    using TrajectoryPlanner<dim>::m_environment;
    using TrajectoryPlanner<dim>::m_stepSize;
    using TrajectoryPlanner<dim>::m_sqStepSize;
};

/*!
*  \brief      Constructor of the class LinearTrajectory
*  \author     Sascha Kaden
*  \param[in]  CollisionDetection
*  \param[in]  step size of the path
*  \param[in]  rotation point
*  \date       2017-06-20
*/
template <unsigned int dim>
RotateAtS<dim>::RotateAtS(const std::shared_ptr<CollisionDetection<dim>> &collision,
                          const std::shared_ptr<Environment> &environment, const double stepSize, const double rotPoint)
    : TrajectoryPlanner<dim>("RotateAtS", collision, environment, stepSize) {
    setRotationPoint(rotPoint);
    initMasks();
}

/*!
*  \brief      Compute the binary (section wise) trajectory between source and target. Return vector of points.
*  \details    The trajectory doesn't contain source or target vector.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[out] trajectory
*  \date       2017-06-20
*/
template <unsigned int dim>
std::vector<Vector<dim>> RotateAtS<dim>::calcTrajectoryBin(const Vector<dim> &source, const Vector<dim> &target) {
    // Todo: implementation of binary rotate at s trajectory
    return calcTrajectoryCont(source, target);
}

/*!
*  \brief      Compute the continuous trajectory between source and target. Return vector of points.
*  \details    The trajectory doesn't contain source or target vector.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[out] trajectory
*  \date       2017-06-20
*/
template <unsigned int dim>
std::vector<Vector<dim>> RotateAtS<dim>::calcTrajectoryCont(const Vector<dim> &source, const Vector<dim> &target) {
    std::vector<Vector<dim>> configs;

    Vector<dim> posSource = util::multiplyElementWise<dim>(source, m_posMask);
    Vector<dim> posTarget = util::multiplyElementWise<dim>(target, m_posMask);

    // compute the linear translation points
    Vector<dim> u(posTarget - posSource);    // u = a - b
    configs.reserve((int)(u.norm() / m_stepSize) + 1);
    u /= u.norm() / m_stepSize;    // u = |u|
    for (Vector<dim> temp(posSource + u); (temp - posTarget).squaredNorm() > 1; temp += u)
        configs.push_back(temp);

    // get middle point
    size_t middleIndex = configs.size() * m_rotationPoint;
    auto middlePos = configs[middleIndex];
    // add the fixed rotations to the linear path
    Vector<dim> rotSource = util::multiplyElementWise<dim>(source, m_rotMask);
    Vector<dim> rotTarget = util::multiplyElementWise<dim>(target, m_rotMask);
    size_t index = 0;
    for (auto config = configs.begin(); config != configs.end(); ++config, ++index) {
        if (index <= middleIndex)
            *config += rotSource;
        else
            *config += rotTarget;
    }

    // add rotation changing points to the vector
    rotSource += middlePos;
    rotTarget += middlePos;
    auto it = configs.begin() + middleIndex;
    u = (rotTarget - rotSource);    // u = a - b
    u /= u.norm() / m_stepSize;    // u = |u|
    for (Vector<dim> temp(rotSource + u); (temp - rotTarget).squaredNorm() > m_stepSize; temp += u)
        it = configs.insert(it, temp);

    return configs;
}

/*!
*  \brief      Sets to rotation point and checks if it is inside of the boundings.
*  \author     Sascha Kaden
*  \param[in]  rotation point
*  \date       2017-06-20
*/
template <unsigned int dim>
void RotateAtS<dim>::setRotationPoint(const double rotPoint) {
    if (rotPoint > 0 && 1 > rotPoint) {
        m_rotationPoint = rotPoint;
    } else {
        m_rotationPoint = 0.5;
        Logging::error("rotation point have to be between 0 and 1", this);
    }
}

/*!
*  \brief      Returns to rotation point.
*  \author     Sascha Kaden
*  \param[out] rotation point
*  \date       2017-06-20
*/
template <unsigned int dim>
double RotateAtS<dim>::getRotationPoint() {
    return m_rotationPoint;
}

/*!
*  \brief      Initialize the rotation and translations mask with the dofs of the robots.
*  \author     Sascha Kaden
*  \date       2017-06-20
*/
template <unsigned int dim>
void RotateAtS<dim>::initMasks() {
    size_t index = 0;
    for (size_t i = 0; i < m_environment->numRobots(); ++i) {
        auto dofTypes = m_environment->getRobot(i)->getDofTypes();
        for (auto dof = dofTypes.begin(); dof != dofTypes.end(); ++dof, ++index) {
            if (*dof == DofType::planarPos || *dof == volumetricPos) {
                m_posMask[index] = 1;
                m_rotMask[index] = 0;
            } else {
                m_posMask[index] = 0;
                m_rotMask[index] = 1;
            }
        }
    }
}

} /* namespace ippp */

#endif /* ROTATEATS_HPP */
