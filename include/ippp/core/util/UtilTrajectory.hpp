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

#ifndef UTILTRAJECTORY_HPP
#define UTILTRAJECTORY_HPP

#include <ippp/core/util/UtilGeo.hpp>

namespace ippp {
namespace util {

/*!
*  \brief      Compute the continuous trajectory between source and target. Return vector of points.
*  \details    The trajectory doesn't contain source or target configuration.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[in]  positional resolution
*  \param[in]  rotational resolution
*  \param[out] trajectory
*  \date       2017-09-30
*/
template <unsigned int dim>
static std::vector<Vector<dim>> linearTrajectoryCont(const Vector<dim> &source, const Vector<dim> &target, const double res) {
    std::vector<Vector<dim>> configs;

    Vector<dim> u(target - source);    // u = a - b
    configs.reserve((int)(u.norm() / res) + 1);
    u /= u.norm() / res;    // u = |u|
    for (Vector<dim> temp(source + u); (temp - target).squaredNorm() > 1; temp += u)
        configs.push_back(temp);
    return configs;
}

/*!
*  \brief      Compute the continuous trajectory between source and target. Return vector of points.
*  \details    The trajectory doesn't contain source or target configuration.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[in]  positional resolution
*  \param[in]  rotational resolution
*  \param[in]  position mask
*  \param[in]  rotation mask
*  \param[out] trajectory
*  \date       2017-09-30
*/
template <unsigned int dim>
static std::vector<Vector<dim>> linearTrajectoryCont(const Vector<dim> &source, const Vector<dim> &target, const double posRes,
                                                     const double oriRes, const Vector<dim> &posMask,
                                                     const Vector<dim> &oriMask) {
    double uNormPos = (util::multiplyElementWise<dim>((target - source), posMask)).squaredNorm();
    double uNormOri = (util::multiplyElementWise<dim>((target - source), oriMask)).squaredNorm();

    // check resolutions of position and orientation resolution and take the smaller one
    auto oRes = uNormOri / oriRes;
    if (oRes > 0 && oRes < uNormPos / posRes)
        return linearTrajectoryCont<dim>(source, target, oriRes);
    else
        return linearTrajectoryCont<dim>(source, target, posRes);
}

/*!
*  \brief      Compute the continuous trajectory between source and target. Return vector of points.
*  \details    The trajectory doesn't contain source or target configuration.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[in]  resolution
*  \param[out] trajectory
*  \date       2017-09-30
*/
template <unsigned int dim>
static std::vector<Vector<dim>> linearTrajectoryBin(const Vector<dim> &source, const Vector<dim> &target, const double res) {
    std::vector<Vector<dim>> configs;

    Vector<dim> u(target - source);
    configs.reserve((int)(u.norm() / res) + 1);
    double sqRes = res * res;
    unsigned int divider = 2;
    for (Vector<dim> uTemp(u / divider); uTemp.squaredNorm() > sqRes; divider *= 2, uTemp = u / divider)
        for (unsigned int i = 1; i < divider; i += 2)
            configs.push_back(source + (uTemp * i));

    return configs;
}

/*!
*  \brief      Compute the continuous trajectory between source and target. Return vector of points.
*  \details    The trajectory doesn't contain source or target configuration.
*  \author     Sascha Kaden
*  \param[in]  source Vector
*  \param[in]  target Vector
*  \param[in]  positional resolution
*  \param[in]  rotational resolution
*  \param[in]  position mask
*  \param[in]  rotation mask
*  \param[out] trajectory
*  \date       2017-09-30
*/
template <unsigned int dim>
static std::vector<Vector<dim>> linearTrajectoryBin(const Vector<dim> &source, const Vector<dim> &target, const double posRes,
                                                    const double oriRes, const Vector<dim> &posMask, const Vector<dim> &oriMask) {
    double uNormPos = (util::multiplyElementWise<dim>((target - source), posMask)).squaredNorm();
    double uNormOri = (util::multiplyElementWise<dim>((target - source), oriMask)).squaredNorm();

    // check resolutions of position and orientation resolution and take the smaller one
    auto oRes = uNormOri / oriRes;
    if (oRes > 0 && oRes < uNormPos / posRes)
        return linearTrajectoryBin<dim>(source, target, oriRes);
    else
        return linearTrajectoryBin<dim>(source, target, posRes);
}

} /* namespace util */
} /* namespace ippp */

#endif    // UTILTRAJECTORY_HPP
