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

#include <core/Utilities.h>

using namespace rmpl;

/*!
*  \brief         Decompose transformation matrix T in rotation R and translation t
*  \author        Sascha Kaden
*  \param[in]     transformation matrix
*  \param[in,out] rotation matrix
*  \param[in,out] translatin matrix
*  \date          2016-08-25
*/
void Utilities::decomposeT(Eigen::Matrix4f &T, Eigen::Matrix3f &R, Eigen::Vector3f &t) {
    R = T.block<3, 3>(0, 0);
    t = T.block<3, 1>(0, 3);
}

/*!
*  \brief      Convert deg angles to rad
*  \author     Sascha Kaden
*  \param[in]  Vec of deg angles
*  \param[out] Vec of rad angles
*  \date       2016-07-07
*/
Vec<float> Utilities::degToRad(const Vec<float> deg) {
    Vec<float> rad(deg.getDim());
    for (unsigned int i = 0; i < deg.getDim(); ++i)
        rad[i] = deg[i] / 360 * pi();
    return rad;
}


/*!
*  \brief      Convert rmpl Vec to Eigen Array
*  \author     Sascha Kaden
*  \param[in]  Vec
*  \param[out] Eigen Array
*  \date       2016-07-07
*/
Eigen::ArrayXf Utilities::VecToEigen(const Vec<float> &vec) {
    Eigen::ArrayXf eigenVec(vec.getDim());
    for (unsigned int i = 0; i < vec.getDim(); ++i)
        eigenVec(i, 0) = vec[i];
    return eigenVec;
}

/*!
*  \brief      Convert Eigen Array to rmpl Vec
*  \author     Sascha Kaden
*  \param[in]  Eigen Array
*  \param[out] Vec
*  \date       2016-07-07
*/
Vec<float> Utilities::EigenToVec(const Eigen::ArrayXf &eigenVec) {
    Vec<float> vec((unsigned int)eigenVec.rows());
    for (unsigned int i = 0; i < vec.getDim(); ++i)
        vec[i] = eigenVec(i, 0);
    return vec;
}