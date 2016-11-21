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

#include <core/Triangle.h>

#include <core/Logging.h>
#include <core/Utilities.h>

using namespace rmpl;

/*!
*  \brief      Standard constructor of the Triangle
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
Triangle::Triangle() {
}

/*!
*  \brief      Constructor of the Triangle
*  \param[in]  point 1
*  \param[in]  point 2
*  \param[in]  point 3
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
Triangle::Triangle(Vec<float> p1, Vec<float> p2, Vec<float> p3) {
    setPoints(p1, p2, p3);
}

/*!
*  \brief      Set points of the Triangle
*  \param[in]  point 1
*  \param[in]  point 2
*  \param[in]  point 3
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
void Triangle::setPoints(Vec<float> &p1, Vec<float> &p2, Vec<float> &p3) {
    if (p1.getDim() != p2.getDim() || p1.getDim() != p3.getDim()) {
        Logging::warning("Points have different dimensions", "Triangle");
        return;
    }
    m_p1 = p1;
    m_p2 = p2;
    m_p3 = p3;
}

/*!
*  \brief      Set point 1 of Triangle
*  \param[in]  point 1
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
void Triangle::setP1(Vec<float> &pt) {
    m_p1 = pt;
}

/*!
*  \brief      Set point 2 of Triangle
*  \param[in]  point 2
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
void Triangle::setP2(Vec<float> &pt) {
    m_p2 = pt;
}

/*!
*  \brief      Set point 3 of Triangle
*  \param[in]  point 3
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
void Triangle::setP3(Vec<float> &pt) {
    m_p3 = pt;
}

/*!
*  \brief      Return points of the Triangle
*  \param[out] point 1
*  \param[out] point 2
*  \param[out] point 3
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
void Triangle::getPoints(Vec<float> &p1, Vec<float> &p2, Vec<float> &p3) {
    p1 = m_p1;
    p2 = m_p2;
    p3 = m_p3;
}

/*!
*  \brief      Return point 1 of Triangle
*  \param[out] point 1
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
Vec<float> Triangle::getP1() {
    return m_p1;
}

/*!
*  \brief      Return point 2 of Triangle
*  \param[out] point 2
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
Vec<float> Triangle::getP2() {
    return m_p2;
}

/*!
*  \brief      Return point 3 of Triangle
*  \param[out] point 3
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
Vec<float> Triangle::getP3() {
    return m_p3;
}

/*!
*  \brief      Return true if one point of the triangle is empty
*  \param[out] binary result
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
bool Triangle::empty() {
    if (m_p1.empty())
        return true;
    else if (m_p2.empty())
        return true;
    else if (m_p3.empty())
        return true;
    else
        return false;
}

/*!
*  \brief      Transform Triangle by passed rotation matrix and translation vector
*  \param[in]  rotation matrix R
*  \param[in]  translation vector t
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
bool Triangle::transform(Eigen::MatrixXf R, Eigen::VectorXf t) {
    if (R.cols() != m_p1.getDim() || t.rows() != m_p1.getDim()) {
        Logging::warning("Different dimensions transformation to Triangle", "Triangle");
        return false;
    }

    Eigen::VectorXf p1 = Utilities::VecToEigen(m_p1);
    Eigen::VectorXf p2 = Utilities::VecToEigen(m_p2);
    Eigen::VectorXf p3 = Utilities::VecToEigen(m_p3);

    p1 = R * p1 + t;
    p2 = R * p2 + t;
    p3 = R * p3 + t;

    m_p1 = Utilities::EigenToVec(p1);
    m_p2 = Utilities::EigenToVec(p2);
    m_p3 = Utilities::EigenToVec(p3);
    return true;
}

/*!
*  \brief      Transform Triangle by passed pose vector (2D and 3D)
*  \param[in]  pose vector with translation and rotation
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
bool Triangle::transform(Vec<float> vec) {
    if (vec.getDim() == 3) {
        assert(m_p1.getDim() == 2);

        Eigen::Matrix2f R;
        Eigen::Vector2f t;
        Utilities::poseVecToRandT(vec, R, t);
        transform(R, t);
    } else if (vec.getDim() == 6) {
        assert(m_p1.getDim() == 3);

        Eigen::Matrix3f R;
        Eigen::Vector3f t;
        Utilities::poseVecToRandT(vec, R, t);
        transform(R, t);
    } else {
        Logging::error("Pose dim is not compatible to triangle dim", "Triangle");
    }
}

/*!
*  \brief      Return bounding box of Triangle by minimal and maximum point
*  \param[out] minimum point
*  \param[out] maximum point
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
void Triangle::getBoundingBox(Vec<float> &min, Vec<float> &max) {
    min = Vec<float>(m_p1.getDim());
    max = Vec<float>(m_p1.getDim());
    for (unsigned int i = 0; i < m_p1.getDim(); ++i) {
        min[i] = std::min(std::min(m_p1[i], m_p2[i]), m_p3[i]);
        max[i] = std::max(std::max(m_p1[i], m_p2[i]), m_p3[i]);
    }
}
