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

#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <Eigen/Core>
#include <core/Logging.h>
#include <core/Utilities.h>

namespace rmpl {

/*!
* \brief   Base class of Triangle, consists of three points and the manipulation of them
* \author  Sascha Kaden
* \date    2016-11-15
*/
template <typename T>
class Triangle {
  public:
    Triangle();
    Triangle(T p1, T p2, T p3);

    void setPoints(T &pt1, T &pt2, T &pt3);
    void setP1(T &pt);
    void setP2(T &pt);
    void setP3(T &pt);
    void getPoints(T &pt1, T &pt2, T &pt3);
    T getP1();
    T getP2();
    T getP3();

    void transform(const Eigen::MatrixXf R, const T t);
    void getBoundingBox(T &p1, T &p2);

  private:
    T m_p1, m_p2, m_p3;
};

/*!
*  \brief      Standard constructor of the Triangle
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
template <typename T>
Triangle<T>::Triangle() {
}

/*!
*  \brief      Constructor of the Triangle
*  \param[in]  point 1
*  \param[in]  point 2
*  \param[in]  point 3
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
template <typename T>
Triangle<T>::Triangle(T p1, T p2, T p3) {
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
template <typename T>
void Triangle<T>::setPoints(T &p1, T &p2, T &p3) {
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
template <typename T>
void Triangle<T>::setP1(T &pt) {
    m_p1 = pt;
}

/*!
*  \brief      Set point 2 of Triangle
*  \param[in]  point 2
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
template <typename T>
void Triangle<T>::setP2(T &pt) {
    m_p2 = pt;
}

/*!
*  \brief      Set point 3 of Triangle
*  \param[in]  point 3
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
template <typename T>
void Triangle<T>::setP3(T &pt) {
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
template <typename T>
void Triangle<T>::getPoints(T &p1, T &p2, T &p3) {
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
template <typename T>
T Triangle<T>::getP1() {
    return m_p1;
}

/*!
*  \brief      Return point 2 of Triangle
*  \param[out] point 2
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
template <typename T>
T Triangle<T>::getP2() {
    return m_p2;
}

/*!
*  \brief      Return point 3 of Triangle
*  \param[out] point 3
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
template <typename T>
T Triangle<T>::getP3() {
    return m_p3;
}

/*!
*  \brief      Transform Triangle by passed rotation matrix and translation vector
*  \param[in]  rotation matrix R
*  \param[in]  translation vector t
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
template <typename T>
void Triangle<T>::transform(const Eigen::MatrixXf R, const T t) {
    assert(R.cols() == m_p1.cols());

    m_p1 = R * m_p1 + t;
    m_p2 = R * m_p2 + t;
    m_p3 = R * m_p3 + t;
}

/*!
*  \brief      Return bounding box of Triangle by minimal and maximum point
*  \param[out] minimum point
*  \param[out] maximum point
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
template <typename T>
void Triangle<T>::getBoundingBox(T &min, T &max) {
    for (unsigned int i = 0; i < m_p1.getDim(); ++i) {
        min[i] = std::min(std::min(m_p1[i], m_p2[i]), m_p3[i]);
        max[i] = std::max(std::max(m_p1[i], m_p2[i]), m_p3[i]);
    }
}

} /* namespace rmpl */

#endif    // TRIANGLE_H
