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
template <class T, unsigned int P>
class PointList {
  public:
    PointList() = default;

    template <class... Args>
    PointList(unsigned int index, T pt) {
        assert(index < P);
        m_p[index] = pt;
    }

    template <class... Args>
    PointList(T pt, Args... fargs) : PointList(1, fargs...) {
        m_p[0] = pt;
    }

    template <class... Args>
    PointList(unsigned int index, T pt, Args... fargs) : PointList(index + 1, fargs...) {
        assert(index < P);
        m_p[index] = pt;
    }

    void setP(T p, unsigned int index) {
        assert(index > 0);
        assert(index <= P);
        m_p[index - 1] = p;
    }

    T getP(unsigned int index) {
        assert(index > 0);
        assert(index <= P);
        return m_p[index - 1];
    }

    /*!
    *  \brief      Transform Triangle by passed rotation matrix and translation vector
    *  \param[in]  rotation matrix R
    *  \param[in]  translation vector t
    *  \author     Sascha Kaden
    *  \date       2016-11-15
    */
    void transform(const Eigen::MatrixXf R, const T t) {
        assert(R.rows() == m_p[0].rows());

        for (unsigned int i = 0; i < P; ++i)
            m_p[i] = R * m_p[i] + t;
    }

  private:
    T m_p[P];
};

typedef PointList<Eigen::Vector2f, 2> Line2D;
typedef PointList<Eigen::Vector3f, 2> Line3D;
typedef PointList<Eigen::Vector2f, 3> Triangle2D;
typedef PointList<Eigen::Vector3f, 3> Triangle3D;
typedef PointList<Eigen::Vector2f, 4> Square2D;
typedef PointList<Eigen::Vector3f, 4> Square3D;
typedef PointList<Eigen::Vector2f, 5> Pentagon2D;
typedef PointList<Eigen::Vector3f, 5> Pentagon3D;
typedef PointList<Eigen::Vector2f, 6> Hexagon2D;
typedef PointList<Eigen::Vector3f, 6> Hexagon3D;
typedef PointList<Eigen::Vector2f, 7> Heptagon2D;
typedef PointList<Eigen::Vector3f, 7> Heptagon3D;

} /* namespace rmpl */

#endif    // TRIANGLE_H
