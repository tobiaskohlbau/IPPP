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

#ifndef POINTLIST_H
#define POINTLIST_H

#include <Eigen/Core>
#include <core/utility/Logging.h>
#include <core/utility/Utility.h>

namespace rmpl {

/*!
* \brief   Data structure for 2D or 3D point list
* \details Typedefs has to be used otherwise the variadic template constructor will not be working.
*          Number of input points have to be the size of the PointList
* \author  Sascha Kaden
* \date    2016-12-19
*/
template <class T, unsigned int P>
class PointList {
  public:
    PointList() = default;

    /*!
    *  \brief      Last constructor of PointList
    *  \param[in]  index
    *  \param[in]  point
    *  \author     Sascha Kaden
    *  \date       2016-12-19
    */
    template <class... Args>
    PointList(unsigned int index, T pt) {
        assert(index == P - 1);
        m_p[index] = pt;
    }

    /*!
    *  \brief      First constructor of PointList
    *  \param[in]  point
    *  \param[in]  remaining list of points
    *  \author     Sascha Kaden
    *  \date       2016-12-19
    */
    template <class... Args>
    PointList(T pt, Args... fargs) : PointList(1, fargs...) {
        m_p[0] = pt;
    }

    /*!
    *  \brief      Second constructor of PointList
    *  \param[in]  index
    *  \param[in]  point
    *  \param[in]  remaining list of points
    *  \author     Sascha Kaden
    *  \date       2016-12-19
    */
    template <class... Args>
    PointList(unsigned int index, T pt, Args... fargs) : PointList(index + 1, fargs...) {
        assert(index < P - 1);
        m_p[index] = pt;
    }

    /*!
    *  \brief      Set point of PointList from index (index starts with 1)
    *  \param[in]  index
    *  \param[out] point
    *  \author     Sascha Kaden
    *  \date       2016-12-19
    */
    void setP(T p, unsigned int index) {
        assert(index > 0);
        assert(index <= P);
        m_p[index - 1] = p;
    }

    /*!
    *  \brief      Return point of PointList from index
    *  \param[in]  index
    *  \param[out] point
    *  \author     Sascha Kaden
    *  \date       2016-12-19
    */
    T getP(unsigned int index) {
        assert(index > 0);
        assert(index <= P);
        return m_p[index - 1];
    }

    /*!
    *  \brief      Transform Pointlist itself by passed rotation matrix and translation vector
    *  \param[in]  rotation matrix R
    *  \param[in]  translation vector t
    *  \author     Sascha Kaden
    *  \date       2016-12-19
    */
    void transform(const MatrixX &R, const T &t) {
        assert(R.rows() == m_p[0].rows());

        for (unsigned int i = 0; i < P; ++i)
            m_p[i] = R * m_p[i] + t;
    }

  private:
    T m_p[P];
};

typedef PointList<Vector2, 2> Line2D;
typedef PointList<Vector3, 2> Line3D;
typedef PointList<Vector2, 3> Triangle2D;
typedef PointList<Vector3, 3> Triangle3D;
typedef PointList<Vector2, 4> Square2D;
typedef PointList<Vector3, 4> Square3D;
typedef PointList<Vector2, 5> Pentagon2D;
typedef PointList<Vector3, 5> Pentagon3D;
typedef PointList<Vector2, 6> Hexagon2D;
typedef PointList<Vector3, 6> Hexagon3D;
typedef PointList<Vector2, 7> Heptagon2D;
typedef PointList<Vector3, 7> Heptagon3D;

} /* namespace rmpl */

#endif    // POINTLIST_H
