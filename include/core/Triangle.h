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
#include <core/Vec.hpp>

namespace rmpl {

/*!
* \brief   Base class of Triangle, consists of three points and the manipulation of them
* \author  Sascha Kaden
* \date    2016-11-15
*/
class Triangle {
  public:
    Triangle();
    Triangle(Vec<float> p1, Vec<float> p2, Vec<float> p3);

    void setPoints(Vec<float> &pt1, Vec<float> &pt2, Vec<float> &pt3);
    void setP1(Vec<float> &pt);
    void setP2(Vec<float> &pt);
    void setP3(Vec<float> &pt);
    void getPoints(Vec<float> &pt1, Vec<float> &pt2, Vec<float> &pt3);
    Vec<float> getP1();
    Vec<float> getP2();
    Vec<float> getP3();

    bool empty();

    void transform(Eigen::MatrixXf rot, Eigen::VectorXf t);
    void transform(Vec<float> vec);
    void getBoundingBox(Vec<float> &p1, Vec<float> &p2);

  private:
    Vec<float> m_p1, m_p2, m_p3;
};

} /* namespace rmpl */

#endif    // TRIANGLE_H
