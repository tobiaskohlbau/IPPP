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

Triangle::Triangle() {
}

Triangle::Triangle(Vec<float> p1, Vec<float> p2, Vec<float> p3) {
    if (p1.getDim() != p2.getDim() || p1.getDim() != p3.getDim()) {
        Logging::warning("Points have different dimensions", "Triangle");
        return;
    }
    m_p1 = p1;
    m_p2 = p2;
    m_p3 = p3;
}

void Triangle::setP1(Vec<float> &pt) {
    m_p1 = pt;
}

void Triangle::setP2(Vec<float> &pt) {
    m_p2 = pt;
}

void Triangle::setP3(Vec<float> &pt) {
    m_p3 = pt;
}

Vec<float> Triangle::getP1() {
    return m_p1;
}

Vec<float> Triangle::getP2() {
    return m_p2;
}

Vec<float> Triangle::getP3() {
    return m_p3;
}

void Triangle::transform(Eigen::MatrixXf rot, Eigen::VectorXf t) {
    // control dimensions
    if (rot.cols() != m_p1.getDim() || t.rows() != m_p1.getDim()) {
        Logging::warning("Different dimensions by transformation", "Triangle");
        return;
    }

    Eigen::VectorXf p1 = Utilities::VecToEigen(m_p1);
    Eigen::VectorXf p2 = Utilities::VecToEigen(m_p2);
    Eigen::VectorXf p3 = Utilities::VecToEigen(m_p3);

    p1 = rot * p1 + t;
    p2 = rot * p2 + t;
    p3 = rot * p3 + t;

    m_p1 = Utilities::EigenToVec(p1);
    m_p2 = Utilities::EigenToVec(p2);
    m_p3 = Utilities::EigenToVec(p3);
}

void Triangle::transform(Vec<float> vec) {
    assert(vec.getDim() == m_p1.getDim()+1);

    Eigen::Matrix2f rot;
    rot(0,0) = std::cos(Utilities::degToRad(vec[2]));
    rot(1,0) = std::sin(Utilities::degToRad(vec[2]));
    rot(0,1) = -std::sin(Utilities::degToRad(vec[2]));
    rot(1,1) = std::cos(Utilities::degToRad(vec[2]));
    Eigen::Vector2f t;
    t(0) = vec[0];
    t(1) = vec[1];
    transform(rot, t);
}

void Triangle::getBoundingBox(Vec<float> &min, Vec<float> &max) {
    min = Vec<float>(m_p1.getDim());
    max = Vec<float>(m_p1.getDim());
    for (unsigned int i = 0; i < m_p1.getDim(); ++i) {
        min[i] = std::min(std::min(m_p1[i], m_p2[i]), m_p3[i]);
        max[i] = std::max(std::max(m_p1[i], m_p2[i]), m_p3[i]);
    }
}

