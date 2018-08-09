//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#ifndef UTILVEC_HPP
#define UTILVEC_HPP

#include <vector>

#include <Eigen/Core>

#include <ippp/types.h>

namespace ippp {
namespace util {

bool empty(const Vector3 &vec);
Vector5 Vecd(double x, double y, double z, double rx, double ry);
Vector6 Vecd(double x, double y, double z, double rx, double ry, double rz);
Vector7 Vecd(double a1, double a2, double a3, double a4, double a5, double a6, double a7);
VectorX Vecd(unsigned int dim, double data[]);

/*!
*  \brief      Check whether the vector is empty (contains a NaN value)
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \param[out] result, true if vector is empty
*  \date       2017-04-04
*/
template <unsigned int dim>
bool empty(const Vector<dim> &vec) {
    bool status = false;
    for (unsigned int i = 0; i < dim; ++i) {
        if (std::isnan(vec[i])) {
            status = true;
            break;
        }
    }
    return status;
}

/*!
*  \brief      Appends the second Vector to the first one and returns the result Vector.
*  \author     Sascha Kaden
*  \param[in]  first Vector
*  \param[in]  second Vector
*  \param[out] result Vector
*  \date       2016-12-23
*/
template <unsigned int dim1, unsigned int dim2>
Vector<dim1 + dim2> append(const Vector<dim1> &first, const Vector<dim2> &second) {
    Vector<dim1 + dim2> vec;
    for (unsigned int i = 0; i < dim1; ++i)
        vec[i] = first[i];

    for (unsigned int i = 0; i < dim2; ++i)
        vec[dim1 + i] = second[i];

    return vec;
}

/*!
*  \brief      Appends the value to the first one and returns the result Vector.
*  \author     Sascha Kaden
*  \param[in]  first Vector
*  \param[in]  value
*  \param[out] result Vector
*  \date       2016-12-23
*/
template <unsigned int dim>
Vector<dim + 1> append(const Vector<dim> &source, double add) {
    Vector<dim + 1> vec;
    for (unsigned int i = 0; i < dim; ++i)
        vec[i] = source[i];

    vec[dim] = add;
    return vec;
}

/*!
*  \brief      Multiplicate two vectors and return the result.
*  \author     Sascha Kaden
*  \param[in]  first Vector
*  \param[in]  second vector
*  \param[out] result Vector
*  \date       2017-06-20
*/
template <unsigned int dim>
Vector<dim> multiplyElementWise(const Vector<dim> &source, const Vector<dim> &mask) {
    Vector<dim> result(source);
    for (unsigned int i = 0; i < dim; ++i)
        result[i] *= mask[i];
    return result;
}

/*!
*  \brief      Splits the Vector in separate Vectors with the sizes of the passed list.
*  \author     Sascha Kaden
*  \param[in]  main Vector
*  \param[in]  splitting sequence
*  \param[out] separate result Vectors
*  \date       2017-06-20
*/
template <unsigned int dim>
std::vector<VectorX> splitVec(const Vector<dim> vec, const std::vector<unsigned int> &dofs) {
    std::vector<VectorX> vecs;
    vecs.reserve(dofs.size());
    unsigned int count = 0;
    for (auto &dof : dofs) {
        VectorX temp(dof);
        for (unsigned int i = 0; i < dof; ++i, ++count) {
            temp[i] = vec[count];
        }
        vecs.push_back(temp);
    }
    return vecs;
}

/*!
*  \brief      Creates a Vector with template dimension and all elements are set to the passed value
*  \author     Sascha Kaden
*  \param[in]  value
*  \param[out] result Vector
*  \date       2016-12-23
*/
template <unsigned int dim>
Vector<dim> Vecd(double data) {
    Vector<dim> vec;
    for (unsigned int i = 0; i < dim; ++i) {
        vec[i] = data;
    }
    return vec;
}

/*!
*  \brief      Creates a Vector with passed array (dimension by template parameter)
*  \author     Sascha Kaden
*  \param[in]  array
*  \param[out] result Vector
*  \date       2016-12-23
*/
template <unsigned int dim>
Vector<dim> Vecd(double data[]) {
    Vector<dim> vec;
    for (unsigned int i = 0; i < dim; ++i) {
        vec[i] = data[i];
    }
    return vec;
}

/*!
*  \brief      Creates a NaN Vector
*  \author     Sascha Kaden
*  \param[out] result NaN Vector
*  \date       2017-04-04
*/
template <unsigned int dim>
Vector<dim> NaNVector() {
    Vector<dim> vec;
    for (unsigned int i = 0; i < dim; ++i) {
        vec[i] = std::nanf("1");
    }
    return vec;
}

/*!
*  \brief      Checks the equality of two n-dimensional Vector.
*  \detail     Using of Epsilon parameter from types.h for checking.
*  \author     Sascha Kaden
*  \param[in]  first n-dimensional vector
*  \param[in]  second n-dimensional vector
*  \param[out] true if equal
*  \date       2017-04-04
*/
template <unsigned int dim>
bool equal(const Vector<dim> &vec1, const Vector<dim> &vec2) {
    return (vec1 - vec2).isZero(IPPP_EPSILON);
}

} /* namespace util */
} /* namespace ippp */

#endif    // UTILVEC_HPP
