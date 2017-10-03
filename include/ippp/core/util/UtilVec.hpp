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

#ifndef UTILVEC_HPP
#define UTILVEC_HPP

#include <Eigen/Core>

#include <ippp/core/types.h>

namespace ippp {
namespace util {

/*!
*  \brief      Check equality of two Vectors
*  \author     Sascha Kaden
*  \param[in]  Vector a
*  \param[in]  Vector b
*  \param[out] result, true if vectors are equal
*  \date       2017-09-30
*/

template <unsigned int dim>
static bool equal(const Vector<dim> &a, const Vector<dim> &b) {
    for (unsigned int i = 0; i < dim; ++i)
        if (a[i] - b[i] > 0.00001)
            return false;

    return true;
}

/*!
*  \brief      Check whether the vector is empty (contains a NaN value)
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \param[out] result, true if vector is empty
*  \date       2017-04-04
*/
static bool empty(const Vector3 &vec) {
    bool status = false;
    for (unsigned int i = 0; i < 3; ++i) {
        if (std::isnan(vec[i])) {
            status = true;
            break;
        }
    }
    return status;
}

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
    for (int i = 0; i < dim1; ++i) {
        vec[i] = first[i];
    }
    for (int i = 0; i < dim2; ++i) {
        vec[dim1 + i] = second[i];
    }
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
Vector<dim + 1> append(const Vector<dim> &source, const double add) {
    Vector<dim + 1> vec;
    for (int i = 0; i < dim; ++i) {
        vec[i] = source[i];
    }
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
*  \brief      Creates a 5 dimensional Vector with passed values
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \param[in]  rx
*  \param[in]  ry
*  \param[out] result Vector
*  \date       2016-12-23
*/
static Vector5 Vecd(double x, double y, double z, double rx, double ry) {
    Vector5 vec;
    vec << x, y, z, rx, ry;
    return vec;
}

/*!
*  \brief      Creates a 6 dimensional Vector with passed values
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \param[in]  rx
*  \param[in]  ry
*  \param[in]  rz
*  \param[out] result Vector
*  \date       2016-12-23
*/
static Vector6 Vecd(double x, double y, double z, double rx, double ry, double rz) {
    Vector6 vec;
    vec << x, y, z, rx, ry, rz;
    return vec;
}

/*!
*  \brief      Creates a Vector with template dimension and all elements are set to the passed value
*  \author     Sascha Kaden
*  \param[in]  value
*  \param[out] result Vector
*  \date       2016-12-23
*/
template <unsigned int dim>
Vector<dim> Vecd(const double data) {
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
Vector<dim> Vecd(const double data[]) {
    Vector<dim> vec;
    for (unsigned int i = 0; i < dim; ++i) {
        vec[i] = data[i];
    }
    return vec;
}

/*!
*  \brief      Creates a Vector with passed dimension and array
*  \author     Sascha Kaden
*  \param[in]  dim
*  \param[in]  array
*  \param[out] result Vector
*  \date       2016-12-23
*/
static VectorX Vecd(const unsigned int dim, const double data[]) {
    VectorX vec(dim);
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

} /* namespace util */
} /* namespace ippp */

#endif    // UTILVEC_HPP
