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

#include <ippp/util/UtilVec.hpp>

namespace ippp {
namespace util {

/*!
*  \brief      Check whether the vector is empty (contains a NaN value)
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \param[out] result, true if vector is empty
*  \date       2017-04-04
*/
bool empty(const Vector3 &vec) {
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
Vector5 Vecd(double x, double y, double z, double rx, double ry) {
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
Vector6 Vecd(double x, double y, double z, double rx, double ry, double rz) {
    Vector6 vec;
    vec << x, y, z, rx, ry, rz;
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
Vector7 Vecd(double a1, double a2, double a3, double a4, double a5, double a6, double a7) {
    Vector7 vec;
    vec << a1, a2, a3, a4, a5, a6, a7;
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
VectorX Vecd(unsigned int dim, double data[]) {
    VectorX vec(dim);
    for (unsigned int i = 0; i < dim; ++i) {
        vec[i] = data[i];
    }
    return vec;
}

} /* namespace util */
} /* namespace ippp */
