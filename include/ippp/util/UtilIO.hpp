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

#ifndef UTILIO_HPP
#define UTILIO_HPP

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <ippp/types.h>
#include <ippp/util/Logging.h>

namespace ippp {
namespace util {

/*!
*  \brief      Read transformation matrix from file.
*  \author     Sascha Kaden
*  \param[in]  file path
*  \param[out] transformation matrix
*  \date       2017-04-07
*/
static Matrix4 readT(const std::string &path, const double scale = 1) {
    Matrix4 T;
    if (path.substr(path.find_last_of('.')) != ".dat") {
        Logging::error("Wrong file type", "UtilIO");
    }
    std::ifstream fin(path);

    if (fin.is_open()) {
        for (size_t row = 0; row < 4; ++row)
            for (size_t col = 0; col < 4; ++col) {
                double item = 0.0;
                fin >> item;
                T(row, col) = item;
            }
        fin.close();
    }
    T.block<3, 1>(0, 3) = T.block<3, 1>(0, 3) * scale;
    return T;
}

} /* namespace util */
} /* namespace ippp */

#endif    // UTILIO_HPP
