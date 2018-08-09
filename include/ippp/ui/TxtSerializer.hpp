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

#ifndef TXTSERIALIZER_H
#define TXTSERIALIZER_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ippp/types.h>

namespace ippp {
namespace txtSerializer {

Vector6 deserializePosition(const std::string &data, double scale = 1);

/*!
*  \brief      Serialize vectors to std::string
*  \param[in]  vector of configurations
*  \param[in]  scale
*  \param[out] std::string
*  \author     Sascha Kaden
*  \date       2018-04-19
*/
template <unsigned int dim>
std::string serialize(const std::vector<Vector<dim>> &configs, double scale = 1, const std::string &delimeter = " ") {
    if (configs.empty())
        return std::string();

    std::stringstream stream;

    std::vector<std::vector<double>> data;
    for (const auto &config : configs) {
        Vector<dim> tempConfig = scale * config;
        for (unsigned int i = 0; i < dim; ++i)
            stream << tempConfig[i] << delimeter;
        stream << std::endl;
    }
    return stream.str();
}

/*!
*  \brief      Serialize vectors to std::string
*  \param[in]  vector of configurations
*  \param[in]  scale
*  \param[out] std::string
*  \author     Sascha Kaden
*  \date       2018-04-19
*/
template <unsigned int dim>
std::vector<Vector<dim>> deserializeVectors(const std::string &data) {
    if (data.empty())
        return std::vector<Vector<dim>>();

    std::stringstream stream(data);
    std::string line;
    std::string element;

    std::vector<Vector<dim>> vectors;
    while (std::getline(stream, line)) {
        Vector<dim> vec;
        for (size_t i = 0; i < dim; ++i) {
            std::stringstream ss(line);

            ss >> vec[i];
        }
        vectors.push_back(vec);
    }
    return vectors;
}

} /* namespace txtSerializer */
} /* namespace ippp */

#endif    // TXTSERIALIZER_H
