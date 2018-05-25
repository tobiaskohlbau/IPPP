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

#ifndef TXTSERIALIZER_H
#define TXTSERIALIZER_H

#include <sstream>
#include <string>
#include <vector>

#include <ippp/types.h>

namespace ippp {
namespace txtSerializer {

static Vector6 serializePosition(const std::string &data, double scale = 1) {
    std::cout << data << std::endl;
    Vector6 pose = Vector6::Zero();
    std::stringstream stream(data);
    std::string line;
    std::getline(stream, line);
    for (size_t i = 0; i < 3; ++i) {
        std::getline(stream, line);
        pose[i] = std::stof(line) * scale;
    }
    return pose;
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
std::string serialize(const std::vector<Vector<dim>> &configs, double scale = 1) {
    if (configs.empty())
        return std::string();

    std::stringstream stream;

    //stream << "Dimension: " << dim << std::endl;
    stream << /*"NumberConfigurations: " <<*/ configs.size() << std::endl;
    //stream << std::endl;

    std::vector<std::vector<double>> data;
    for (const auto &config : configs) {
        Vector<dim> tempConfig = scale * config;
        for (unsigned int i = 0; i < dim; ++i)
            stream << tempConfig[i] << "!";
        stream << std::endl;
    }
    return stream.str();
}

} /* namespace txtSerializer */
} /* namespace ippp */

#endif    // TXTSERIALIZER_H
