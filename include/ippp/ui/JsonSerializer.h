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

#ifndef JSONSERIALIZER_H
#define JSONSERIALIZER_H

#include <string>
#include <vector>

#include <ippp/types.h>
#include <ippp/ui/Configurator.h>
#include <ippp/util/Logging.h>

namespace ippp {

class JsonSerializer : public Configurator {
  public:
    JsonSerializer();

    std::string serialize(const std::vector<Transform> &configs);
    std::vector<Transform> deserializeTransforms(const std::string &data);

    /*!
     *  \brief      Serialize vectors to a std::string
     *  \param[in]  vector of configurations
     *  \param[in]  scale
     *  \param[out] serialized string
     *  \author     Sascha Kaden
     *  \date       2017-11-30
     */
    template <unsigned int dim>
    std::string serialize(const std::vector<Vector<dim>> &configs, double scale = 1) {
        if (configs.empty())
            return std::string();

        nlohmann::json json;
        json["Dimension"] = dim;
        json["NumberConfigurations"] = configs.size();

        std::vector<std::vector<double>> data;
        for (const auto &config : configs) {
            Vector<dim> tempConfig = scale * config;
            data.push_back(std::vector<double>(tempConfig.data(), tempConfig.data() + tempConfig.size()));
        }
        json["data"] = data;
        return json.dump(4);
    }

    /*!
    *  \brief      Deserialize std::string to a std::vector of Vector
    *  \param[in]  serialized string
    *  \param[out] vector of configurations
    *  \author     Sascha Kaden
    *  \date       2017-11-30
    */
    template <unsigned int dim>
    std::vector<Vector<dim>> deserializeVectors(const std::string &data) {
        std::vector<Vector<dim>> vectors;
        if (data.empty())
            return vectors;

        nlohmann::json json = nlohmann::json::parse(data);
        if (json["Dimension"].get<unsigned int>() != dim) {
            Logging::error("Data has wrong dimension", this);
            return vectors;
        }

        std::vector<std::vector<double>> stdVectors = json["data"].get<std::vector<std::vector<double>>>();
        size_t size = json["NumberConfigurations"].get<size_t>();
        if (stdVectors.size() != size) {
            Logging::error("Wrong vector size of file", this);
            return vectors;
        }

        for (const auto &vec : stdVectors) {
            Vector<dim> eigenVec;
            for (unsigned int j = 0; j < dim; ++j)
                eigenVec[j] = vec[j];
            vectors.push_back(eigenVec);
        }
        return vectors;
    }
};

} /* namespace ippp */

#endif    // JSONSERIALIZER_H
