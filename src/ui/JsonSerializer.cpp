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

#include <ippp/ui/JsonSerializer.h>

namespace ippp {

JsonSerializer::JsonSerializer() : Configurator("JsonSerializer") {
}

std::string JsonSerializer::serialize(const std::vector<Transform> &configs) {
    if (configs.empty())
        return std::string();

    nlohmann::json json;
    json["NumberTransformations"] = configs.size();

    std::vector<std::vector<double>> data;
    for (const auto &config : configs) {
        MatrixX trafo = config.matrix();
        data.push_back(std::vector<double>(trafo.data(), trafo.data() + trafo.size()));
    }
    json["data"] = data;
    return json.dump(4);
}

std::vector<Transform> JsonSerializer::deserializeTransforms(const std::string &data) {
    std::vector<Transform> transforms;
    if (data.empty())
        return transforms;

    nlohmann::json json = nlohmann::json::parse(data);

    std::vector<std::vector<double>> stdVectors = json["data"].get<std::vector<std::vector<double>>>();
    size_t size = json["NumberTransformations"].get<size_t>();
    if (stdVectors.size() != size) {
        Logging::error("Wrong transforms size of file", this);
        return transforms;
    }

    for (auto &vec : stdVectors) {
        double* ptr = &vec[0];
        Eigen::Map<Eigen::Matrix<double, 3, 4>> map(vec.data());
        transforms.push_back(Transform(map));
    }
    return transforms;
}

} /* namespace ippp */