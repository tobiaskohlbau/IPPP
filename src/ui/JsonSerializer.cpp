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
namespace jsonSerializer {

std::string serialize(const nlohmann::json &data) {
    return data.dump(4);
}

/*!
*  \brief      Serialize Transforms to a std::string
*  \param[in]  vector of Transforms
*  \param[out] serialized string
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
nlohmann::json serialize(const std::vector<Transform> &configs) {
    if (configs.empty())
        return std::string();

    nlohmann::json json;
    json["NumberTransformations"] = configs.size();

    std::vector<std::vector<double>> data;
    for (const auto &config : configs) {
        MatrixX trafo = config.matrix();
        data.push_back(std::vector<double>(trafo.data(), trafo.data() + trafo.size()));
    }
    json["Transforms"] = data;
    return json;
}

/*!
*  \brief      Deserialize std::string to a std::vector of Transforms
*  \param[in]  serialized string
*  \param[out] vector of Transforms
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
std::vector<Transform> deserializeTransforms(const nlohmann::json &data) {
    std::vector<Transform> transforms;
    if (data.empty())
        return transforms;

    std::vector<std::vector<double>> stdVectors = data["Transforms"].get<std::vector<std::vector<double>>>();
    size_t size = data["NumberTransformations"].get<size_t>();
    if (stdVectors.size() != size) {
        Logging::error("Wrong transforms size of file", "JsonSerializer");
        return transforms;
    }

    for (auto &vec : stdVectors) {
        double *ptr = &vec[0];
        Eigen::Map<Eigen::Matrix<double, 3, 4>> map(vec.data());
        transforms.push_back(Transform(map));
    }
    return transforms;
}

/*!
*  \brief      Serialize Transforms to a std::string
*  \param[in]  vector of Transforms
*  \param[out] serialized string
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
nlohmann::json serialize(const std::vector<DhParameter> &parameters) {
    if (parameters.empty())
        return std::string();

    nlohmann::json json;
    json["NumberParams"] = parameters.size();

    std::vector<std::vector<double>> data;
    for (const auto &param : parameters)
        data.push_back({param.alpha, param.a, param.d, param.theta});

    json["DhParameters"] = data;
    return json;
}

/*!
*  \brief      Deserialize std::string to a std::vector of Transforms
*  \param[in]  serialized string
*  \param[out] vector of Transforms
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
std::vector<DhParameter> deserializeDhParameters(const nlohmann::json &data) {
    std::vector<DhParameter> params;
    if (data.empty())
        return params;

    std::vector<std::vector<double>> stdVectors = data["DhParameters"].get<std::vector<std::vector<double>>>();
    size_t size = data["NumberParams"].get<size_t>();
    if (stdVectors.size() != size) {
        Logging::error("Wrong transforms size of file", "JsonSerializer");
        return params;
    }

    for (auto &vec : stdVectors)
        params.push_back(DhParameter(vec[0], vec[1], vec[2], vec[3]));
    return params;
}

nlohmann::json serialize(const VectorX &vector) {
    nlohmann::json json;
    size_t size = vector.rows();
    json["Size"] = size;

    std::vector<double> stdVector(size);
    for (size_t i = 0; i < size; ++i)
        stdVector[i] = vector[i];

    json["Vector"] = stdVector;
    return json;
}

VectorX deserializeVector(const nlohmann::json &data) {
    if (data.empty())
        return VectorX();

    size_t size = data["Size"].get<size_t>();
    VectorX eigenVec(size);
    std::vector<double> vector = data["Vector"].get<std::vector<double>>();

    if (vector.size() != size) {
        Logging::error("Unequal vector sizes", "jsonSerializer");
        return VectorX();
    }

    for (size_t i = 0; i < size; ++i)
        eigenVec[i] = vector[i];

    return eigenVec;
}

nlohmann::json serialize(const AABB &aabb) {
    nlohmann::json json;

    VectorX min = aabb.min();
    VectorX max = aabb.max();
    json["MinBound"] = serialize(min);
    json["MaxBound"] = serialize(max);

    return json;
}

AABB deserializeAABB(const nlohmann::json &data) {
    if (data.empty())
        return AABB();

    Vector3 bottomLeft = jsonSerializer::deserializeVector(data["MinBound"]);
    Vector3 topRight = jsonSerializer::deserializeVector(data["MaxBound"]);

    return AABB(bottomLeft, topRight);
}

nlohmann::json serialize(const std::vector<DofType> &dofTypes) {
    nlohmann::json json;
    if (dofTypes.empty())
        return json;
        
    json["Size"] = dofTypes.size();
    std::vector<int> dofs;
    for (auto &dof : dofTypes)
        dofs.push_back(static_cast<int>(dof));

    json["DofTypes"] = dofs;
    return json;
}

std::vector<DofType> deserializeDofTypes(const nlohmann::json &data) {
    std::vector<DofType> dofTypes;
    if (data.empty())
        return dofTypes;

    size_t size = data["Size"].get<size_t>();
    dofTypes.reserve(size);
    std::vector<int> vector = data["DofTypes"].get<std::vector<int>>();

    if (vector.size() != size) {
        Logging::error("Unequal vector sizes", "jsonSerializer");
        return dofTypes;
    }

    for (size_t i = 0; i < size; ++i)
        dofTypes.push_back(static_cast<DofType>(vector[i]));

    return dofTypes;
}

} /* namespace jsonSerializer */
} /* namespace ippp */