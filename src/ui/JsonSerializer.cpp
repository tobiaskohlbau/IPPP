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

#include <ippp/ui/JsonSerializer.hpp>

namespace ippp {
namespace jsonSerializer {

std::string serialize(const nlohmann::json &data) {
    return data.dump(4);
}

/*!
*  \brief      Serialize Transform to a json
*  \param[in]  Transform
*  \param[out] nlohmann::json
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
nlohmann::json serialize(const Transform &transform) {
    nlohmann::json json;

    MatrixX trafo = transform.matrix();
    std::vector<double> data(trafo.data(), trafo.data() + trafo.size());

    json["Transform"] = data;
    return json;
}

/*!
*  \brief      Deserialize nlohmann::json to Transform.
*  \param[in]  nlohmann::json
*  \param[out] Transform
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
Transform deserializeTransform(const nlohmann::json &data) {
    if (data.empty())
        return Transform::Identity();

    std::vector<double> stdVector = data["Transform"].get<std::vector<double>>();
    Eigen::Map<Eigen::Matrix<double, 3, 4>> map(stdVector.data());

    return Transform(map);
}

/*!
*  \brief      Serialize Transforms to a nlohmann::json.
*  \param[in]  vector of Transforms
*  \param[out] nlohmann::json
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
nlohmann::json serialize(const std::vector<Transform> &transforms) {
    if (transforms.empty())
        return std::string();

    nlohmann::json json;
    json["NumberOfTransforms"] = transforms.size();

    std::vector<std::vector<double>> data;
    for (const auto &transform : transforms) {
        MatrixX trafo = transform.matrix();
        data.push_back(std::vector<double>(trafo.data(), trafo.data() + trafo.size()));
    }
    json["Transforms"] = data;
    return json;
}

/*!
*  \brief      Deserialize nlohmann::json to a std::vector of Transforms.
*  \param[in]  nlohmann::json
*  \param[out] vector of Transforms
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
std::vector<Transform> deserializeTransforms(const nlohmann::json &data) {
    std::vector<Transform> transforms;
    if (data.empty() || data.dump().size() == 0)
        return transforms;

    std::vector<std::vector<double>> stdVectors = data["Transforms"].get<std::vector<std::vector<double>>>();
    size_t size = data["NumberOfTransforms"].get<size_t>();
    if (stdVectors.size() != size) {
        Logging::error("Wrong transforms size of file", "JsonSerializer");
        return transforms;
    }

    for (auto &vec : stdVectors) {
        Eigen::Map<Eigen::Matrix<double, 3, 4>> map(vec.data());
        transforms.push_back(Transform(map));
    }
    return transforms;
}

/*!
*  \brief      Serialize vector of DhParameter to a nlohmann::json.
*  \param[in]  std::vector of DhParameter
*  \param[out] nlohmann::json
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
*  \brief      Deserialize nlohmann::json to a std::vector of DhParameter.
*  \param[in]  nlohmann::json
*  \param[out] vector of DhParameter
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

/*!
*  \brief      Serialize VectorX with dynamic size to a nlohmann::json.
*  \param[in]  dynamical VectorX
*  \param[out] nlohmann::json
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
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

/*!
*  \brief      Deserialize nlohmann::json to a dynamic VectorX.
*  \param[in]  nlohmann::json
*  \param[out] dynamic VectorX
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
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

/*!
*  \brief      Serialize AABB (axis aligned bounding box) to a nlohmann::json.
*  \param[in]  AABB
*  \param[out] nlohmann::json
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
nlohmann::json serialize(const AABB &aabb) {
    nlohmann::json json;

    VectorX min = aabb.min();
    VectorX max = aabb.max();
    json["MinBound"] = serialize(min);
    json["MaxBound"] = serialize(max);

    return json;
}

/*!
*  \brief      Deserialize nlohmann::json to a AABB (axis aligned bounding box).
*  \param[in]  nlohmann::json
*  \param[out] AABB
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
AABB deserializeAABB(const nlohmann::json &data) {
    if (data.empty())
        return AABB();

    Vector3 bottomLeft = jsonSerializer::deserializeVector(data["MinBound"]);
    Vector3 topRight = jsonSerializer::deserializeVector(data["MaxBound"]);

    return AABB(bottomLeft, topRight);
}

/*!
*  \brief      Serialize std::vector of DofType to a nlohmann::json.
*  \param[in]  std::vector of DofType 
*  \param[out] nlohmann::json
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
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

/*!
*  \brief      Deserialize nlohmann::json to a std::vector of DofType.
*  \param[in]  nlohmann::json
*  \param[out] std::vector of DofType
*  \author     Sascha Kaden
*  \date       2017-11-30
*/
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

nlohmann::json serialize(const std::pair<Vector6, Vector6> &C) {
    nlohmann::json json;

    json["MinBound"] = serialize(static_cast<VectorX>(C.first));
    json["MaxBound"] = serialize(static_cast<VectorX>(C.second));

    return json;
}

std::pair<Vector6, Vector6> deserializeC(const nlohmann::json &data) {
    if (data.empty())
        return std::make_pair(Vector6(), Vector6());

    Vector6 min = jsonSerializer::deserializeVector(data["MinBound"]);
    Vector6 max = jsonSerializer::deserializeVector(data["MaxBound"]);

    return std::make_pair(min, max);
}

} /* namespace jsonSerializer */
} /* namespace ippp */