//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#include <robot/model/ModelFcl.h>

#include <robot/model/CadProcessing.h>

namespace rmpl {

ModelFcl::ModelFcl() : ModelContainer("ModelFcl") {
}

/*!
*  \brief      Return true if model is empty
*  \author     Sascha Kaden
*  \param[out] state
*  \date       2017-02-19
*/
bool ModelFcl::empty() const {
    if (m_fclModel.num_vertices == 0) {
        return true;
    } else {
        return false;
    }
}

void ModelFcl::transform(const Vector6 &config) {
    transformCad(config, m_vertices);

    std::vector<fcl::Vector3f> verts;
    std::vector<fcl::Triangle> triangles;
    for (auto vert : m_vertices) {
        verts.push_back(fcl::Vector3f(vert[0], vert[1], vert[2]));
    }
    for (auto face : m_faces) {
        triangles.push_back(fcl::Triangle(face[0], face[1], face[2]));
    }
    m_fclModel = FCLModel();
    m_fclModel.beginModel();
    m_fclModel.addSubModel(verts, triangles);
    m_fclModel.endModel();
}

} /* namespace rmpl */
