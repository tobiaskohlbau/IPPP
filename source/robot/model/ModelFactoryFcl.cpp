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

#include <robot/model/ModelFactoryFcl.h>

#include <core/utility/Logging.h>

namespace rmpl {

std::shared_ptr<ModelContainer> ModelFactoryFcl::createModel(const std::string &filePath) {
    if (filePath == "") {
        Logging::error("Empty file path", this);
        return nullptr;
    }
    std::vector<Vector3> vertices;
    std::vector<Vector3i> faces;
    if (!importCad(filePath, vertices, faces)) {
        Logging::error("Could not load mesh", this);
        return nullptr;
    }

    std::shared_ptr<ModelFcl> fclModel(new ModelFcl());
    fclModel->m_vertices = vertices;
    fclModel->m_faces = faces;
    std::vector<fcl::Vector3f> verts;
    std::vector<fcl::Triangle> triangles;
    for (auto vert : vertices)
        verts.push_back(fcl::Vector3f(vert[0], vert[1], vert[2]));
    for (auto face : faces)
        triangles.push_back(fcl::Triangle(face[0], face[1], face[2]));
    fclModel->m_fclModel.beginModel();
    fclModel->m_fclModel.addSubModel(verts, triangles);
    fclModel->m_fclModel.endModel();

    return fclModel;
}

std::vector<std::shared_ptr<ModelContainer>> ModelFactoryFcl::createModels(const std::vector<std::string> &filePaths) {
    std::vector<std::shared_ptr<ModelContainer>> models;
    std::shared_ptr<ModelContainer> model;
    for (auto filePath : filePaths) {
        model = createModel(filePath);
        if (model)
            models.push_back(model);
        else
            return std::vector<std::shared_ptr<ModelContainer>>();
    }
    return models;
}

} /* namespace rmpl */
