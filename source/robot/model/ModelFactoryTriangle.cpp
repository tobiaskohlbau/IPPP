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

#include <robot/model/ModelFactoryTriangle.h>

#include <core/utility/Logging.h>

namespace rmpl {

ModelFactoryTriangle::ModelFactoryTriangle() : ModelFactory("ModelFactoryTriangle"){

};

std::shared_ptr<ModelContainer> ModelFactoryTriangle::createModel(const std::string &filePath) {
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

    std::shared_ptr<ModelTriangle> triangleModel(new ModelTriangle());
    triangleModel->m_vertices = vertices;
    triangleModel->m_faces = faces;
    for (auto face : faces) {
        // go through faces
        Triangle2D tri(Vector2(vertices[face[0]][0], vertices[face[0]][1]), Vector2(vertices[face[1]][0], vertices[face[1]][1]), Vector2(vertices[face[2]][0], vertices[face[2]][1]));
        triangleModel->m_triangles.push_back(tri);
    }

    return triangleModel;
}

std::vector<std::shared_ptr<ModelContainer>> ModelFactoryTriangle::createModels(const std::vector<std::string> &filePaths) {
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

std::shared_ptr<ModelContainer> ModelFactoryTriangle::createModel(const std::vector<Triangle2D> triangles) {
    std::shared_ptr<ModelTriangle> triangleModel(new ModelTriangle());
    triangleModel->m_triangles = triangles;
    return triangleModel;
}

} /* namespace rmpl */
