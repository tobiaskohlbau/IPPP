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

#ifndef CADIMPORTEXPORT_H
#define CADIMPORTEXPORT_H

#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <ippp/environment/cad/CadProcessing.h>

namespace ippp {
namespace cad {

enum class ExportFormat {
    COLLADA,
    X_FILES,
    STEP,
    OBJ,
    STEREOLITHOGRAPHY,
    STEREOLITHOGRAPHY_BINARY,
    STANFORD_POLYGON_LIBRARY,
    STANFORD_POLYGON_LIBRARY_BINARY,
    AUTODESK_3DS,
    GL_TRANSMISSION_FORMAT,
    GL_TRANSMISSION_FORMAT_BINARY,
    ASSIMP_BINARY,
    ASSXML_DOCUMENT,
    EXTENSIBLE_3D
};

bool importMesh(const std::string &filePath, Mesh &mesh, double scale = 1, bool calcNormals = false, bool useTrafo = true);
bool importMeshes(const std::string &filePath, std::vector<Mesh> &meshes, double scale = 1, bool calcNormals = false,
                  bool useTrafo = true);
Matrix4 importTransformation(const std::string &filePath);
void getMeshes(const aiScene *scene, const aiNode *node, aiMatrix4x4 *trafo, std::vector<Mesh> &meshes,
               const bool useTrafo = true);
bool importBYU(const std::string &filePath, Mesh &mesh);

bool exportCad(ExportFormat format, const std::string &filePath, const Mesh &mesh);
bool exportCad(ExportFormat format, const std::string &filePath, const std::vector<Vector3> &vertices,
               const std::vector<Vector3i> &faces);

aiScene generateScene(const std::vector<Vector3> &vertices, const std::vector<Vector3i> &faces);

} /* namespace cad */

} /* namespace ippp */

#endif    // CADIMPORTEXPORT_H
