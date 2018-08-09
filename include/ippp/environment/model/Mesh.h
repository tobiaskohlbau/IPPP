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

#ifndef MESH_H
#define MESH_H

#include <vector>

#include <ippp/types.h>

namespace ippp {

/*!
* \brief   The Mesh class holds all information and data of a mesh, always used at the ModelContainer classes.
* \author  Sascha Kaden
* \date    2017-11-10
*/
class Mesh {
  public:
    std::vector<Vector3> vertices; /*!< vertex list of the mesh */
    std::vector<Vector3i> faces;   /*!< faces list of the mesh with indexes to the vertices */
    std::vector<Vector3> normals;  /*!< normals to every vertex */
    AABB aabb;                     /*!< axis aligned bounding box of the mesh */
};

} /* namespace ippp */

#endif    // MESH_H
