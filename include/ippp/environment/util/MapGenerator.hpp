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

#ifndef MAPGENERATOR_HPP
#define MAPGENERATOR_HPP

#include <ippp/core/Identifier.h>
#include <ippp/core/sampler/Sampler.hpp>
#include <ippp/environment/cad/CadProcessing.h>

namespace ippp {

/*!
* \brief   MapGenerator creates 2D and 3D maps with random rectangles or cubes.
* \details The rectangles and cubes are build of triangles and will be saved in a Mesh container class. For every
 * obstacle will be a new random extension size between 0 and maxExtension computed.
* \author  Sascha Kaden
* \date    2017-06-99
*/
template <unsigned int dim>
class MapGenerator : public Identifier {
  public:
    MapGenerator(const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary, const std::shared_ptr<Sampler<dim>> &sampler);
    std::vector<Mesh> generateMap(const unsigned int numObstacles, const Vector<dim> &maxExtensions);

  protected:
    bool checkBounding(const Vector<dim> &sample, const Vector<dim> &extension);

    Vector<dim> m_minBoundary;
    Vector<dim> m_maxBoundary;

    std::shared_ptr<Sampler<dim>> m_sampler = nullptr;
};

/*!
*  \brief      Standard constructor of the MapGenerator
*  \author     Sascha Kaden
*  \param[in]  bounding for the map
*  \param[in]  sampler
*  \date       2017-06-99
*/
template <unsigned int dim>
MapGenerator<dim>::MapGenerator(const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary,
                                const std::shared_ptr<Sampler<dim>> &sampler)
    : Identifier("MapGenerator"), m_minBoundary(minBoundary), m_maxBoundary(maxBoundary), m_sampler(sampler) {
}

/*!
*  \brief      Generates a list of obstacle meshes with the defined options
*  \author     Sascha Kaden
*  \param[in]  number of obstacles
*  \param[in]  maximum extensions of a rectangle/cube
*  \param[out] list of obstacles
*  \date       2017-06-99
*/
template <unsigned int dim>
std::vector<Mesh> MapGenerator<dim>::generateMap(const unsigned int numObstacles, const Vector<dim> &maxExtensions) {
    assert(dim == 3 || dim == 2);
    Vector<dim> ext;
    std::vector<Mesh> meshes;
    for (int i = 0; i < numObstacles; ++i) {
        Mesh mesh;
        auto sample = m_sampler->getSample();
        for (unsigned int j = 0; j < dim; ++j)
            ext[j] = m_sampler->getRandomNumber() * maxExtensions[j];

        // check if new obstacle is in the boundaries
        if (!checkBounding(sample, ext)) {
            --i;
            continue;
        }

        int faceCount = mesh.vertices.size();
        if (dim == 2) {
            mesh.vertices.push_back(Vector3(sample[0], sample[1], 0));
            mesh.vertices.push_back(Vector3(sample[0] + ext[0], sample[1], 0));
            mesh.vertices.push_back(Vector3(sample[0], sample[1] + ext[1], 0));
            mesh.vertices.push_back(Vector3(sample[0] + ext[0], sample[1] + ext[1], 0));

            mesh.faces.push_back(Vector3i(faceCount, faceCount + 1, faceCount + 2));
            mesh.faces.push_back(Vector3i(faceCount + 1, faceCount + 2, faceCount + 3));
        }

        if (dim == 3) {
            mesh.vertices.push_back(Vector3(sample[0], sample[1], sample[2]));
            mesh.vertices.push_back(Vector3(sample[0] + ext[0], sample[1], sample[2]));
            mesh.vertices.push_back(Vector3(sample[0] + ext[0], sample[1] + ext[1], sample[2]));
            mesh.vertices.push_back(Vector3(sample[0], sample[1] + ext[1], sample[2]));
            mesh.vertices.push_back(Vector3(sample[0], sample[1], sample[2] + ext[2]));
            mesh.vertices.push_back(Vector3(sample[0] + ext[0], sample[1], sample[2] + ext[2]));
            mesh.vertices.push_back(Vector3(sample[0] + ext[0], sample[1] + ext[1], sample[2] + ext[2]));
            mesh.vertices.push_back(Vector3(sample[0], sample[1] + ext[1], sample[2] + ext[2]));

            mesh.faces.push_back(Vector3i(faceCount, faceCount + 2, faceCount + 1));
            mesh.faces.push_back(Vector3i(faceCount, faceCount + 3, faceCount + 2));
            mesh.faces.push_back(Vector3i(faceCount + 1, faceCount + 2, faceCount + 6));
            mesh.faces.push_back(Vector3i(faceCount + 6, faceCount + 5, faceCount + 1));
            mesh.faces.push_back(Vector3i(faceCount + 4, faceCount + 5, faceCount + 6));
            mesh.faces.push_back(Vector3i(faceCount + 6, faceCount + 7, faceCount + 4));
            mesh.faces.push_back(Vector3i(faceCount + 2, faceCount + 3, faceCount + 6));
            mesh.faces.push_back(Vector3i(faceCount + 6, faceCount + 3, faceCount + 7));
            mesh.faces.push_back(Vector3i(faceCount, faceCount + 7, faceCount + 3));
            mesh.faces.push_back(Vector3i(faceCount, faceCount + 4, faceCount + 7));
            mesh.faces.push_back(Vector3i(faceCount, faceCount + 1, faceCount + 5));
            mesh.faces.push_back(Vector3i(faceCount, faceCount + 5, faceCount + 4));
        }
        meshes.push_back(mesh);
    }
    return meshes;
}

/*!
*  \brief      Check if a sample with the calculated extension lies inside of the boundary
*  \author     Sascha Kaden
*  \param[in]  sample
*  \param[in]  extensions
*  \param[out] validity, true if out of bounds
*  \date       2017-06-99
*/
template <unsigned int dim>
bool MapGenerator<dim>::checkBounding(const Vector<dim> &sample, const Vector<dim> &ext) {
    for (unsigned int i = 0; i < dim; ++i)
        if (sample[i] < m_minBoundary[i] || sample[i] + ext[i] > m_maxBoundary[i])
            return false;

    return true;
}

} /* namespace ippp */

#endif    // MAPGENERATOR_HPP
