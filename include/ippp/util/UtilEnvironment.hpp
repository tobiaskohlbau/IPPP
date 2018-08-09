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

#ifndef UTILENVIRONMENT_HPP
#define UTILENVIRONMENT_HPP

#include <ippp/dataObj/Graph.hpp>
#include <ippp/environment/Environment.h>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {
namespace util {

void saveMeshes(const Environment &env, const VectorX config, const std::string &prefix = "");

/*!
*  \brief      Check the summation of the robot dimensions to the planner dimension (template), return true if valid.
*  \author     Sascha Kaden
*  \param[in]  environment pointer
*  \param[out] dimension validity
*  \date       2017-05-25
*/
template <unsigned int dim>
bool checkDimensions(const Environment &environment) {
    unsigned int robotDims = 0;
    for (auto &robot : environment.getRobots())
        robotDims += robot->getDim();

    if (robotDims == dim)
        return true;
    else
        return false;
}

/*!
*  \brief      Generate a list of tcp poses from the graph and the robot.
*  \author     Sascha Kaden
*  \param[in]  graph
*  \param[in]  robot
*  \param[out] list of transformations vectors
*  \date       2018-06-19
*/
template <unsigned int dim>
std::vector<Vector6> calcTcpList(const Graph<dim> &graph, const RobotBase &robot) {
    auto nodes = graph.getNodes();
    std::vector<Vector6> tcps;
    tcps.reserve(nodes.size());

    for (auto &node : nodes)
        tcps.push_back(util::toPoseVec(robot.getTransformation(node->getValues())));

    return tcps;
}

template <unsigned int dim>
std::vector<Mesh> createTcpMeshes(const Graph<dim> &graph, const RobotBase &robot, double cubeExt = 10) {
    auto tcps = calcTcpList(graph, robot);
    Vector3 ext(cubeExt, cubeExt, cubeExt);

    std::vector<Mesh> meshes;
    for (auto &tcp : tcps) {
        Mesh mesh;

        mesh.vertices.push_back(Vector3(tcp[0], tcp[1], tcp[2]));
        mesh.vertices.push_back(Vector3(tcp[0] + ext[0], tcp[1], tcp[2]));
        mesh.vertices.push_back(Vector3(tcp[0] + ext[0], tcp[1] + ext[1], tcp[2]));
        mesh.vertices.push_back(Vector3(tcp[0], tcp[1] + ext[1], tcp[2]));
        mesh.vertices.push_back(Vector3(tcp[0], tcp[1], tcp[2] + ext[2]));
        mesh.vertices.push_back(Vector3(tcp[0] + ext[0], tcp[1], tcp[2] + ext[2]));
        mesh.vertices.push_back(Vector3(tcp[0] + ext[0], tcp[1] + ext[1], tcp[2] + ext[2]));
        mesh.vertices.push_back(Vector3(tcp[0], tcp[1] + ext[1], tcp[2] + ext[2]));

        size_t faceCount = 0;
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

        meshes.push_back(mesh);
    }

    return meshes;
}

} /* namespace util */
} /* namespace ippp */

#endif /* ENVIRONMENT_H */
