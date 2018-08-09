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

#include <ippp/environment/cad/CadDrawing.h>

#include <ippp/util/Logging.h>

namespace ippp {
namespace cad {

std::pair<MatrixXi, Vector2i> create2dspace(const AABB &boundary, const int fillValue) {
    Vector3 dia = boundary.diagonal();
    Vector3 min = boundary.min();
    // pair with Eigen::Matrix as space and the offset of the AABB to the origin
    return std::make_pair(MatrixXi::Constant(static_cast<int>(dia[0]), static_cast<int>(dia[1]), fillValue),
                          Vector2i(-min[0], -min[1]));
}

} /* namespace cad */
} /* namespace ippp */
