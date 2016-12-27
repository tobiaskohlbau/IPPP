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

#ifndef UTILVEC_H
#define UTILVEC_H

#include <Eigen/Core>

namespace rmpl {
namespace utilVec {

bool empty(Eigen::VectorXf vec);
Eigen::VectorXf append(Eigen::VectorXf source, Eigen::VectorXf add);
Eigen::Vector3f append(Eigen::Vector2f source, float add);
Eigen::Vector4f append(Eigen::Vector3f source, float add);
Eigen::VectorXf append(Eigen::VectorXf source, float add);

Eigen::VectorXf Vecf(unsigned int dim);
Eigen::Matrix<float, 5, 1> Vecf(float x, float y, float z, float rx, float ry);
Eigen::Matrix<float, 6, 1> Vecf(float x, float y, float z, float rx, float ry, float rz);
Eigen::VectorXf Vecf(unsigned int dim, float data[]);

} /* namespace utilVec */
} /* namespace rmpl */


#endif //UTILVEC_H
