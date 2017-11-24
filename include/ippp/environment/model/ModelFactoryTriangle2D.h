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

#ifndef MODELFACTORYTRIANGLE2D_H
#define MODELFACTORYTRIANGLE2D_H

#include <ippp/environment/model/ModelFactory.h>
#include <ippp/environment/model/ModelTriangle2D.h>

namespace ippp {

/*!
* \brief   ModelFactory to create ModelFcl by path to source model or by a list of triangles
* \author  Sascha Kaden
* \date    2017-02-19
*/
class ModelFactoryTriangle2D : public ModelFactory {
  public:
    ModelFactoryTriangle2D();

    std::shared_ptr<ModelContainer> createModel(const std::string &filePath);
    std::vector<std::shared_ptr<ModelContainer>> createModels(const std::string &filePath);
    std::vector<std::shared_ptr<ModelContainer>> createModels(const std::vector<std::string> &filePaths);
    std::shared_ptr<ModelContainer> createModel(const std::vector<Triangle2D> &triangles);
};

} /* namespace ippp */

#endif    // MODELFACTORYTRIANGLE2D_H
