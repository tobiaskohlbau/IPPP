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

#ifndef MODELFACTORYFCL_H
#define MODELFACTORYFCL_H

#include <robot/model/ModelFactory.h>
#include <robot/model/ModelFcl.h>

namespace rmpl {

/*!
* \brief   ModelFactory to create ModelFcl by path to source cad model
* \author  Sascha Kaden
* \date    2017-02-19
*/
class ModelFactoryFcl : public ModelFactory {
  public:
    ModelFactoryFcl() : ModelFactory("ModelFactory"){};

    std::shared_ptr<ModelContainer> createModel(const std::string &filePath);
    std::vector<std::shared_ptr<ModelContainer>> createModels(const std::vector<std::string> &filePaths);
};

} /* namespace rmpl */

#endif    // MODELFACTORYFCL_H
