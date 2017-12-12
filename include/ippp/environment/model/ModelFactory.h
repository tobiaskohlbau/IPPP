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

#ifndef MODELFACTORY_H
#define MODELFACTORY_H

#include <memory>
#include <string>
#include <vector>

#include <ippp/Identifier.h>
#include <ippp/environment/cad/CadImportExport.h>
#include <ippp/environment/cad/CadProcessing.h>
#include <ippp/environment/model/ModelContainer.h>

namespace ippp {

/*!
* \brief   Superclass of all models factories. It defines the general interface of the factories.
* \author  Sascha Kaden
* \date    2017-11-10
*/
class ModelFactory : public Identifier {
  public:
    ModelFactory(const std::string &name) : Identifier(name){};

    virtual std::shared_ptr<ModelContainer> createModelFromFile(const std::string &filePath) = 0;
    virtual std::vector<std::shared_ptr<ModelContainer>> createModelsFromFile(const std::string &filePath) = 0;
    virtual std::vector<std::shared_ptr<ModelContainer>> createModelsFromFiles(const std::vector<std::string> &filePaths) = 0;
};

} /* namespace ippp */

#endif    // MODELFACTORY_H
