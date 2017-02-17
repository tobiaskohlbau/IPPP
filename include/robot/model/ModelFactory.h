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

#ifndef MODELFACTORY_H
#define MODELFACTORY_H

#include <memory>
#include <string>
#include <vector>

#include <core/module/ModuleBase.h>
#include <robot/model/CadProcessing.h>
#include <robot/model/ModelContainer.h>

namespace rmpl {

class ModelFactory : public ModuleBase {
public:
    ModelFactory(const std::string &name) : ModuleBase("ModelFactory"){};

    virtual std::shared_ptr<ModelContainer> createModel(const std::string &filePath) = 0;
    virtual std::vector<std::shared_ptr<ModelContainer>> createModels(const std::vector<std::string> &filePaths) = 0;
};

} /* namespace rmpl */

#endif //MODELFACTORY_H
