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

#include <ippp/ui/Configurator.h>

#include <fstream>

#include <ippp/util/Logging.h>

namespace ippp {

/*!
*  \brief      Standard constructor of the Configurator
*  \author     Sascha Kaden
*  \param[in]  name
*  \date       2017-10-16
*/
Configurator::Configurator(const std::string &name) : Identifier(name) {
}

/*!
*  \brief      Load a json file from the passed file path.
*  \author     Sascha Kaden
*  \param[in]  file path
*  \param[out] json
*  \date       2017-10-16
*/
nlohmann::json Configurator::loadJson(const std::string &filePath) {
    if (filePath.empty()) {
        Logging::error("Empty file path!", this);
        return nlohmann::json();
    }
    std::ifstream i(filePath);
    nlohmann::json j;
    i >> j;
    return j;
}

/*!
*  \brief      Saves the json data to the passed file path.
*  \author     Sascha Kaden
*  \param[in]  file path
*  \param[in]  json data
*  \param[out] validation of the saving
*  \date       2017-10-16
*/
bool Configurator::saveJson(const std::string &filePath, const nlohmann::json &data) {
    if (filePath.empty()) {
        Logging::error("Empty file path!", this);
        return false;
    }
    if (data.empty()) {
        Logging::warning("Empty data!", this);
        return false;
    }

    std::ofstream o(filePath);
    o << std::setw(4) << data << std::endl;
    return true;
}

} /* namespace ippp */