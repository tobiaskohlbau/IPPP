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

#ifndef FILEWRITERREADER_H
#define FILEWRITERREADER_H

#include <string>
#include <fstream>

#include <json.hpp>

namespace ippp {
namespace ui {

bool save(const std::string &filePath, const nlohmann::json &data);
bool save(const std::string &filePath, const std::string &data);
std::string load(const std::string &filePath);
nlohmann::json loadJson(const std::string &filePath);

} /* namespace ui */
} /* namespace ippp */

#endif    // FILEWRITERREADER_H
