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

#include <ippp/ui/FileWriterReader.h>

#include <ippp/util/Logging.h>

namespace ippp {
namespace ui {

bool save(const std::string &filePath, const std::string &data) {
    std::ofstream file;
    file.open(filePath);

    if (!file.is_open()) {
        Logging::error("Could not open file", "FileWriterReader");
        return false;
    }

    file << data;
    file.close();

    return true;
}

std::string load(const std::string &filePath) {
    std::string data;

    std::ifstream file;
    file.open(filePath);

    if (!file.is_open()) {
        Logging::error("Could not open file", "FileWriterReader");
        return std::string();
    }
    
    file >> data;
    file.close();

    return data;
}

} /* namespace ui */
} /* namespace ippp */