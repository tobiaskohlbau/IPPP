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

#include <ippp/ui/TxtSerializer.hpp>

namespace ippp {
namespace txtSerializer {

Vector6 deserializePosition(const std::string &data, double scale) {
    Vector6 pose = Vector6::Zero();
    std::stringstream stream(data);
    std::string line;
    std::getline(stream, line);
    std::getline(stream, line);

    std::string::size_type sz;
    pose[0] = std::stof(line, &sz);
    line = line.substr(sz);
    pose[1] = std::stof(line, &sz);
    line = line.substr(sz);
    pose[2] = std::stof(line, &sz);
    return pose;
}

} /* namespace txtSerializer */
} /* namespace ippp */
