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

#include <ui/BenchmarkReader.h>

#include <fstream>
#include <iostream>

#include <core/util/Logging.h>
#include <core/util/UtilList.hpp>

namespace ippp {

EnvironmentConfig readEnvironment(const std::string &file) {
    std::string extension = file.substr(file.find_last_of("."));
    if (extension != ".env") {
        Logging::error("Wrong file type", "BenchmarkReader");
        return EnvironmentConfig();
    }

    EnvironmentConfig config;

    std::ifstream is(file);
    std::string str;
    while (getline(is, str)) {
        if (util::contains(str, "Boundary Box")) {
            size_t firstLim = str.find("[");
            size_t lastLim = str.find("]");
            std::string box = str.substr(firstLim + 1, lastLim);

            std::size_t found = box.find_first_of(";");
            unsigned int count = 0;
            std::string::size_type sz;
            while (found != std::string::npos) {
                config.minBoundary[count] = std::stof(box, &sz);
                ++sz;
                config.maxBoundary[count] = std::stof(box.substr(sz));

                box = box.substr(found + 2);
                found = box.find_first_of(";");
                ++count;
            }
            config.minBoundary[count] = std::stof(box, &sz);
            ++sz;
            config.maxBoundary[count] = std::stof(box.substr(sz));

        } else if (util::contains(str, "Passive")) {
            getline(is, str);
            if (util::contains(str, "#"))
                getline(is, str);
            std::size_t found = str.find_first_of(" ");
            config.obstacleFile = str.substr(0, found);

            str = str.substr(found + 1);
            if (util::contains(str, "-c")) {
                str = str.substr(str.find_first_of(")") + 1);
            }

            util::trimWhitespaces(str);
            std::string::size_type sz;
            for (int i = 0; i < 6; ++i) {
                config.obstacleConfig[i] = std::stof(str, &sz);
                if (sz < str.size())
                    str = str.substr(sz + 1);
            }
        } else if (util::contains(str, "Active")) {
            getline(is, str);
            config.numOfRobots = std::stoi(str);
            getline(is, str);
            if (util::contains(str, "#"))
                getline(is, str);
            std::size_t found = str.find_first_of(" ");
            config.robotFile = str.substr(0, found);
        }
    }
    return config;
}

std::vector<Vector6> readQuery(const std::string &path) {
    std::vector<Vector6> configs;
    std::ifstream is(path);
    std::string str;
    while (getline(is, str)) {
        util::trimWhitespaces(str);
        if (str == "")
            continue;

        std::string::size_type sz;
        Vector6 config;
        for (int i = 0; i < 7; ++i) {
            double value = std::stof(str, &sz);
            if (i != 0)
                config[i - 1] = value;

            if (sz < str.size())
                str = str.substr(sz + 1);
        }
        configs.push_back(config);
    }
    return configs;
}

} /* namespace ippp */
