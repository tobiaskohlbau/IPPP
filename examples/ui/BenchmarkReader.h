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

#ifndef BENCHMARKREADER_H
#define BENCHMARKREADER_H

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <Eigen/Core>

#include <core/types.h>

namespace rmpl {

typedef struct EnvironmentConfig {
    Vector3 minBoundary;
    Vector3 maxBoundary;
    unsigned int numOfRobots;
    std::string robotFile;
    std::string obstacleFile;
    Vector6 obstacleConfig;
} EnvironmentConfig_t;

EnvironmentConfig readEnvironment(const std::string &path);
std::vector<Vector6> readQuery(const std::string &path);

} /* namespace rmpl */

#endif //BENCHMARKREADER_H