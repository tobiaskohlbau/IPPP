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

#ifndef WRITER_H
#define WRITER_H

#include <vector>

#include <core/ModuleBase.h>
#include <core/Vec.hpp>

/*!
* \brief   Provides methods for writing Vec lists to passed files
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Writer : public rmpl::ModuleBase {
  public:
    static void writeVecsToFile(const std::vector<rmpl::Vec<float>> &vecs, const std::string &filename, float scale = 1);
    static void appendVecsToFile(const std::vector<rmpl::Vec<float>> &vecs, const std::string &filename, float scale = 1);
    static void writeTrafosToFile(const std::vector<std::vector<rmpl::Vec<float>>> &vecs, const std::string &filenames);
};

#endif    // WRITER_H
