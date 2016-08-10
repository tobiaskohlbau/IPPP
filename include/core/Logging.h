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

#ifndef LOGGING_H_
#define LOGGING_H_

#include <core/Base.h>

namespace rmpl {

class Logging {
  public:
    static void info(std::string message, Base *module = nullptr);
    static void warning(std::string message, Base *module = nullptr);
    static void error(std::string message, Base *module = nullptr);
    static void debug(std::string message, Base *module = nullptr);
};

} /* namespace rmpl */

#endif    // LOGGING_H_
