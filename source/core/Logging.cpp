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

#include <core/Logging.h>

using namespace rmpl;

void Logging::info(std::string message, Base *module) {
    if (module == nullptr)
        std::cout << "Info "
                  << "Unknown"
                  << " : " << message << std::endl;
    else
        std::cout << "Info " << module->getName() << " : " << message << std::endl;
}

void Logging::warning(std::string message, Base *module) {
    if (module == nullptr)
        std::cout << "Warning "
                  << "Unknown"
                  << " : " << message << std::endl;
    else
        std::cout << "Warning " << module->getName() << " : " << message << std::endl;
}

void Logging::error(std::string message, Base *module) {
    if (module == nullptr)
        std::cout << "Error "
                  << "Unknown"
                  << " : " << message << std::endl;
    else
        std::cout << "Error " << module->getName() << " : " << message << std::endl;
}

void Logging::debug(std::string message, Base *module) {
#ifdef DEBUG_OUTPUT
    if (module == nullptr)
        std::cout << "Debug "
                  << "Unknown"
                  << " : " << message << std::endl;
    else
        std::cout << "Debug " << module->getName() << " : " << message << std::endl;
#endif
}
