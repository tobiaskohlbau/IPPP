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

/*!
*  \brief      Info logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sasch Kaden
*  \date       2016-10-22
*/
void Logging::info(std::string message, ModuleBase *module) {
    if (module == nullptr)
        info(message, "Unknown");
    else
        info(message, module->getName());
}

/*!
*  \brief      Warning logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sasch Kaden
*  \date       2016-10-22
*/
void Logging::warning(std::string message, ModuleBase *module) {
    if (module == nullptr)
        warning(message, "Unknown");
    else
        warning(message, module->getName());
}

/*!
*  \brief      Error logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sasch Kaden
*  \date       2016-10-22
*/
void Logging::error(std::string message, ModuleBase *module) {
    if (module == nullptr)
        error(message, "Unknown");
    else
        error(message, module->getName());
}

/*!
*  \brief      Debug logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sasch Kaden
*  \date       2016-10-22
*/
void Logging::debug(std::string message, ModuleBase *module) {
    if (module == nullptr)
        debug(message, "Unknown");
    else
        debug(message, module->getName());
}

/*!
*  \brief      Info logging with module name for identificaion
*  \param[in]  message
*  \param[in]  module name
*  \author     Sasch Kaden
*  \date       2016-10-22
*/
void Logging::info(std::string message, std::string moduleName) {
    std::cout << "Info " << moduleName << ": " << message << std::endl;
}

/*!
*  \brief      Warning logging with module name for identificaion
*  \param[in]  message
*  \param[in]  module name
*  \author     Sasch Kaden
*  \date       2016-10-22
*/
void Logging::warning(std::string message, std::string moduleName) {
    std::cout << "Warning " << moduleName << ": " << message << std::endl;
}

/*!
*  \brief      Error logging with module name for identificaion
*  \param[in]  message
*  \param[in]  module name
*  \author     Sasch Kaden
*  \date       2016-10-22
*/
void Logging::error(std::string message, std::string moduleName) {
    std::cout << "ERROR " << moduleName << ": " << message << std::endl;
}

/*!
*  \brief      Debug logging with module name for identificaion
*  \param[in]  message
*  \param[in]  module name
*  \author     Sasch Kaden
*  \date       2016-10-22
*/
void Logging::debug(std::string message, std::string moduleName) {
#ifdef DEBUG_OUTPUT
    std::cout << "Debug " << moduleName << ": " << message << std::endl;
#endif
}
