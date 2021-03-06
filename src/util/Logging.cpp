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

#include <ippp/util/Logging.h>

#include <fstream>
#include <iostream>

namespace ippp {

LogLevel Logging::m_level = LogLevel::info;
LogOutput Logging::m_output = LogOutput::terminal;
std::string Logging::m_file;
std::mutex Logging::m_mutex;

/*!
*  \brief      Set global log level
*  \param[in]  LogLevel
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
void Logging::setLogLevel(LogLevel level) {
    m_level = level;
}

/*!
*  \brief      Return global log level
*  \param[out] LogLevel
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
LogLevel Logging::getLogLevel() {
    return m_level;
}

/*!
*  \brief      Set Logging output
*  \param[in]  LogOutput
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
void Logging::setLogOutput(LogOutput output) {
    m_output = output;
}

/*!
*  \brief      Return Logging output
*  \param[out] LogOutput
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
LogOutput Logging::getLogOutput() {
    return m_output;
}

/*!
*  \brief      Set output file
*  \param[in]  output file
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
void Logging::setOutputFile(const std::string& file) {
    if (file.empty()) {
        return;
    }
    m_file = file;
}

/*!
*  \brief      Return output file
*  \param[out] output file
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
std::string Logging::getOutputFile() {
    return m_file;
}

/*!
*  \brief      Info logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::info(const std::string& message, Identifier* module) {
    if (module == nullptr) {
        info(message, "Unknown");
    } else {
        info(message, module->getName());
    }
}

/*!
*  \brief      Warning logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::warning(const std::string& message, Identifier* module) {
    if (module == nullptr) {
        warning(message, "Unknown");
    } else {
        warning(message, module->getName());
    }
}

/*!
*  \brief      Error logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::error(const std::string& message, Identifier* module) {
    if (module == nullptr) {
        error(message, "Unknown");
    } else {
        error(message, module->getName());
    }
}

/*!
*  \brief      Debug logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::debug(const std::string& message, Identifier* module) {
    if (module == nullptr) {
        debug(message, "Unknown");
    } else {
        debug(message, module->getName());
    }
}

/*!
*  \brief      Trace logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::trace(const std::string& message, Identifier* module) {
    if (module == nullptr) {
        trace(message, "Unknown");
    } else {
        trace(message, module->getName());
    }
}

/*!
*  \brief      Info logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::info(const std::string& message, const Identifier* module) {
    if (module == nullptr) {
        info(message, "Unknown");
    } else {
        info(message, module->getName());
    }
}

/*!
*  \brief      Warning logging with module for identificaion
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::warning(const std::string& message, const Identifier* module) {
    if (module == nullptr) {
        warning(message, "Unknown");
    } else {
        warning(message, module->getName());
    }
}

/*!
*  \brief      Error logging with module for identification
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::error(const std::string& message, const Identifier* module) {
    if (module == nullptr) {
        error(message, "Unknown");
    } else {
        error(message, module->getName());
    }
}

/*!
*  \brief      Debug logging with module for identification
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::debug(const std::string& message, const Identifier* module) {
    if (module == nullptr) {
        debug(message, "Unknown");
    } else {
        debug(message, module->getName());
    }
}

/*!
*  \brief      Trace logging with module for identification
*  \param[in]  message
*  \param[in]  module pointer
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::trace(const std::string& message, const Identifier* module) {
    if (module == nullptr) {
        trace(message, "Unknown");
    } else {
        trace(message, module->getName());
    }
}

/*!
*  \brief      Info logging with module name for identification
*  \param[in]  message
*  \param[in]  module name
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::info(const std::string& message, const std::string& moduleName) {
    sendString("Info " + moduleName + ": " + message, LogLevel::info);
}

/*!
*  \brief      Warning logging with module name for identification
*  \param[in]  message
*  \param[in]  module name
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::warning(const std::string& message, const std::string& moduleName) {
    sendString("Warning " + moduleName + ": " + message, LogLevel::warn);
}

/*!
*  \brief      Error logging with module name for identification
*  \param[in]  message
*  \param[in]  module name
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::error(const std::string& message, const std::string& moduleName) {
    sendString("Error " + moduleName + ": " + message, LogLevel::error);
}

/*!
*  \brief      Debug logging with module name for identification
*  \param[in]  message
*  \param[in]  module name
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::debug(const std::string& message, const std::string& moduleName) {
    sendString("Debug " + moduleName + ": " + message, LogLevel::debug);
}

/*!
*  \brief      Trace logging with module name for identification
*  \param[in]  message
*  \param[in]  module name
*  \author     Sascha Kaden
*  \date       2016-10-22
*/
void Logging::trace(const std::string& message, const std::string& moduleName) {
    sendString("Trace " + moduleName + ": " + message, LogLevel::trace);
}

/*!
*  \brief      Send message by defined LogOutput method, satisfy thread safety
*  \param[in]  message
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
void Logging::sendString(const std::string& message, LogLevel level) {
    if (level > m_level)
        return;

    m_mutex.lock();
    if (m_output == LogOutput::file) {
        writeToFile(message);
    } else if (m_output == LogOutput::terminlAndFile) {
        printToTerminal(message);
        writeToFile(message);
    } else {
        printToTerminal(message);
    }
    m_mutex.unlock();
}

/*!
*  \brief      Send message to the terminal
*  \param[in]  message
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
void Logging::printToTerminal(const std::string& message) {
    std::cout << message << std::endl;
}

/*!
*  \brief      Append message to the defined output file
*  \param[in]  message
*  \author     Sascha Kaden
*  \date       2016-11-14
*/
void Logging::writeToFile(const std::string& message) {
    if (m_file.empty()) {
        return;
    }
    std::ofstream myfile;
    myfile.open(m_file, std::ios_base::app);
    if (myfile.is_open()) {
        myfile << message << std::endl;
        myfile.close();
    }
}

} /* namespace ippp */
