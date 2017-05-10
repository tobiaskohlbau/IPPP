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

#ifndef LOGGING_H
#define LOGGING_H

#include <mutex>

#include <core/Identifier.h>

namespace ippp {

enum LogLevel { none, debug, all };
enum LogOutput { terminal, file, terminlAndFile };

/*!
* \brief   Logging class for the complete framework
* \detail  At the time the output will be send by iostream
* \author  Sascha Kaden
* \date    2016-10-22
*/
class Logging {
  public:
    static void setLogLevel(LogLevel level);
    static LogLevel getLogLevel();
    static void setLogOutput(LogOutput output);
    static LogOutput getLogOutput();
    static void setOutputFile(const std::string file);
    static std::string getOutputFile();

    static void info(std::string message, Identifier *module = nullptr);
    static void warning(std::string message, Identifier *module = nullptr);
    static void error(std::string message, Identifier *module = nullptr);
    static void debug(std::string message, Identifier *module = nullptr);

    static void info(std::string message, std::string moduleName);
    static void warning(std::string message, std::string moduleName);
    static void error(std::string message, std::string moduleName);
    static void debug(std::string message, std::string moduleName);

    static void printToTerminal(std::string message);
    static void writeToFile(std::string message);

  private:
    static void sendString(std::string message);

    static LogLevel m_level;
    static LogOutput m_output;
    static std::string m_file;
    static std::mutex m_mutex;
};

} /* namespace ippp */

#endif    // LOGGING_H
