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

#include <core/Base.h>

#define DEBUG 0

using namespace rmpl;

/*!
*  \brief      Constructor of the class Base
*  \author     Sasch Kaden
*  \param[in]  name
*  \date       2016-05-30
*/
Base::Base(const std::string &name) {
    m_name = name;
}

/*!
*  \brief      Return name
*  \author     Sasch Kaden
*  \param[out] name
*  \date       2016-05-30
*/
std::string Base::getName() {
    return m_name;
}

/*!
*  \brief      Send message to the console with the name
*  \author     Sasch Kaden
*  \param[in]  message
*  \param[in]  message type
*  \date       2016-07-14
*/
void Base::sendMessage(const std::string &message, Message type) {
    if (type == Message::warning) {
        std::cout << "Warning - ";
    }
    else if (type == Message::error) {
        std::cout << "Error - ";
    }
    else if (type == Message::debug) {
        if (DEBUG) {
            std::cout << "Debug - ";
        }
        else {
            return;
        }
    }
    else
        std::cout << "Info - ";

    std::cout << m_name << ": ";
    std::cout << message << std::endl;
}
