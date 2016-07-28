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

#ifndef BASE_H_
#define BASE_H_

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

namespace rmpl{

enum class Message
{
    info,
    warning,
    error,
    debug
};

/*!
* \brief   Base class of all modules
* \detail  Will be used to send messages and adds the name of the class
* \author  Sascha Kaden
* \date    2016-06-02
*/
class Base
{
public:
    Base(const std::string &name);
    std::string getName();
    void sendMessage(const std::string &message, Message type = Message::info);

protected:
    std::string m_name;
};

} /* namespace rmpl */

#endif /* BASE_H_ */
