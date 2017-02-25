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

#ifndef MODULEBASE_H
#define MODULEBASE_H

#include <string>

namespace rmpl {

/*!
* \brief   Base class of all modules, at the time it is an identifier.
* \author  Sascha Kaden
* \date    2016-06-02
*/
class Identifier {
  public:
    virtual ~Identifier();

  protected:
    Identifier(const std::string& name);

  public:
    const std::string& getName();

  private:
    const std::string m_name;
};

} /* namespace rmpl */

#endif /* BASE_H */
