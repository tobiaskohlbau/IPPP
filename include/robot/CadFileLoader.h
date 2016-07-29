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

#ifndef CADFILELOADER_H_
#define CADFILELOADER_H_

#include <vector>

#include <PQP.h>

#include <core/Base.h>

namespace rmpl {

/*!
* \brief   Class for loading of cad files
* \author  Sascha Kaden
* \date    2016-07-12
*/
class CadFileLoader : public Base {
  public:
    CadFileLoader();
    std::shared_ptr<PQP_Model> loadFile(const std::string filename);

  protected:
    std::shared_ptr<PQP_Model> readObj(const std::string filename);
    std::string getFileExt(const std::string& s);
};

} /* namespace rmpl */

#endif /* CADFILELOADER_H_ */
