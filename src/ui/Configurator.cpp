//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#include <ippp/ui/Configurator.h>

#include <fstream>

#include <ippp/util/Logging.h>

namespace ippp {

/*!
*  \brief      Standard constructor of the Configurator
*  \author     Sascha Kaden
*  \param[in]  name
*  \date       2017-10-16
*/
Configurator::Configurator(const std::string &name) : Identifier(name) {
}

std::vector<double> Configurator::eigenToStdVector(const VectorX vec) {
    return std::vector<double>(vec.data(), vec.data() + vec.size());
}

VectorX Configurator::stdVectorToEigen(std::vector<double> vec) {
    VectorX eigenVec(vec.size());
    for (size_t i = 0; i < vec.size(); ++i)
        eigenVec[i] = vec[i];
    return eigenVec;
}


} /* namespace ippp */