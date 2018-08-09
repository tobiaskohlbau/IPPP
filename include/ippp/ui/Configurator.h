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

#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H

#include <string>
#include <vector>

#include <json.hpp>

#include <ippp/Identifier.h>
#include <ippp/types.h>
#include <ippp/ui/FileWriterReader.h>

namespace ippp {

/*!
* \brief   Class Configurator is the base class to load and save json files.
* \author  Sascha Kaden
* \date    2017-10-16
*/
class Configurator : public Identifier {
  public:
    Configurator(const std::string &name);

  protected:
    template <unsigned int dim>
    std::string vectorToString(const Vector<dim> &vector);
    template <unsigned int dim>
    Vector<dim> stringToVector(const std::string &string);
    std::vector<double> eigenToStdVector(const VectorX vec);
    VectorX stdVectorToEigen(std::vector<double> vec);
};

/*!
*  \brief      Converts a Vector to a string
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \param[out] string
*  \date       2017-10-16
*/
template <unsigned int dim>
std::string Configurator::vectorToString(const Vector<dim> &vector) {
    std::string string;
    for (unsigned int i = 0; i < dim; ++i)
        string += std::to_string(vector[i]) + " ";
    return string;
}

/*!
*  \brief      Converts a string to a Vector
*  \author     Sascha Kaden
*  \param[in]  string
*  \param[out] Vector
*  \date       2017-10-16
*/
template <unsigned int dim>
Vector<dim> Configurator::stringToVector(const std::string &string) {
    Vector<dim> vector;
    std::stringstream stream(string);
    for (unsigned int i = 0; i < dim; ++i)
        stream >> vector[i];

    return vector;
}

} /* namespace ippp */

#endif    // CONFIGURATOR_H
