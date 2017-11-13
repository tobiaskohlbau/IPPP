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

#ifndef WRITER_H
#define WRITER_H

#include <fstream>
#include <type_traits>
#include <vector>

#include <Eigen/Core>

#include <ippp/types.h>

namespace ippp {
namespace writer {

/*!
*  \brief      Write configs to the defined file, clear file
*  \param[in]  vector of Vec
*  \param[in]  filename
*  \param[in]  scale
*  \author     Sasch Kaden
*  \date       2016-11-14
*/
template <unsigned int dim>
void writeVecsToFile(const std::vector<Vector<dim>> &configs, const std::string &filename, double scale) {
    std::ofstream myfile(filename);
    for (int i = 0; i < configs.size(); ++i) {
        for (unsigned int j = 0; j < dim; ++j)
            myfile << configs[i][j] * scale << " ";
        myfile << std::endl;
    }
    myfile.close();
}

/*!
*  \brief      Append configs to defined file
*  \param[in]  vector of Vec
*  \param[in]  filename
*  \param[in]  scale
*  \author     Sasch Kaden
*  \date       2016-11-14
*/
template <unsigned int dim>
void appendVecsToFile(const std::vector<Vector<dim>> &configs, const std::string &filename, double scale) {
    std::ofstream myfile;
    myfile.open(filename, std::ios_base::app);
    for (int i = 0; i < configs.size(); ++i) {
        for (unsigned int j = 0; j < dim; ++j)
            myfile << configs[i][j] * scale << " ";
        myfile << std::endl;
    }
    myfile.close();
}

/*!
*  \brief      Write transformation vectors to defined file
*  \param[in]  vector of vector of Vec
*  \param[in]  filename
*  \author     Sasch Kaden
*  \date       2016-11-14
*/
static void writeTrafosToFile(const std::vector<std::vector<Vector6>> &configs, const std::string &filename) {
    std::ofstream myfile;
    myfile.open(filename, std::ios_base::app);
    for (auto trafos : configs) {
        for (auto trafo : trafos) {
            for (unsigned int i = 0; i < trafo.cols(); ++i) {
                myfile << trafo[i] << " ";
            }
        }
        myfile << std::endl;
    }
    myfile.close();
}

} /* namespace utilList */
} /* namespace ippp */

#endif    // WRITER_H
