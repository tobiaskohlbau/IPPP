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

#include "Writer.h"

#include <fstream>

using namespace rmpl;

/*!
*  \brief      Write vecs to defined file, clear file
*  \param[in]  vector of Vec
*  \param[in]  filename
*  \param[in]  scale
*  \author     Sasch Kaden
*  \date       2016-11-14
*/
void Writer::writeVecsToFile(const std::vector<Vec<float>> &vecs, const std::string &filename, float scale) {
    std::ofstream myfile(filename);
    for (int i = 0; i < vecs.size(); ++i) {
        for (unsigned int j = 0; j < vecs[i].getDim(); ++j)
            myfile << vecs[i][j] * scale << " ";
        myfile << std::endl;
    }
    myfile.close();
}

/*!
*  \brief      Append vecs to defined file
*  \param[in]  vector of Vec
*  \param[in]  filename
*  \param[in]  scale
*  \author     Sasch Kaden
*  \date       2016-11-14
*/
void Writer::appendVecsToFile(const std::vector<Vec<float>> &vecs, const std::string &filename, float scale) {
    std::ofstream myfile;
    myfile.open(filename, std::ios_base::app);
    for (int i = 0; i < vecs.size(); ++i) {
        for (unsigned int j = 0; j < vecs[i].getDim(); ++j)
            myfile << vecs[i][j] * scale << " ";
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
void Writer::writeTrafosToFile(const std::vector<std::vector<Vec<float>>> &vecs, const std::string &filename) {
    std::ofstream myfile;
    myfile.open(filename, std::ios_base::app);
    for (auto trafos : vecs) {
        for (auto trafo : trafos) {
            for (unsigned int i = 0; i < trafo.getDim(); ++i) {
                myfile << trafo[i] << " ";
            }
        }
        myfile << std::endl;
    }
    myfile.close();
}
