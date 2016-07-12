#include <robot/CadFileLoader.h>

#include <fstream>
#include <sstream>

using namespace rmpl;

/*!
*  \brief      Constructor of the class CadFileLoader
*  \author     Sascha Kaden
*  \date       2016-07-12
*/
CadFileLoader::CadFileLoader()
    : Base("CadFileLoader") {
}


std::shared_ptr<PQP_Model> CadFileLoader::loadFile(const std::string filename) {
    std::shared_ptr<PQP_Model> model;

    std::string extension = getFileExt(filename);

    if (extension == "obj")
        return readObj(filename);

    return model;
}


std::shared_ptr<PQP_Model> CadFileLoader::readObj(const std::string filename) {
    std::shared_ptr<PQP_Model> model(new PQP_Model());

    std::ifstream input(filename);

    PQP_REAL value;
    std::vector<PQP_REAL> vertice;
    std::vector<std::vector<PQP_REAL>> vertices;
    int num;
    std::vector<int> face;
    std::vector<std::vector<int>> faces;

    for (std::string line; getline(input, line); )
    {
        if (line.at(0) == 'v') {
            line = line.substr(2);
            std::stringstream iss(line);
            vertice.clear();
            for (int i = 0; i < 3; ++i) {
                iss >> value;
                vertice.push_back(value);
            }

            vertices.push_back(vertice);
        }
        else if (line.at(0) == 'f') {
            line = line.substr(2);
            std::stringstream iss(line);
            face.clear();
            for (int i = 0; i < 3; ++i) {
                iss >> num;
                face.push_back(num-1);
            }
            faces.push_back(face);
        }
    }

    model->BeginModel();
    // create pqp triangles
    PQP_REAL p[3][3];
    for (int i = 0; i < faces.size(); ++i) {
        // go through faces
        for (int j = 0; j < 3; ++j) {
            // go through face
            int vert = faces[i][j];
            for (int k = 0; k < 3; ++k) {
                p[j][k] = vertices[vert][k];
            }

        }
        model->AddTri(p[0], p[1], p[2], i);
    }
    model->EndModel();
    model->MemUsage(1);

    return model;
}


std::string CadFileLoader::getFileExt(const std::string& s) {
    size_t i = s.rfind('.', s.length());

    if (i != std::string::npos) {
        std::string ext = s.substr(i+1, s.length() - i);
        for (std::basic_string<char>::iterator p = ext.begin(); p != ext.end(); ++p)
           *p = tolower(*p);
        return ext;
    }

    return("");
}
