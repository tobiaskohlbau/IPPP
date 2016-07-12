#ifndef CADFILELOADER_H_
#define CADFILELOADER_H_

#include <string>
#include <vector>

#include <PQP/include/PQP.h>

#include <core/Base.h>

namespace rmpl {

/*!
* \brief   Class for loading of cad files
* \author  Sascha Kaden
* \date    2016-07-12
*/
class CadFileLoader : public Base
{
public:
    CadFileLoader();
    std::shared_ptr<PQP_Model> loadFile(const std::string filename);

protected:
    std::shared_ptr<PQP_Model> readObj(const std::string filename);
    std::string getFileExt(const std::string& s);
};

} /* namespace rmpl */

#endif /* CADFILELOADER_H_ */
