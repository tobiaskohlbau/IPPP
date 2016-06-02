#ifndef BASE_H_
#define BASE_H_

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

namespace rmpl{

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
    void sendMessage(const std::string &message);

protected:
    std::string m_name;
};

} /* namespace rmpl */

#endif /* BASE_H_ */
