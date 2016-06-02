#ifndef BASE_H_
#define BASE_H_

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

namespace rmpl{

class Base
{
public:
    Base(std::string name);
    std::string getName();
    void sendMessage(std::string message);

protected:
    std::string m_name;
};

} /* namespace rmpl */

#endif /* BASE_H_ */
