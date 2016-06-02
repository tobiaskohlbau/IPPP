#include <core/Base.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class Base
*  \author     Sasch Kaden
*  \param[in]  name
*  \date       2016-05-30
*/
Base::Base(std::string name) {
    m_name = name;
}

/*!
*  \brief      Return name
*  \author     Sasch Kaden
*  \param[out] name
*  \date       2016-05-30
*/
std::string Base::getName() {
    return m_name;
}

/*!
*  \brief      Send message to the console with the name
*  \author     Sasch Kaden
*  \param[in]  message
*  \date       2016-05-30
*/
void Base::sendMessage(std::string message) {
    std::cout << m_name << ": ";
    std::cout << message << std::endl;
}
