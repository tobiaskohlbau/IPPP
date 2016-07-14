#include <core/Base.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class Base
*  \author     Sasch Kaden
*  \param[in]  name
*  \date       2016-05-30
*/
Base::Base(const std::string &name) {
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
*  \param[in]  message type
*  \date       2016-07-14
*/
void Base::sendMessage(const std::string &message, Message type) {
    if (type == Message::warning)
        std::cout << "Warning - ";
    else if (type == Message::error)
        std::cout << "Error - ";
    else
        std::cout << "Info - ";

    std::cout << m_name << ": ";
    std::cout << message << std::endl;
}
