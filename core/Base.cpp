#include <core/Base.h>

using namespace rmpl;

Base::Base(std::string name) {
    m_name = name;
}

std::string Base::getName() {
    return m_name;
}

void Base::sendMessage(std::string message) {
    std::cout << m_name << ": ";
    std::cout << message << std::endl;
}
