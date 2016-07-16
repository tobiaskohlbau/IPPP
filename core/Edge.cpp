#include <core/Edge.h>
#include <core/Node.h>

using std::shared_ptr;
using namespace rmpl;

/*!
*  \brief      Default constructor of the class Edge
*  \author     Sasch Kaden
*  \date       2016-05-30
*/
Edge::Edge()
    : Base("Edge") {
}

/*!
*  \brief      Constructor of the class Edge
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \date       2016-05-25
*/
Edge::Edge(const shared_ptr<Node> &target, float length)
        : Base("Edge") {
    m_target = target;
    m_length = length;
}

/*!
*  \brief      Return legth of the Edge
*  \author     Sascha Kaden
*  \param[out] length of the Edge
*  \date       2016-05-25
*/
float Edge::getLength() {
    return m_length;
}

/*!
*  \brief      Set target Node of the Edge
*  \author     Sascha Kaden
*  \param[in]  target Node
*  \date       2016-05-25
*/
void Edge::setTarget(const shared_ptr<Node> &target, float length) {
    m_target = target;
}

/*!
*  \brief      Return target Node of the Edge
*  \author     Sascha Kaden
*  \param[out] target Node
*  \date       2016-05-25
*/
shared_ptr<Node> Edge::getTarget() {
    return m_target;
}
