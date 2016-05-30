#include <core/Edge.h>
#include <core/Node.h>

/*!
*  \brief      Constructor of the class Edge
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \date       2016-05-25
*/
Edge::Edge(const shared_ptr<Node> &source, const shared_ptr<Node> &target) {
    m_source = source;
    m_target = target;
    m_length = m_source->getDist(m_target);
}

/*!
*  \brief      Return legth of the Edge
*  \author     Sascha Kaden
*  \param[out] length of the Edge
*  \date       2016-05-25
*/
float Edge::getLength() const {
    return 0;//return m_source->getDist(m_target);
}

/*!
*  \brief      Set source Node of the Edge
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \date       2016-05-25
*/
void Edge::setSource(const shared_ptr<Node> &source) {
    m_source = source;
    m_length = m_source->getDist(m_target);
}

/*!
*  \brief      Set target Node of the Edge
*  \author     Sascha Kaden
*  \param[in]  target Node
*  \date       2016-05-25
*/
void Edge::setTarget(const shared_ptr<Node> &target) {
    m_target = target;
    m_length = m_source->getDist(m_target);
}

/*!
*  \brief      Return source Node of the Edge
*  \author     Sascha Kaden
*  \param[out] source Node
*  \date       2016-05-25
*/
shared_ptr<Node> Edge::getSource() const {
    return m_source;
}

/*!
*  \brief      Return target Node of the Edge
*  \author     Sascha Kaden
*  \param[out] target Node
*  \date       2016-05-25
*/
shared_ptr<Node> Edge::getTarget() const {
    return m_target;
}
