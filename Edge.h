#ifndef EDGE_H_
#define EDGE_H_

#include <cstdint>
#include <memory>

class Node;

using std::shared_ptr;

/*!
* \brief   Class Edge contains the two nodes of the Edge and different parameters
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Edge
{
public:
    Edge();
    Edge(const shared_ptr<Node> &source, const shared_ptr<Node> &target);

    float getLength() const;

    void setSource(const shared_ptr<Node> &source);
    void setTarget (const shared_ptr<Node> &target);

    shared_ptr<Node> getSource() const;
    shared_ptr<Node> getTarget() const;

private:
    shared_ptr<Node> m_source;
    shared_ptr<Node> m_target;
    float m_length;
};

#endif /* EDGE_H_ */
