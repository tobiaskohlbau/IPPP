#ifndef EDGE_H_
#define EDGE_H_

#include <cstdint>
#include <memory>

class Node;



/*!
* \brief   Class Edge contains the two nodes of the Edge and different parameters
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Edge
{
public:
    Edge();
    Edge(const std::shared_ptr<Node> &source, const std::shared_ptr<Node> &target);

    float getLength() const;

    void setSource(const std::shared_ptr<Node> &source);
    void setTarget (const std::shared_ptr<Node> &target);

    std::shared_ptr<Node> getSource() const;
    std::shared_ptr<Node> getTarget() const;

private:
    std::shared_ptr<Node> m_source;
    std::shared_ptr<Node> m_target;
    float m_length;
};

#endif /* EDGE_H_ */
