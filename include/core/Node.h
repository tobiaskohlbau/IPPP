#ifndef NODE_H_
#define NODE_H_

#include <assert.h>
#include <vector>

#include <core/Base.h>
#include <core/Edge.h>
#include <core/Vec.hpp>

namespace rmpl {

/*!
* \brief   Class Node to present nodes of path planner
* \details Consists of the position by an Vec, a cost parameter, pointer to an parent Node and a list of child
* \author  Sascha Kaden
* \date    2016-05-23
*/
class Node : public Base
{
public:
    Node();
    Node(float x);
    Node(float x, float y);
    Node(float x, float y, float z);
    Node(float x, float y, float z, float rx);
    Node(float x, float y, float z, float rx, float ry);
    Node(float x, float y, float z, float rx, float ry, float rz);
    Node(float x, float y, float z, float rx, float ry, float rz, float wx);
    Node(float x, float y, float z, float rx, float ry, float rz, float wx, float wy);
    Node(float x, float y, float z, float rx, float ry, float rz, float wx, float wy, float wz);
    Node(const Vec<float> &vec);

    float getX();
    float getY();
    float getZ();

    unsigned int getDim();
    bool empty() const ;
    void setVecValue(float value, unsigned int index);
    float getVecValue(unsigned int index);
    float getDist(const std::shared_ptr<Node> &node);
    float getDist(const Node &node);
    float getDistToParent();
    float norm();

    void setCost(float cost);
    float getCost();

    void setParent(const std::shared_ptr<Node> &parent);
    std::shared_ptr<Node> getParent();
    void clearParent();
    void addChild(const std::shared_ptr<Node> &child);
    std::vector<std::shared_ptr<Node>> getChilds();
    void clearChilds();

    Vec<float> getVec() const;

private:
    float        m_cost;
    Vec<float>   m_vec;
    std::shared_ptr<Node>              m_parent;
    std::vector<std::shared_ptr<Node>> m_childs;
    Edge m_edge;
};

} /* namespace rmpl */

#endif /* NODE_H_ */
