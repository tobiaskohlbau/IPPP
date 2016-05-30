#ifndef NODE_H_
#define NODE_H_

#include <assert.h>
#include <cstdint>
#include <memory>
#include <vector>

#include "Vec.h"
#include <core/Edge.h>

using std::shared_ptr;


/*!
* \brief   Class Node to present nodes of path planner
* \details Consists of the position by an Vec, a cost parameter, pointer to an parent Node and a list of child
* \author  Sascha Kaden
* \date    2016-05-23
*/
class Node
{
public:
    Node();
    Node(const float &x);
    Node(const float &x, const float &y);
    Node(const float &x, const float &y, const float &z);
    Node(const float &x, const float &y, const float &z, const float &rx);
    Node(const float &x, const float &y, const float &z, const float &rx, const float &ry);
    Node(const float &x, const float &y, const float &z, const float &rx, const float &ry, const float &rz);
    Node(const Vec<float> &vec);

    float getX() const;
    float getY() const;
    float getZ() const;

    unsigned int getDim();
    bool empty() const ;
    void setVecValue(const float &value, const unsigned int &index);
    float getVecValue(const unsigned int &index);
    float getDist(const shared_ptr<Node> &node) const;
    float getDist(const Node &node) const;
    float getDistToParent() const;
    float norm() const;

    void setCost(const float &cost);
    float getCost() const;

    void setParent(const shared_ptr<Node> &parent);
    shared_ptr<Node> getParent();
    void clearParent();
    void addChild(const shared_ptr<Node> &child);
    std::vector<shared_ptr<Node>> getChilds();
    void clearChilds();

    Vec<float> getVec() const;

private:
    float        m_cost;
    Vec<float>   m_vec;
    shared_ptr<Node>              m_parent;
    std::vector<shared_ptr<Node>> m_childs;
    Edge m_edge;
};

#endif /* NODE_H_ */
