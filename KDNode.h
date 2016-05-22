#ifndef KDNODE_H_
#define KDNODE_H_

#include <cstdint>

using std::shared_ptr;

template<unsigned int dim, class T>
class KDNode
{
public:
    KDNode(Vec<dim, float> vec, T node);

    shared_ptr<KDNode<dim, T>> left;
    shared_ptr<KDNode<dim, T>> right;
    Vec<dim, float> vec;
    T node;
    unsigned int axis;
    float value;
private:

};

template<unsigned int dim, class T>
KDNode<dim, T>::KDNode(Vec<dim, float> vec, T node) {
    this->vec = vec;
    this->node = node;
    axis = 0;
    value = 0;
}

#endif /* KDNODE_H_ */
