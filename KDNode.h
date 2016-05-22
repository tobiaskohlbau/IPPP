#ifndef KDNODE_H_
#define KDNODE_H_

#include <cstdint>

using std::shared_ptr;

template<typename T>
class KDNode
{
public:
    KDNode(Vec<float> vec, T node);
    unsigned int getDim();

    shared_ptr<KDNode<T>> left;
    shared_ptr<KDNode<T>> right;
    Vec<float> vec;
    T node;
    unsigned int axis;
    float value;
private:

};

template<typename T>
KDNode<T>::KDNode(Vec<float> vec, T node) {
    this->vec = vec;
    this->node = node;
    axis = 0;
    value = 0;
}

template<typename T>
unsigned int KDNode<T>::getDim() {
    return vec.getDim();
}

#endif /* KDNODE_H_ */
