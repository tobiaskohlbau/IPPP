#ifndef KDNODE_H_
#define KDNODE_H_

#include <cstdint>

#include <core/Vec.hpp>

namespace rmpl {

/*!
* \brief   Class KDNode is the node object of the KDTree
* \details Owns the position of the node, pointer to left and right leaf, pointer to an extern object, axis split value and the split value itself.
* \author  Sascha Kaden
* \date    2016-05-27
*/
template<typename T>
class KDNode
{
public:
    KDNode(const Vec<float> &vec, const T &node);
    unsigned int getDim();

    std::shared_ptr<KDNode<T>> left;
    std::shared_ptr<KDNode<T>> right;
    Vec<float> vec;
    T node;
    unsigned int axis;
    float value;
};

/*!
*  \brief      Constructor of the class KDNode
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  pointer to an extern object
*  \date       2016-05-27
*/
template<typename T>
KDNode<T>::KDNode(const Vec<float> &vec, const T &node) {
    this->vec = vec;
    this->node = node;
    axis = 0;
    value = 0;
}

/*!
*  \brief      Return dimension of KDNode
*  \author     Sascha Kaden
*  \param[out] dimension
*  \date       2016-05-27
*/
template<typename T>
unsigned int KDNode<T>::getDim() {
    return vec.getDim();
}

} /* namespace rmpl */

#endif /* KDNODE_H_ */
