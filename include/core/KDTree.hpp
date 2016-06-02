#ifndef KDTREE_H_
#define KDTREE_H_

#include <core/Base.h>
#include <core/KDNode.hpp>
#include <core/Node.h>

namespace rmpl {

/*!
* \brief   Class KDTree for a fast binary search
* \details Class uses KDNode to save the points
* \author  Sascha Kaden
* \date    2016-05-27
*/
template<class T>
class KDTree : public Base
{
public:
    KDTree();
    void addNode(const Vec<float> &vec, T node);
    T searchNearestNeighbor(const Vec<float> &vec);
    std::vector<T> searchRange(const Vec<float> &vec, const float &range);

private:
    std::shared_ptr<KDNode<T>> insert(std::shared_ptr<KDNode<T>> insertNode, std::shared_ptr<KDNode<T>> currentNode, unsigned int depth);
    void NNS(const Vec<float> &point, std::shared_ptr<KDNode<T>> node, std::shared_ptr<KDNode<T>> &refNode, float &bestDist);
    void RS(const Vec<float> &point, std::shared_ptr<KDNode<T>> node, std::vector<std::shared_ptr<KDNode<T>>> &refNodes,
        const float &sqRange, const Vec<float> &maxBoundary, const Vec<float> &minBoundary);

    std::shared_ptr<KDNode<T>> m_root;
};

/*!
*  \brief      Default constructor of the class KDTree
*  \author     Sascha Kaden
*  \date       2016-06-02
*/
template<class T>
KDTree<T>::KDTree()
    : Base("KD Tree") {

}

/*!
*  \brief      Add Node to the KDTree
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  pointer to the Node
*  \date       2016-05-27
*/
template<class T>
void KDTree<T>::addNode(const Vec<float> &vec, T node) {
    std::shared_ptr<KDNode<T>> shrKDNode(new KDNode<T>(vec, node));
    if (m_root == nullptr) {
        m_root = shrKDNode;
        return;
    }
    insert(shrKDNode, m_root, 0);
}

/*!
*  \brief      Insert KDNode to the KDTree (recursive function)
*  \author     Sascha Kaden
*  \param[in]  KDNode to insert
*  \param[in]  current KDNode in the KDTree
*  \param[in]  split dimension
*  \param[out] current KDNode
*  \date       2016-05-27
*/
template<class T>
std::shared_ptr<KDNode<T>> KDTree<T>::insert(std::shared_ptr<KDNode<T>> insertNode, std::shared_ptr<KDNode<T>> currentNode, unsigned int cd) {
    if (currentNode == nullptr) {              // node at leaf doesn't exist and will be added
        currentNode = insertNode;
        currentNode->axis = cd;
        currentNode->value = currentNode->vec[cd];
    }
    else if (insertNode == currentNode) {   // error! duplicate
        return currentNode;
    }
    else if (insertNode->vec[cd] < currentNode->vec[cd]) {
        currentNode->left = insert(insertNode, currentNode->left, (cd+1) % insertNode->getDim());
    }
    else {
        currentNode->right = insert(insertNode, currentNode->right, (cd+1) % insertNode->getDim());
    }
    return currentNode;
}

/*!
*  \brief      Search for the nearest neighbor
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[out] pointer to the nearest Node
*  \date       2016-05-27
*/
template<class T>
T KDTree<T>::searchNearestNeighbor(const Vec<float> &vec) {
    if (m_root == nullptr)
        return nullptr;

    std::shared_ptr<KDNode<T>> node;
    float bestDist = std::numeric_limits<float>::max();
    NNS(vec, m_root, node, bestDist);
    return node->node;
}

/*!
*  \brief      Search for range around a position
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  distance of the range
*  \param[out] list of near nodes to the position
*  \date       2016-05-27
*/
template<class T>
std::vector<T> KDTree<T>::searchRange(const Vec<float> &vec, const float &range) {
    std::vector<T> nodes;
    if (m_root == nullptr)
        return nodes;

    std::vector<std::shared_ptr<KDNode<T>>> kdNodes;
    float sqRange = range * range;
    Vec<float> maxBoundary = vec + range;
    Vec<float> minBoundary = vec - range;
    RS(vec, m_root, kdNodes, sqRange, maxBoundary, minBoundary);

    for (int i = 0; i < kdNodes.size(); ++i)
        nodes.push_back(kdNodes[i]->node);
    return nodes;
}

/*!
*  \brief      Search for the nearest neighbor (recursive function)
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  reference KDNode
*  \param[in]  shortest distance
*  \date       2016-05-27
*/
template<class T>
void KDTree<T>::NNS(const Vec<float> &vec, std::shared_ptr<KDNode<T>> node, std::shared_ptr<KDNode<T>> &refNode, float &bestDist) {
    if (node->left == nullptr && node->right == nullptr) {
        float dist = vec.getSqDist(node->vec);
        if (dist < bestDist) {
            bestDist = dist;
            refNode = node;
        }
    }
    else {
        if (vec[node->axis] <= node->value) {
            if (vec[node->axis] - bestDist <= node->value && node->left != nullptr)
                NNS(vec, node->left, refNode, bestDist);
            if (vec[node->axis] + bestDist > node->value && node->right != nullptr)
                NNS(vec, node->right, refNode, bestDist);
        }
        else {
            if (vec[node->axis] + bestDist > node->value && node->right != nullptr)
                NNS(vec, node->right, refNode, bestDist);
            if (vec[node->axis] - bestDist <= node->value && node->left != nullptr)
                NNS(vec, node->left, refNode, bestDist);
        }
    }
}

/*!
*  \brief      Search range for near nodes (recursive function)
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  list of reference kdNodes
*  \param[in]  squared range distance
*  \param[in]  maximum boundary
*  \param[in]  minimum boundary
*  \date       2016-05-27
*/
template<class T>
void KDTree<T>::RS(const Vec<float> &vec, std::shared_ptr<KDNode<T>> node, std::vector<std::shared_ptr<KDNode<T>>> &refNodes,
    const float &sqRange, const Vec<float> &maxBoundary, const Vec<float> &minBoundary) {
    if (node == nullptr) {
        return;
    }
    if (vec.getSqDist(node->vec) < sqRange) {
        refNodes.push_back(node);
    }
    if (vec[node->axis] < maxBoundary[node->axis] && vec[node->axis] < minBoundary[node->axis]) {
        RS(vec, node->left, refNodes, sqRange, maxBoundary, minBoundary);
    }
    else if (vec[node->axis] > maxBoundary[node->axis] && vec[node->axis] > minBoundary[node->axis]) {
        RS(vec, node->right, refNodes, sqRange, maxBoundary, minBoundary);
    }
    else {
        RS(vec, node->left, refNodes, sqRange, maxBoundary, minBoundary);
        RS(vec, node->right, refNodes, sqRange, maxBoundary, minBoundary);
    }
}

} /* namespace rmpl */

#endif /* KDTREE_H_ */
