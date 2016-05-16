#ifndef KDTREE_H_
#define KDTREE_H_

#include <cstdint>

#include "KDNode.h"
#include "Node.h"

using std::shared_ptr;

template<uint16_t dim, class T>
class KDTree
{
public:
    void addNode(const Vec<dim, float> &vec, T node);
    T searchNearestNeighbor(const Vec<dim, float> &vec);
    std::vector<T> searchRange(const Vec<dim, float> &vec, const float &range);

private:
    shared_ptr<KDNode<dim, T>> insert(shared_ptr<KDNode<dim, T>> insertNode, shared_ptr<KDNode<dim, T>> currentNode, uint16_t depth);
    void NNS(const Vec<dim, float> &point, shared_ptr<KDNode<dim, T>> node, shared_ptr<KDNode<dim, T>> &refNode, float &bestDist);
    void RS(const Vec<dim, float> &point, shared_ptr<KDNode<dim, T>> node, std::vector<shared_ptr<KDNode<dim, T>>> &refNodes,
        const float &sqRange, const Vec<dim, float> &maxBoundary, const Vec<dim, float> &minBoundary);
    //variables
    shared_ptr<KDNode<dim, T>> m_root;
};

template<uint16_t dim, class T>
void KDTree<dim, T>::addNode(const Vec<dim, float> &vec, T node) {
    shared_ptr<KDNode<dim, T>> shrKDNode(new KDNode<dim, T>(vec, node));
    if (m_root == NULL) {
        m_root = shrKDNode;
        return;
    }
    insert(shrKDNode, m_root, 0);
}

template<uint16_t dim, class T>
shared_ptr<KDNode<dim, T>> KDTree<dim, T>::insert(shared_ptr<KDNode<dim, T>> insertNode, shared_ptr<KDNode<dim, T>> currentNode, uint16_t cd) {
    if (currentNode == NULL) {              // node at leaf doesn't exist and will be added
        currentNode = insertNode;
        currentNode->axis = cd;
        currentNode->value = currentNode->vec[cd];
    }
    else if (insertNode == currentNode) {   // error! duplicate
        return currentNode;
    }
    else if (insertNode->vec[cd] < currentNode->vec[cd]) {
        currentNode->left = insert(insertNode, currentNode->left, (cd+1) % dim);
    }
    else {
        currentNode->right = insert(insertNode, currentNode->right, (cd+1) % dim);
    }
    return currentNode;
}

template<uint16_t dim, class T>
T KDTree<dim, T>::searchNearestNeighbor(const Vec<dim, float> &vec) {
    if (m_root == NULL)
        return NULL;

    shared_ptr<KDNode<dim, T>> node;
    float bestDist = std::numeric_limits<float>::max();
    NNS(vec, m_root, node, bestDist);
    return node->node;
}

template<uint16_t dim, class T>
std::vector<T> KDTree<dim, T>::searchRange(const Vec<dim, float> &vec, const float &range) {
    std::vector<T> nodes;
    if (m_root == NULL)
        return nodes;

    std::vector<shared_ptr<KDNode<dim, T>>> kdNodes;
    float sqRange = range * range;
    Vec<dim, float> maxBoundary = vec + range;
    Vec<dim, float> minBoundary = vec - range;
    RS(vec, m_root, kdNodes, sqRange, maxBoundary, minBoundary);

    for (int i = 0; i < kdNodes.size(); ++i)
        nodes.push_back(kdNodes[i]->node);
    return nodes;
}

template<uint16_t dim, class T>
void KDTree<dim, T>::NNS(const Vec<dim, float> &vec, shared_ptr<KDNode<dim, T>> node, shared_ptr<KDNode<dim, T>> &refNode, float &bestDist) {
    if (node->left == NULL && node->right == NULL) {
        float dist = vec.getSqDist(node->vec);
        if (dist < bestDist) {
            bestDist = dist;
            refNode = node;
        }
    }
    else {
        if (vec[node->axis] <= node->value) {
            if (vec[node->axis] - bestDist <= node->value && node->left != NULL)
                NNS(vec, node->left, refNode, bestDist);
            if (vec[node->axis] + bestDist > node->value && node->right != NULL)
                NNS(vec, node->right, refNode, bestDist);
        }
        else {
            if (vec[node->axis] + bestDist > node->value && node->right != NULL)
                NNS(vec, node->right, refNode, bestDist);
            if (vec[node->axis] - bestDist <= node->value && node->left != NULL)
                NNS(vec, node->left, refNode, bestDist);
        }
    }
}

template<uint16_t dim, class T>
void KDTree<dim, T>::RS(const Vec<dim, float> &vec, shared_ptr<KDNode<dim, T>> node, std::vector<shared_ptr<KDNode<dim, T>>> &refNodes,
    const float &sqRange, const Vec<dim, float> &maxBoundary, const Vec<dim, float> &minBoundary) {
    if (node == NULL) {
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

#endif /* KDTREE_H_ */
