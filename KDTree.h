#ifndef KDTREE_H_
#define KDTREE_H_

#include <cstdint>

#include "KDNode.h"
#include "Node.h"

template<uint16_t dim, class T>
class KDTree
{
public:
    void addNode(const Vec<dim, float> &vec, T node);
    T getNearestNeighbor(const Vec<dim, float> &vec);

private:
    std::shared_ptr<KDNode<dim, T>> insert(std::shared_ptr<KDNode<dim, T>> insertNode, std::shared_ptr<KDNode<dim, T>> currentNode, uint16_t depth);
    void NNS(const Vec<dim, float> &point, std::shared_ptr<KDNode<dim, T>> node, std::shared_ptr<KDNode<dim, T>> &refNode, float &bestDist);

    //variables
    std::shared_ptr<KDNode<dim, T>> m_root;
};

template<uint16_t dim, class T>
void KDTree<dim, T>::addNode(const Vec<dim, float> &vec, T node) {
    std::shared_ptr<KDNode<dim, T>> shrKDNode(new KDNode<dim, T>(vec, node));
    if (m_root == NULL) {
        m_root = shrKDNode;
        return;
    }
    insert(shrKDNode, m_root, 0);
}

template<uint16_t dim, class T>
std::shared_ptr<KDNode<dim, T>> KDTree<dim, T>::insert(std::shared_ptr<KDNode<dim, T>> insertNode, std::shared_ptr<KDNode<dim, T>> currentNode, uint16_t cd) {
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
T KDTree<dim, T>::getNearestNeighbor(const Vec<dim, float> &vec) {
    if (m_root == NULL)
        return NULL;

    std::shared_ptr<KDNode<dim, T>> node;
    float bestDist = std::numeric_limits<float>::max();
    NNS(vec, m_root, node, bestDist);
    return node->node;
}

template<uint16_t dim, class T>
void KDTree<dim, T>::NNS(const Vec<dim, float> &vec, std::shared_ptr<KDNode<dim, T>> node, std::shared_ptr<KDNode<dim, T>> &refNode, float &bestDist) {
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



#endif /* KDTREE_H_ */
