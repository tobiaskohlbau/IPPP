//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

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
template <class T>
class KDTree : public Base {
  public:
    KDTree();
    KDTree(std::vector<std::shared_ptr<Node>> &vec);
    void addNode(const Vec<float> &vec, const T &node);
    T searchNearestNeighbor(const Vec<float> &vec);
    std::vector<T> searchRange(const Vec<float> &vec, float range);

    void buildTree();
    std::shared_ptr<KDNode<T>> sort(std::vector<std::shared_ptr<Node>> &vec, unsigned int dim);
    void quickSort(std::vector<std::shared_ptr<Node>> &A, int left, int right, int dim);
    int partition(std::vector<std::shared_ptr<Node>> &A, int left, int right, int dim);

  private:
    std::shared_ptr<KDNode<T>> insert(std::shared_ptr<KDNode<T>> insertNode, std::shared_ptr<KDNode<T>> currentNode,
                                      unsigned int depth);
    void NNS(const Vec<float> &point, std::shared_ptr<KDNode<T>> node, std::shared_ptr<KDNode<T>> &refNode, float &bestDist);
    void RS(const Vec<float> &point, std::shared_ptr<KDNode<T>> node, std::vector<std::shared_ptr<KDNode<T>>> &refNodes,
            float sqRange, const Vec<float> &maxBoundary, const Vec<float> &minBoundary);

    std::shared_ptr<KDNode<T>> m_root;
};

/*!
*  \brief      Default constructor of the class KDTree
*  \author     Sascha Kaden
*  \date       2016-06-02
*/
template <class T>
KDTree<T>::KDTree() : Base("KD Tree") {
}

/*!
*  \brief      Cconstructor of the class KDTree, builds a sorted tree with the passed nodes
*  \author     Sascha Kaden
*  \param[in]  vector of nodes
*  \date       2016-07-18
*/
template <class T>
KDTree<T>::KDTree(std::vector<std::shared_ptr<Node>> &vec) : Base("KD Tree") {
    quickSort(vec, 0, vec.size() - 1, 0);
    m_root = std::shared_ptr<KDNode<T>>(new KDNode<T>(vec[vec.size() / 2]->getVec(), vec[vec.size() / 2]));
    m_root->axis = 0;
    m_root->value = m_root->vec[0];

    std::vector<std::shared_ptr<Node>> vecLeft(vec.begin(), vec.begin() + (vec.size() / 2) - 1);
    std::vector<std::shared_ptr<Node>> vecRight(vec.begin() + (vec.size() / 2) + 1, vec.end());
    m_root->left = sort(vecLeft, 1);
    m_root->right = sort(vecRight, 1);
}

/*!
*  \brief      Add Node to the KDTree
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  pointer to the Node
*  \date       2016-05-27
*/
template <class T>
void KDTree<T>::addNode(const Vec<float> &vec, const T &node) {
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
template <class T>
std::shared_ptr<KDNode<T>> KDTree<T>::insert(std::shared_ptr<KDNode<T>> insertNode, std::shared_ptr<KDNode<T>> currentNode,
                                             unsigned int cd) {
    if (currentNode == nullptr) {    // node at leaf doesn't exist and will be added
        currentNode = insertNode;
        currentNode->axis = cd;
        currentNode->value = currentNode->vec[cd];
    } else if (insertNode == currentNode) {    // error! duplicate
        return currentNode;
    } else if (insertNode->vec[cd] < currentNode->vec[cd]) {
        currentNode->left = insert(insertNode, currentNode->left, (cd + 1) % insertNode->getDim());
    } else {
        currentNode->right = insert(insertNode, currentNode->right, (cd + 1) % insertNode->getDim());
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
template <class T>
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
template <class T>
std::vector<T> KDTree<T>::searchRange(const Vec<float> &vec, float range) {
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
template <class T>
void KDTree<T>::NNS(const Vec<float> &vec, std::shared_ptr<KDNode<T>> node, std::shared_ptr<KDNode<T>> &refNode,
                    float &bestDist) {
    if (node->left == nullptr && node->right == nullptr) {
        float dist = vec.getSqDist(node->vec);
        if (dist < bestDist) {
            bestDist = dist;
            refNode = node;
        }
    } else {
        if (vec[node->axis] <= node->value) {
            if (vec[node->axis] - bestDist <= node->value && node->left != nullptr)
                NNS(vec, node->left, refNode, bestDist);
            if (vec[node->axis] + bestDist > node->value && node->right != nullptr)
                NNS(vec, node->right, refNode, bestDist);
        } else {
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
template <class T>
void KDTree<T>::RS(const Vec<float> &vec, std::shared_ptr<KDNode<T>> node, std::vector<std::shared_ptr<KDNode<T>>> &refNodes,
                   float sqRange, const Vec<float> &maxBoundary, const Vec<float> &minBoundary) {
    if (node == nullptr) {
        return;
    }
    if (vec.getSqDist(node->vec) < sqRange) {
        refNodes.push_back(node);
    }
    if (vec[node->axis] < maxBoundary[node->axis] && vec[node->axis] < minBoundary[node->axis]) {
        RS(vec, node->left, refNodes, sqRange, maxBoundary, minBoundary);
    } else if (vec[node->axis] > maxBoundary[node->axis] && vec[node->axis] > minBoundary[node->axis]) {
        RS(vec, node->right, refNodes, sqRange, maxBoundary, minBoundary);
    } else {
        RS(vec, node->left, refNodes, sqRange, maxBoundary, minBoundary);
        RS(vec, node->right, refNodes, sqRange, maxBoundary, minBoundary);
    }
}

template <class T>
std::shared_ptr<KDNode<T>> KDTree<T>::sort(std::vector<std::shared_ptr<Node>> &vec, unsigned int dim) {
    if (vec.size() < 1) {
        return nullptr;
    } else if (vec.size() == 1) {
        std::shared_ptr<KDNode<T>> kdNode = std::shared_ptr<KDNode<T>>(new KDNode<T>(vec[0]->getVec(), vec[0]));
        kdNode->axis = dim;
        kdNode->value = kdNode->vec[dim];
        return kdNode;
    } else if (vec.size() == 2) {
        quickSort(vec, 0, vec.size() - 1, dim);
        std::shared_ptr<KDNode<T>> kdNode = std::shared_ptr<KDNode<T>>(new KDNode<T>(vec[1]->getVec(), vec[1]));
        kdNode->axis = dim;
        kdNode->value = kdNode->vec[dim];

        std::vector<std::shared_ptr<Node>> vecLeft(vec.begin(), vec.begin() + 1);
        kdNode->left = sort(vecLeft, (dim + 1) % vec[vec.size() / 2]->getDim());

        return kdNode;
    } else {
        quickSort(vec, 0, vec.size() - 1, dim);
        std::shared_ptr<KDNode<T>> kdNode =
            std::shared_ptr<KDNode<T>>(new KDNode<T>(vec[vec.size() / 2]->getVec(), vec[vec.size() / 2]));
        kdNode->axis = dim;
        kdNode->value = kdNode->vec[dim];

        std::vector<std::shared_ptr<Node>> vecLeft(vec.begin(), vec.begin() + (vec.size() / 2) - 1);
        std::vector<std::shared_ptr<Node>> vecRight(vec.begin() + (vec.size() / 2) + 1, vec.end());
        kdNode->left = sort(vecLeft, (dim + 1) % vec[vec.size() / 2]->getDim());
        kdNode->right = sort(vecRight, (dim + 1) % vec[vec.size() / 2]->getDim());
        return kdNode;
    }
}

template <class T>
void KDTree<T>::quickSort(std::vector<std::shared_ptr<Node>> &A, int left, int right, int dim) {
    int r;
    if (left < right) {
        r = partition(A, left, right, dim);
        quickSort(A, left, r, dim);
        quickSort(A, r + 1, right, dim);
    }
}

template <class T>
int KDTree<T>::partition(std::vector<std::shared_ptr<Node>> &A, int left, int right, int dim) {
    float x = A[left]->getVecValue(dim);
    int i = left;
    int j;

    for (j = left + 1; j < right; j++) {
        if (A[j]->getVecValue(dim) <= x) {
            i = i + 1;
            swap(A[i], A[j]);
        }
    }

    swap(A[i], A[left]);
    return i;
}

} /* namespace rmpl */

#endif /* KDTREE_H_ */
