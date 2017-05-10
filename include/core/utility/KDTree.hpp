//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
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

#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <mutex>

#include <core/dataObj/Node.hpp>
#include <core/Identifier.h>
#include <core/dataObj/KDNode.hpp>

namespace ippp {

enum Direction { left, right };

/*!
* \brief   Class KDTree for a fast binary search
* \details Class uses KDNode<dim> to save the points
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim, class T>
class KDTree : public Identifier {
  public:
    KDTree();
    KDTree(std::vector<std::shared_ptr<Node<dim>>> &nodes);

    ~KDTree();
    void removeNodes(std::shared_ptr<KDNode<dim, T>> node);

    void addNode(const Vector<dim> &vec, const T &node);

    T searchNearestNeighbor(const Vector<dim> &vec);
    std::vector<T> searchRange(const Vector<dim> &vec, float range);

    std::shared_ptr<KDNode<dim, T>> sort(std::vector<std::shared_ptr<Node<dim>>> &vec, unsigned int cd);
    void quickSort(std::vector<std::shared_ptr<Node<dim>>> &A, int left, int right, int dimension);
    int partition(std::vector<std::shared_ptr<Node<dim>>> &A, int left, int right, int dimension);

  private:
    std::shared_ptr<KDNode<dim, T>> insert(std::shared_ptr<KDNode<dim, T>> insertNode,
                                           std::shared_ptr<KDNode<dim, T>> currentNode, unsigned int depth);
    void NNS(const Vector<dim> &point, std::shared_ptr<KDNode<dim, T>> node, std::shared_ptr<KDNode<dim, T>> &refNode,
             float &bestDist);
    void RS(const Vector<dim> &point, std::shared_ptr<KDNode<dim, T>> node,
            std::vector<std::shared_ptr<KDNode<dim, T>>> &refNodes, float sqRange, const Vector<dim> &maxBoundary,
            const Vector<dim> &minBoundary);

    std::shared_ptr<KDNode<dim, T>> m_root;
    std::mutex m_mutex;
};

/*!
*  \brief      Default constructor of the class KDTree
*  \author     Sascha Kaden
*  \date       2016-06-02
*/
template <unsigned int dim, class T>
KDTree<dim, T>::KDTree() : Identifier("KD Tree") {
}

/*!
*  \brief      Constructor of the class KDTree, builds a sorted tree with the passed nodes
*  \author     Sascha Kaden
*  \param[in]  vector of nodes
*  \date       2016-07-18
*/
template <unsigned int dim, class T>
KDTree<dim, T>::KDTree(std::vector<std::shared_ptr<Node<dim>>> &nodes) : Identifier("KD Tree") {
    quickSort(nodes, 0, nodes.size() - 1, 0);
    m_root = std::shared_ptr<KDNode<dim, T>>(new KDNode<dim, T>(nodes[nodes.size() / 2]->getValues(), nodes[nodes.size() / 2]));
    m_root->axis = 0;
    m_root->value = m_root->vec[0];

    std::vector<std::shared_ptr<Node<dim>>> vecLeft(nodes.begin(), nodes.begin() + (nodes.size() / 2) - 1);
    std::vector<std::shared_ptr<Node<dim>>> vecRight(nodes.begin() + (nodes.size() / 2) + 1, nodes.end());
    m_root->left = sort(vecLeft, 1);
    m_root->right = sort(vecRight, 1);
}

/*!
*  \brief      Destructor of the class KDTree
*  \author     Sascha Kaden
*  \date       2017-01-07
*/
template <unsigned int dim, class T>
KDTree<dim, T>::~KDTree() {
    if (m_root != nullptr) {
        removeNodes(m_root);
        m_root = nullptr;
    }
}

/*!
*  \brief      Remove the Nodes recursively to the leaf, by setting left and right pointer as nullptr
*  \param[in]  start node
*  \author     Sascha Kaden
*  \date       2017-01-07
*/
template <unsigned int dim, class T>
void KDTree<dim, T>::removeNodes(std::shared_ptr<KDNode<dim, T>> node) {
    if (node->left != nullptr) {
        removeNodes(node->left);
        node->left = nullptr;
    }
    if (node->right != nullptr) {
        removeNodes(node->right);
        node->right = nullptr;
    }
};

/*!
*  \brief      Add Node<dim> to the KDTree
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  pointer to the Node
*  \date       2016-05-27
*/
template <unsigned int dim, class T>
void KDTree<dim, T>::addNode(const Vector<dim> &vec, const T &node) {
    std::shared_ptr<KDNode<dim, T>> shrKDNode(new KDNode<dim, T>(vec, node));
    if (m_root == nullptr) {
        m_root = shrKDNode;
        return;
    }
    // insert(shrKDNode, m_root, 0);

    std::shared_ptr<KDNode<dim, T>> leaf = nullptr;
    std::shared_ptr<KDNode<dim, T>> last(m_root);
    Direction dir;
    if (shrKDNode->vec[0] < m_root->vec[0]) {
        leaf = m_root->left;
        dir = left;
    } else {
        leaf = m_root->right;
        dir = right;
    }

    unsigned int cd = 1;
    while (leaf != nullptr) {
        last = leaf;
        if (shrKDNode->vec[cd] < leaf->vec[cd]) {
            leaf = leaf->left;
            dir = left;
        } else {
            leaf = leaf->right;
            dir = right;
        }
        cd = (cd + 1) % dim;
    }

    // TODO: add control for mutex, if leaf is no nullptr
    m_mutex.lock();
    shrKDNode->axis = cd;
    shrKDNode->value = shrKDNode->vec[cd];
    if (dir == left) {
        last->left = shrKDNode;
    } else {
        last->right = shrKDNode;
    }
    m_mutex.unlock();
}

/*!
*  \brief      Insert KDNode<dim> to the KDTree (recursive function)
*  \author     Sascha Kaden
*  \param[in]  KDNode<dim> to insert
*  \param[in]  current KDNode<dim> in the KDTree
*  \param[in]  split dimension
*  \param[out] current KDNode
*  \date       2016-05-27
*/
template <unsigned int dim, class T>
std::shared_ptr<KDNode<dim, T>> KDTree<dim, T>::insert(std::shared_ptr<KDNode<dim, T>> insertNode,
                                                       std::shared_ptr<KDNode<dim, T>> currentNode, unsigned int cd) {
    if (currentNode == nullptr) {    // Node<dim> at leaf doesn't exist and will be added
        currentNode = insertNode;
        currentNode->axis = cd;
        currentNode->value = currentNode->vec[cd];
    } else if (insertNode == currentNode) {    // error! duplicate
        return currentNode;
    } else if (insertNode->vec[cd] < currentNode->vec[cd]) {
        currentNode->left = insert(insertNode, currentNode->left, (cd + 1) % dim);
    } else {
        currentNode->right = insert(insertNode, currentNode->right, (cd + 1) % dim);
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
template <unsigned int dim, class T>
T KDTree<dim, T>::searchNearestNeighbor(const Vector<dim> &vec) {
    if (m_root == nullptr) {
        return nullptr;
    }
    std::shared_ptr<KDNode<dim, T>> node;
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
template <unsigned int dim, class T>
std::vector<T> KDTree<dim, T>::searchRange(const Vector<dim> &vec, float range) {
    std::vector<T> nodes;
    if (m_root == nullptr) {
        return nodes;
    }
    std::vector<std::shared_ptr<KDNode<dim, T>>> kdNodes;
    float sqRange = range * range;
    Vector<dim> maxBoundary = vec;
    Vector<dim> minBoundary = vec;
    maxBoundary = maxBoundary.array() + range;
    minBoundary = minBoundary.array() - range;
    RS(vec, m_root, kdNodes, sqRange, maxBoundary, minBoundary);

    for (auto kdNode : kdNodes) {
        if (kdNode->vec != vec) {
            nodes.push_back(kdNode->node);
        }
    }
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
template <unsigned int dim, class T>
void KDTree<dim, T>::NNS(const Vector<dim> &vec, std::shared_ptr<KDNode<dim, T>> node, std::shared_ptr<KDNode<dim, T>> &refNode,
                         float &bestDist) {
    float dist = (vec - node->vec).squaredNorm();
    if (dist < bestDist) {
        bestDist = dist;
        refNode = node;
    }

    if (node->left == nullptr && node->right == nullptr) {
        return;
    } else {
        if (vec[node->axis] <= node->value) {
            if (vec[node->axis] - bestDist <= node->value && node->left != nullptr) {
                NNS(vec, node->left, refNode, bestDist);
            }
            if (vec[node->axis] + bestDist > node->value && node->right != nullptr) {
                NNS(vec, node->right, refNode, bestDist);
            }
        } else {
            if (vec[node->axis] + bestDist > node->value && node->right != nullptr) {
                NNS(vec, node->right, refNode, bestDist);
            }
            if (vec[node->axis] - bestDist <= node->value && node->left != nullptr) {
                NNS(vec, node->left, refNode, bestDist);
            }
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
template <unsigned int dim, class T>
void KDTree<dim, T>::RS(const Vector<dim> &vec, std::shared_ptr<KDNode<dim, T>> node,
                        std::vector<std::shared_ptr<KDNode<dim, T>>> &refNodes, float sqRange, const Vector<dim> &maxBoundary,
                        const Vector<dim> &minBoundary) {
    if (node == nullptr) {
        return;
    }
    if ((vec - node->vec).squaredNorm() < sqRange) {
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

/*!
*  \brief      Sorts the passed vector of Nodes and add the KDNode<dim> to the KDTree (recursive function)
*  \author     Sascha Kaden
*  \param[in]  vector of Node<dim> pointer
*  \param[in]  split dimension
*  \param[out] pointer of KDNode
*  \date       2016-08-09
*/
template <unsigned int dim, class T>
std::shared_ptr<KDNode<dim, T>> KDTree<dim, T>::sort(std::vector<std::shared_ptr<Node<dim>>> &vec, unsigned int cd) {
    if (vec.size() < 1) {
        return nullptr;
    } else if (vec.size() == 1) {
        std::shared_ptr<KDNode<dim, T>> kdNode = std::shared_ptr<KDNode<dim, T>>(new KDNode<dim, T>(vec[0]->getValues(), vec[0]));
        kdNode->axis = cd;
        kdNode->value = kdNode->vec[cd];
        return kdNode;
    } else if (vec.size() == 2) {
        quickSort(vec, 0, vec.size() - 1, cd);
        std::shared_ptr<KDNode<dim, T>> kdNode = std::shared_ptr<KDNode<dim, T>>(new KDNode<dim, T>(vec[1]->getValues(), vec[1]));
        kdNode->axis = cd;
        kdNode->value = kdNode->vec[cd];

        std::vector<std::shared_ptr<Node<dim>>> vecLeft(vec.begin(), vec.begin() + 1);
        kdNode->left = sort(vecLeft, (cd + 1) % dim);

        return kdNode;
    } else {
        quickSort(vec, 0, vec.size() - 1, cd);
        std::shared_ptr<KDNode<dim, T>> kdNode =
            std::shared_ptr<KDNode<dim, T>>(new KDNode<dim, T>(vec[vec.size() / 2]->getValues(), vec[vec.size() / 2]));
        kdNode->axis = cd;
        kdNode->value = kdNode->vec[cd];

        std::vector<std::shared_ptr<Node<dim>>> vecLeft(vec.begin(), vec.begin() + (vec.size() / 2) - 1);
        std::vector<std::shared_ptr<Node<dim>>> vecRight(vec.begin() + (vec.size() / 2) + 1, vec.end());
        kdNode->left = sort(vecLeft, (cd + 1) % dim);
        kdNode->right = sort(vecRight, (cd + 1) % dim);
        return kdNode;
    }
}

/*!
*  \brief      quick sort algorithm (recursive function)
*  \author     Sascha Kaden
*  \param[in]  vector of Node<dim> pointer
*  \param[in]  left start index for the vector
*  \param[in]  right end index for the vector
*  \param[in]  split dimension
*  \date       2016-08-09
*/
template <unsigned int dim, class T>
void KDTree<dim, T>::quickSort(std::vector<std::shared_ptr<Node<dim>>> &A, int left, int right, int cd) {
    int r;
    if (left < right) {
        r = partition(A, left, right, cd);
        quickSort(A, left, r, cd);
        quickSort(A, r + 1, right, cd);
    }
}

/*!
*  \brief      Partition of the vector and return pivot element
*  \author     Sascha Kaden
*  \param[in]  vector of Node<dim> pointer
*  \param[in]  left start index for the vector
*  \param[in]  right end index for the vector
*  \param[in]  split dimension
*  \param[out] index for splitting (pivot element)
*  \date       2016-08-09
*/
template <unsigned int dim, class T>
int KDTree<dim, T>::partition(std::vector<std::shared_ptr<Node<dim>>> &A, int left, int right, int cd) {
    float x = A[left]->getValues()[cd];
    int i = left;
    int j;

    for (j = left + 1; j < right; j++) {
        if (A[j]->getValues()[cd] <= x) {
            i = i + 1;
            swap(A[i], A[j]);
        }
    }

    swap(A[i], A[left]);
    return i;
}

} /* namespace ippp */

#endif /* KDTREE_HPP */
