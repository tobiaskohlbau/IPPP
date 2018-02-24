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

#ifndef BRUTEFORCENF_HPP
#define BRUTEFORCENF_HPP

#include <utility>

#include <ippp/modules/neighborFinders/NeighborFinder.hpp>

namespace ippp {

/*!
* \brief   Class BruteForceNF for a brute force search
* \details The brute force approach goes through all nodes and returns the matches of the search.
* \author  Sascha Kaden
* \date    2017-05-16
*/
template <unsigned int dim, class T>
class BruteForceNF : public NeighborFinder<dim, T> {
  public:
    BruteForceNF(const std::shared_ptr<DistanceMetric<dim>> &distanceMetric);
    BruteForceNF(const std::shared_ptr<DistanceMetric<dim>> &distanceMetric, std::vector<T> &nodes);
    ~BruteForceNF();

    void addNode(const Vector<dim> &config, const T &node);
    void rebaseSorted(std::vector<T> &nodes);

    T searchNearestNeighbor(const Vector<dim> &config);
    std::vector<T> searchRange(const Vector<dim> &config, double range);

  private:
    std::vector<std::pair<const Vector<dim>, T>> m_nodes;
};

/*!
*  \brief      Default constructor of the class BruteForceNF
*  \author     Sascha Kaden
* \date        2017-05-16
*/
template <unsigned int dim, class T>
BruteForceNF<dim, T>::BruteForceNF(const std::shared_ptr<DistanceMetric<dim>> &distanceMetric)
    : NeighborFinder<dim, T>("BruteForceNF", distanceMetric) {
}

/*!
*  \brief      Constructor of the class BruteForceNF, builds a list with all nodes.
*  \author     Sascha Kaden
*  \param[in]  vector of nodes
* \date        2017-05-16
*/
template <unsigned int dim, class T>
BruteForceNF<dim, T>::BruteForceNF(const std::shared_ptr<DistanceMetric<dim>> &distanceMetric, std::vector<T> &nodes)
    : NeighborFinder<dim, T>("BruteForceNF", distanceMetric) {
    for (auto &node : nodes)
        m_nodes.push_back(std::make_pair(node->getValues(), node));
}

/*!
*  \brief      Destructor of the class BruteForceNF
*  \author     Sascha Kaden
* \date        2017-05-16
*/
template <unsigned int dim, class T>
BruteForceNF<dim, T>::~BruteForceNF() {
    m_nodes.clear();
}

/*!
*  \brief      Function of the interface, at the time it is empty.
*  \author     Sascha Kaden
* \date        2017-05-16
*/
template <unsigned int dim, class T>
void BruteForceNF<dim, T>::rebaseSorted(std::vector<T> &nodes) {
    if (m_nodes.size() != nodes.size()) {
        m_nodes.clear();
        for (auto &node : nodes)
            addNode(node->getValues(), node);
    }
}

/*!
*  \brief      Add a Node to the BruteForceNF
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  pointer to the Node
* \date        2017-05-16
*/
template <unsigned int dim, class T>
void BruteForceNF<dim, T>::addNode(const Vector<dim> &config, const T &node) {
    m_nodes.push_back(std::make_pair(node->getValues(), node));
}

/*!
*  \brief      Search for the nearest neighbor
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[out] pointer to the nearest Node
* \date        2017-05-16
*/
template <unsigned int dim, class T>
T BruteForceNF<dim, T>::searchNearestNeighbor(const Vector<dim> &config) {
    double minDist = std::numeric_limits<double>::max();
    T nodePtr = nullptr;
    for (auto &node : m_nodes) {
        if (this->m_metric->calcSimpleDist(config, node.first) < minDist && !config.isApprox(node.first, IPPP_EPSILON)) {
            minDist = this->m_metric->calcSimpleDist(config, node.first);
            nodePtr = node.second;
        }
    }
    return nodePtr;
}

/*!
*  \brief      Search for range around a position
*  \author     Sascha Kaden
*  \param[in]  position
*  \param[in]  distance of the range
*  \param[out] list of near nodes to the position
* \date        2017-05-16
*/
template <unsigned int dim, class T>
std::vector<T> BruteForceNF<dim, T>::searchRange(const Vector<dim> &config, double range) {
    std::vector<T> nodePtrs;
    this->m_metric->simplifyDist(range);

    for (auto &node : m_nodes)
        if (this->m_metric->calcSimpleDist(config, node.first) < range && !config.isApprox(node.first, IPPP_EPSILON))
            nodePtrs.push_back(node.second);
    
    return nodePtrs;
}

} /* namespace ippp */

#endif /* BRUTEFORCENF_HPP */
