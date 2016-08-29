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

#include <algorithm>
#include <include/core/List.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Standard constructor of the class AStarList
*  \author     Sascha Kaden
*  \date       2016-08-09
*/
List::List() : Base("List") {
}

/*!
*  \brief      Adds a Node to the list
*  \author     Sascha Kaden
*  \param[in]  pointer of Node
*  \date       2016-08-09
*/
void List::addNode(std::shared_ptr<Node> node) {
    m_list.push_back(node);
}

/*!
*  \brief      Removes Node by reference object
*  \author     Sascha Kaden
*  \param[in]  pointer of Node
*  \date       2016-08-09
*/
void List::removeNode(std::shared_ptr<Node> node) {
    m_list.erase(std::remove(m_list.begin(), m_list.end(), node), m_list.end());
}

/*!
*  \brief      Returns the Node with the lowest cost and removes it from the list
*  \author     Sascha Kaden
*  \param[out] Node with lowest cost
*  \date       2016-08-09
*/
std::shared_ptr<Node> List::removeMin() {
    float min = std::numeric_limits<float>::max();
    std::shared_ptr<Node> minNode = nullptr;
    for (int i = 0; i < m_list.size(); ++i) {
        if (m_list[i]->getCost() < min) {
            minNode = m_list[i];
            min = m_list[i]->getCost();
        }
    }
    if (minNode != nullptr)
        removeNode(minNode);

    return minNode;
}

/*!
*  \brief      Return true, if list is empty
*  \author     Sascha Kaden
*  \param[out] empty state
*  \date       2016-08-09
*/
bool List::empty() {
    return m_list.empty();
}

/*!
*  \brief      Return true, if the list contains the passed Node
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[out] true, if list contains passed Node
*  \date       2016-08-09
*/
bool List::contains(std::shared_ptr<Node> node) {
    if (std::find(m_list.begin(), m_list.end(), node) != m_list.end())
        return true;
    else
        return false;
}
