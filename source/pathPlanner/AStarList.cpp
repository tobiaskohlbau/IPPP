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

#include <pathPlanner/AStarList.h>
#include <algorithm>

using namespace rmpl;
using std::shared_ptr;

AStarList::AStarList() : Base("AStarList"){
}

void AStarList::addNode(std::shared_ptr<Node> node) {
    m_list.push_back(node);
}

void AStarList::removeNode(std::shared_ptr<Node> node) {
    m_list.erase(std::remove(m_list.begin(), m_list.end(), node), m_list.end());
}

std::shared_ptr<Node> AStarList::removeMin() {
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

bool AStarList::empty() {
    return m_list.empty();
}

bool AStarList::contains(std::shared_ptr<Node> node) {
    if (std::find(m_list.begin(), m_list.end(), node) != m_list.end())
        return true;
    else
        return false;
}
