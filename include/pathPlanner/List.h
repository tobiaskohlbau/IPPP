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

#ifndef LIST_H_
#define LIST_H_

#include <core/Base.h>
#include <core/Node.h>

namespace rmpl {

/*!
* \brief   Class AStarList offers a list for the A* algorithm
* \author  Sascha Kaden
* \date    2016-08-09
*/
class List : public Base {
  public:
    List();
    void addNode(std::shared_ptr<Node> node);
    void removeNode(std::shared_ptr<Node> node);
    std::shared_ptr<Node> removeMin();
    bool empty();
    bool contains(std::shared_ptr<Node> node);

  private:
    std::vector<std::shared_ptr<Node>> m_list;
};

} /* namespace rmpl */

#endif    // LIST_H_
