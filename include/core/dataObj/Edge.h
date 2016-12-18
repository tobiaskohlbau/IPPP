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

#ifndef EDGE_H_
#define EDGE_H_

#include <memory>

namespace rmpl {

class Node;

/*!
* \brief   Class Edge contains the two nodes of the Edge and different parameters
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Edge {
  public:
    Edge();
    Edge(std::shared_ptr<Node> source, std::shared_ptr<Node> &target, float length);

    float getLength();

    void setSource(std::shared_ptr<Node> &source);
    std::shared_ptr<Node> getSource();
    void setTarget(std::shared_ptr<Node> &target);
    std::shared_ptr<Node> getTarget();

  private:
    std::shared_ptr<Node> m_source = nullptr;
    std::shared_ptr<Node> m_target = nullptr;
    float m_length = -1;
};

} /* namespace rmpl */

#endif /* EDGE_H_ */
