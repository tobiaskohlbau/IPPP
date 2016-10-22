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

#include <boost/test/unit_test.hpp>

#include <core/Node.h>

using namespace rmpl;

BOOST_AUTO_TEST_SUITE(constructor)

BOOST_AUTO_TEST_CASE(standardConstructor) {
    Node node;
    BOOST_CHECK(node.getDim() == 0);
    BOOST_CHECK(node.empty() == true);
}

BOOST_AUTO_TEST_CASE(elementContructor) {
    std::vector<Node> nodes;
    nodes.push_back(Node(0, 1));
    nodes.push_back(Node(0, 1, 2));
    nodes.push_back(Node(0, 1, 2, 3));
    nodes.push_back(Node(0, 1, 2, 3, 4));
    nodes.push_back(Node(0, 1, 2, 3, 4, 5));
    nodes.push_back(Node(0, 1, 2, 3, 4, 5, 6));
    nodes.push_back(Node(0, 1, 2, 3, 4, 5, 6, 7));
    nodes.push_back(Node(0, 1, 2, 3, 4, 5, 6, 7, 8));

    for (int i = 0; i < 8; ++i) {
        BOOST_CHECK(nodes[i].getDim() == (i + 2));
        BOOST_CHECK(nodes[i].empty() == false);
        for (int j = 0; j < nodes[i].getDim(); ++j) {
            BOOST_CHECK(nodes[i].getVecValue(j) == j);
        }
        BOOST_CHECK(nodes[i].getParentNode() == nullptr);
        BOOST_CHECK(nodes[i].getChildNodes().size() == 0);
        BOOST_CHECK(nodes[i].getCost() == 0);
    }
}

BOOST_AUTO_TEST_SUITE_END()
