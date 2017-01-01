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

#include <core/dataObj/Node.hpp>

using namespace rmpl;

BOOST_AUTO_TEST_SUITE(constructor)

BOOST_AUTO_TEST_CASE(standardConstructor) {
    Node<3> node;
    BOOST_CHECK(node.empty() == true);
}

BOOST_AUTO_TEST_CASE(elementContructor) {
    Node<2> node2(0, 1);
    Node<3> node3(0, 1, 2);
    Node<4> node4(0, 1, 2, 3);
    Node<5> node5(0, 1, 2, 3, 4);
    Node<6> node6(0, 1, 2, 3, 4, 5);

    BOOST_CHECK(node2.empty() == false);
    BOOST_CHECK(node3.empty() == false);
    BOOST_CHECK(node4.empty() == false);
    BOOST_CHECK(node5.empty() == false);
    BOOST_CHECK(node6.empty() == false);

    for (int i = 0; i < 2; ++i)
        BOOST_CHECK(node2.getValues()[i] == i);
    for (int i = 0; i < 3; ++i)
        BOOST_CHECK(node3.getValues()[i] == i);
    for (int i = 0; i < 4; ++i)
        BOOST_CHECK(node4.getValues()[i] == i);
    for (int i = 0; i < 5; ++i)
        BOOST_CHECK(node5.getValues()[i] == i);
    for (int i = 0; i < 6; ++i)
        BOOST_CHECK(node6.getValues()[i] == i);

    BOOST_CHECK(node2.getParentNode() == nullptr);
    BOOST_CHECK(node2.getChildNodes().size() == 0);
    BOOST_CHECK(node2.getCost() == -1);
    BOOST_CHECK(node3.getParentNode() == nullptr);
    BOOST_CHECK(node3.getChildNodes().size() == 0);
    BOOST_CHECK(node3.getCost() == -1);
    BOOST_CHECK(node4.getParentNode() == nullptr);
    BOOST_CHECK(node4.getChildNodes().size() == 0);
    BOOST_CHECK(node4.getCost() == -1);
    BOOST_CHECK(node5.getParentNode() == nullptr);
    BOOST_CHECK(node5.getChildNodes().size() == 0);
    BOOST_CHECK(node5.getCost() == -1);
    BOOST_CHECK(node6.getParentNode() == nullptr);
    BOOST_CHECK(node6.getChildNodes().size() == 0);
    BOOST_CHECK(node6.getCost() == -1);
}

BOOST_AUTO_TEST_SUITE_END()
