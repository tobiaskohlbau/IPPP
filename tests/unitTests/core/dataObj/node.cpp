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
#include <core/utility/UtilList.hpp>

using namespace rmpl;

BOOST_AUTO_TEST_SUITE(constructor)

template <unsigned int dim>
void testStandardConstrurctor() {
    Node<dim> node;
    BOOST_CHECK(node.empty() == true);
    BOOST_CHECK(node.getParentNode() == nullptr);
    BOOST_CHECK(node.getChildNodes().size() == 0);
    BOOST_CHECK(node.getCost() == -1);
}

BOOST_AUTO_TEST_CASE(standardConstructor) {
    testStandardConstrurctor<2>();
    testStandardConstrurctor<3>();
    testStandardConstrurctor<4>();
    testStandardConstrurctor<5>();
    testStandardConstrurctor<6>();
    testStandardConstrurctor<7>();
    testStandardConstrurctor<8>();
    testStandardConstrurctor<9>();
}

template <unsigned int dim>
void testElementConstructor(Node<dim> node) {
    BOOST_CHECK(node.empty() == false);
    BOOST_CHECK(node.getParentNode() == nullptr);
    BOOST_CHECK(node.getChildNodes().size() == 0);
    BOOST_CHECK(node.getCost() == -1);
    for (int i = 0; i < dim; ++i)
        BOOST_CHECK(node.getValues()[i] == i);
}

BOOST_AUTO_TEST_CASE(elementConstructor) {
    testElementConstructor<2>(Node<2>(0, 1));
    testElementConstructor<3>(Node<3>(0, 1, 2));
    testElementConstructor<4>(Node<4>(0, 1, 2, 3));
    testElementConstructor<5>(Node<5>(0, 1, 2, 3, 4));
    testElementConstructor<6>(Node<6>(0, 1, 2, 3, 4, 5));
}

template <unsigned int dim>
void testVectorConstructor(Node<dim> node) {
    BOOST_CHECK(node.empty() == false);
    BOOST_CHECK(node.getParentNode() == nullptr);
    BOOST_CHECK(node.getChildNodes().size() == 0);
    BOOST_CHECK(node.getCost() == -1);
    for (int i = 0; i < dim; ++i)
        BOOST_CHECK(node.getValues()[i] == i);
}

BOOST_AUTO_TEST_CASE(vectorConstructor) {
    Node<2> node2(Vector2(0, 1));
    Node<3> node3(Vector3(0, 1, 2));
    Node<4> node4(Vector4(0, 1, 2, 3));
    Node<5> node5(util::Vecf(0, 1, 2, 3, 4));
    Node<6> node6(util::Vecf(0, 1, 2, 3, 4, 5));
    testVectorConstructor<2>(node2);
    testVectorConstructor<3>(node3);
    testVectorConstructor<4>(node4);
    testVectorConstructor<5>(node5);
    testVectorConstructor<6>(node6);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(parent_childes)

template <unsigned int dim>
void testParent() {
    Node<dim> node(Eigen::VectorXf::Constant(dim, 1, 0));
    Node<dim> parent(Eigen::VectorXf::Constant(dim, 1, 10));

    std::shared_ptr<Node<dim>> ptrParent = std::make_shared<Node<dim>>(parent);
    node.setParent(ptrParent, 1);
    BOOST_CHECK(node.getParentNode() == ptrParent);
    BOOST_CHECK(node.getParentEdge()->getTarget() == ptrParent);
    node.clearParent();
    BOOST_CHECK(node.getParentNode() == nullptr);
    BOOST_CHECK(node.getParentEdge() == nullptr);
}

BOOST_AUTO_TEST_CASE(parent) {
    testParent<2>();
    testParent<3>();
    testParent<4>();
    testParent<5>();
    testParent<6>();
    testParent<7>();
    testParent<8>();
    testParent<9>();
}

template <unsigned int dim>
void testChildes() {
    Node<dim> node(Eigen::VectorXf::Constant(dim, 1, 0));
    std::vector<std::shared_ptr<Node<dim>>> childes;
    for (int i = 0; i < 3; ++i) {
        childes.push_back(std::make_shared<Node<dim>>(Eigen::VectorXf::Constant(dim, 1, i)));
        node.addChild(childes[i], 1);
    }

    std::vector<std::shared_ptr<Node<dim>>> resultNodes = node.getChildNodes();
    BOOST_CHECK(util::contains(resultNodes, childes[0]));
    BOOST_CHECK(util::contains(resultNodes, childes[1]));
    BOOST_CHECK(util::contains(resultNodes, childes[2]));

    std::vector<std::shared_ptr<Edge<dim>>> resultEdges = node.getChildEdges();
    BOOST_CHECK(util::contains(childes, resultEdges[0]->getTarget()));
    BOOST_CHECK(util::contains(childes, resultEdges[1]->getTarget()));
    BOOST_CHECK(util::contains(childes, resultEdges[2]->getTarget()));
    node.clearChildes();
    BOOST_CHECK(node.getChildNodes().size() == 0);
    BOOST_CHECK(node.getChildEdges().size() == 0);
}

BOOST_AUTO_TEST_CASE(childes) {
    testParent<2>();
    testParent<3>();
    testParent<4>();
    testParent<5>();
    testParent<6>();
    testParent<7>();
    testParent<8>();
    testParent<9>();
}

BOOST_AUTO_TEST_SUITE_END()