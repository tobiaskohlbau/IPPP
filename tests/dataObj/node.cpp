//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#include <gtest/gtest.h>

#include <ippp/dataObj/Node.hpp>
#include <ippp/util/UtilList.hpp>

using namespace ippp;

template <unsigned int dim>
void testStandardConstrurctor() {
    Node<dim> node;
    EXPECT_TRUE(node.empty());
    EXPECT_EQ(node.getParentNode(), nullptr);
    EXPECT_EQ(node.getChildNodes().size(), 0);
    EXPECT_EQ(node.getPathCost(), 0);
}

TEST(NODE, standardConstructor) {
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
void testVectorConstructor(Node<dim> node) {
    EXPECT_FALSE(node.empty());
    EXPECT_EQ(node.getParentNode(), nullptr);
    EXPECT_EQ(node.getChildNodes().size(), 0);
    EXPECT_EQ(node.getPathCost(), 0);
    for (int i = 0; i < dim; ++i)
        EXPECT_EQ(node.getValues()[i], i);
}

TEST(NODE, vectorConstructor) {
    Node<2> node2(Vector2(0, 1));
    Node<3> node3(Vector3(0, 1, 2));
    Node<4> node4(Vector4(0, 1, 2, 3));
    Node<5> node5(util::Vecd(0, 1, 2, 3, 4));
    Node<6> node6(util::Vecd(0, 1, 2, 3, 4, 5));
    testVectorConstructor<2>(node2);
    testVectorConstructor<3>(node3);
    testVectorConstructor<4>(node4);
    testVectorConstructor<5>(node5);
    testVectorConstructor<6>(node6);
}

template <unsigned int dim>
void testParent() {
    Node<dim> node(Vector<dim>::Constant(dim, 1, 0));
    Node<dim> parent(Vector<dim>::Constant(dim, 1, 10));

    std::shared_ptr<Node<dim>> ptrParent = std::make_shared<Node<dim>>(parent);
    node.setParent(ptrParent, 1);
    EXPECT_EQ(node.getParentNode(), ptrParent);
    EXPECT_EQ(node.getParentEdge().first, ptrParent);
    node.clearParent();
    EXPECT_EQ(node.getParentNode(), nullptr);
    EXPECT_EQ(node.getParentEdge().first, nullptr);
}

TEST(NODE, parent) {
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
    Node<dim> node(Vector<dim>::Constant(dim, 1, 0));
    std::vector<std::shared_ptr<Node<dim>>> childes;
    for (int i = 0; i < 3; ++i) {
        childes.push_back(std::make_shared<Node<dim>>(Vector<dim>::Constant(dim, 1, i)));
        node.addChild(childes[i], 1);
    }

    std::vector<std::shared_ptr<Node<dim>>> resultNodes = node.getChildNodes();
    EXPECT_TRUE(util::contains(resultNodes, childes[0]));
    EXPECT_TRUE(util::contains(resultNodes, childes[1]));
    EXPECT_TRUE(util::contains(resultNodes, childes[2]));

    auto resultEdges = node.getChildEdges();
    EXPECT_TRUE(util::contains(childes, resultEdges[0].first));
    EXPECT_TRUE(util::contains(childes, resultEdges[1].first));
    EXPECT_TRUE(util::contains(childes, resultEdges[2].first));
    node.clearChildren();
    EXPECT_EQ(node.getChildNodes().size(), 0);
    EXPECT_EQ(node.getChildEdges().size(), 0);
}

TEST(NODE, children) {
    testParent<2>();
    testParent<3>();
    testParent<4>();
    testParent<5>();
    testParent<6>();
    testParent<7>();
    testParent<8>();
    testParent<9>();
}
