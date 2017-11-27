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

#include <gtest/gtest.h>

#include <ippp/dataObj/Node.hpp>

using namespace ippp;

template <unsigned int dim>
void testConstructor() {
    VectorX sourceVec = Eigen::MatrixXd::Constant(dim, 1, 0);
    VectorX targetVec = Eigen::MatrixXd::Constant(dim, 1, 3);
    auto source = std::make_shared<Node<dim>>(sourceVec);
    auto target = std::make_shared<Node<dim>>(targetVec);

    Edge<dim> edge(source, target, 1);
    EXPECT_EQ(edge.getCost(), 1);
    EXPECT_EQ(source, edge.getSource());
    EXPECT_EQ(target, edge.getTarget());
}

TEST(EDGE, Constructor) {
    testConstructor<2>();
    testConstructor<3>();
    testConstructor<4>();
    testConstructor<5>();
    testConstructor<6>();
    testConstructor<7>();
    testConstructor<8>();
    testConstructor<9>();
}

template <unsigned int dim>
void testSourceNode() {
    VectorX sourceVec = Eigen::MatrixXd::Constant(dim, 1, 0);
    VectorX targetVec = Eigen::MatrixXd::Constant(dim, 1, 3);
    VectorX sourceVecNew = Eigen::MatrixXd::Constant(dim, 1, -10);
    auto source = std::make_shared<Node<dim>>(sourceVec);
    auto target = std::make_shared<Node<dim>>(targetVec);
    auto sourceNew = std::make_shared<Node<dim>>(sourceVecNew);

    Edge<dim> edge(source, target, 1);
    edge.setSource(sourceNew);
    EXPECT_EQ(sourceNew, edge.getSource());
    EXPECT_EQ(edge.getCost(), 1);
}

TEST(EDGE, SourceNode) {
    testSourceNode<2>();
    testSourceNode<3>();
    testSourceNode<4>();
    testSourceNode<5>();
    testSourceNode<6>();
    testSourceNode<7>();
    testSourceNode<8>();
    testSourceNode<9>();
}

template <unsigned int dim>
void testTargetNode() {
    VectorX sourceVec = Eigen::MatrixXd::Constant(dim, 1, 0);
    VectorX targetVec = Eigen::MatrixXd::Constant(dim, 1, 3);
    VectorX targetVecNew = Eigen::MatrixXd::Constant(dim, 1, 10);
    auto source = std::make_shared<Node<dim>>(sourceVec);
    auto target = std::make_shared<Node<dim>>(targetVec);
    auto targetNew = std::make_shared<Node<dim>>(targetVecNew);

    Edge<dim> edge(source, target, 1);
    edge.setTarget(targetNew);
    EXPECT_EQ(targetNew, edge.getTarget());
    EXPECT_EQ(edge.getCost(), 1);
}

TEST(EDGE, TargetNode) {
    testTargetNode<2>();
    testTargetNode<3>();
    testTargetNode<4>();
    testTargetNode<5>();
    testTargetNode<6>();
    testTargetNode<7>();
    testTargetNode<8>();
    testTargetNode<9>();
}
