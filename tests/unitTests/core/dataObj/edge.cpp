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

#include <boost/test/unit_test.hpp>

#include <core/dataObj/Node.hpp>

using namespace ippp;

BOOST_AUTO_TEST_SUITE(constructor)

template <unsigned int dim>
void testConstructor() {
    VectorX sourceVec = Eigen::MatrixXf::Constant(dim, 1, 0);
    VectorX targetVec = Eigen::MatrixXf::Constant(dim, 1, 3);
    std::shared_ptr<Node<dim>> source = std::shared_ptr<Node<dim>>(new Node<dim>(sourceVec));
    std::shared_ptr<Node<dim>> target = std::shared_ptr<Node<dim>>(new Node<dim>(targetVec));

    Edge<dim> edge(source, target, 1);
    BOOST_CHECK(edge.getCost() == 1);
    BOOST_CHECK(source == edge.getSource());
    BOOST_CHECK(target == edge.getTarget());
}

BOOST_AUTO_TEST_CASE(standardConstructor) {
    testConstructor<2>();
    testConstructor<3>();
    testConstructor<4>();
    testConstructor<5>();
    testConstructor<6>();
    testConstructor<7>();
    testConstructor<8>();
    testConstructor<9>();
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(node)

template <unsigned int dim>
void testSourceNode() {
    VectorX sourceVec = Eigen::MatrixXf::Constant(dim, 1, 0);
    VectorX targetVec = Eigen::MatrixXf::Constant(dim, 1, 3);
    VectorX sourceVecNew = Eigen::MatrixXf::Constant(dim, 1, -10);
    std::shared_ptr<Node<dim>> source = std::shared_ptr<Node<dim>>(new Node<dim>(sourceVec));
    std::shared_ptr<Node<dim>> target = std::shared_ptr<Node<dim>>(new Node<dim>(targetVec));
    std::shared_ptr<Node<dim>> sourceNew = std::shared_ptr<Node<dim>>(new Node<dim>(sourceVecNew));

    Edge<dim> edge(source, target, 1);
    edge.setSource(sourceNew);
    BOOST_CHECK(sourceNew == edge.getSource());
    BOOST_CHECK(edge.getCost() == 1);
}

BOOST_AUTO_TEST_CASE(sourceNode) {
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
    VectorX sourceVec = Eigen::MatrixXf::Constant(dim, 1, 0);
    VectorX targetVec = Eigen::MatrixXf::Constant(dim, 1, 3);
    VectorX targetVecNew = Eigen::MatrixXf::Constant(dim, 1, 10);
    std::shared_ptr<Node<dim>> source = std::shared_ptr<Node<dim>>(new Node<dim>(sourceVec));
    std::shared_ptr<Node<dim>> target = std::shared_ptr<Node<dim>>(new Node<dim>(targetVec));
    std::shared_ptr<Node<dim>> targetNew = std::shared_ptr<Node<dim>>(new Node<dim>(targetVecNew));

    Edge<dim> edge(source, target, 1);
    edge.setTarget(targetNew);
    BOOST_CHECK(targetNew == edge.getTarget());
    BOOST_CHECK(edge.getCost() == 1);
}

BOOST_AUTO_TEST_CASE(targetNode) {
    testTargetNode<2>();
    testTargetNode<3>();
    testTargetNode<4>();
    testTargetNode<5>();
    testTargetNode<6>();
    testTargetNode<7>();
    testTargetNode<8>();
    testTargetNode<9>();
}

BOOST_AUTO_TEST_SUITE_END()