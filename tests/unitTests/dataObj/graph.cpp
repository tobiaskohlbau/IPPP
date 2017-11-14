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

#include <ippp/dataObj/Graph.hpp>
#include <ippp/modules/distanceMetrics/L2Metric.hpp>
#include <ippp/util/UtilList.hpp>

using namespace ippp;

template <unsigned int dim>
void testConstructor() {
    std::shared_ptr<DistanceMetric<dim>> distanceMetric(new L2Metric<dim>());
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> neighborFinder(
        new KDTree<dim, std::shared_ptr<Node<dim>>>(distanceMetric));

    Graph<dim> graph1(0, neighborFinder);
    EXPECT_FALSE(graph1.autoSort());
    EXPECT_EQ(graph1.getSortCount(), 0);
    EXPECT_EQ(graph1.nodeSize(), 0);

    Graph<dim> graph2(10, neighborFinder);
    EXPECT_TRUE(graph2.autoSort());
    EXPECT_EQ(graph2.getSortCount(), 10);
    EXPECT_EQ(graph2.nodeSize(), 0);
}

TEST(GRAPH, constructor) {
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
void NNS() {
    std::shared_ptr<DistanceMetric<dim>> distanceMetric(new L2Metric<dim>());
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> neighborFinder1(
        new KDTree<dim, std::shared_ptr<Node<dim>>>(distanceMetric));
    Graph<dim> graph1(0, neighborFinder1);
    std::vector<Vector<dim>> vecs;
    std::vector<std::shared_ptr<Node<dim>>> nodes;
    for (int i = 0; i < 100; i+=5) {
        Vector<dim> vec = Vector<dim>::Constant(dim, 1, i);
        vecs.push_back(vec);
        std::shared_ptr<Node<dim>> node(new Node<dim>(vec));
        nodes.push_back(node);
        graph1.addNode(node);
    }
    EXPECT_EQ(nodes[1], graph1.getNearestNode(nodes[0]));
    EXPECT_EQ(nodes[1], graph1.getNearestNode(*(nodes[0])));
    EXPECT_EQ(nodes[1], graph1.getNearestNode(vecs[0]));

    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> neighborFinder2(
        new KDTree<dim, std::shared_ptr<Node<dim>>>(distanceMetric));
    Graph<dim> graph2(0, neighborFinder2);
    for (int i = 1; i < 10; ++i)
        graph2.addNode(nodes[i]);
    EXPECT_EQ(nodes[1], graph2.getNearestNode(nodes[0]));
    EXPECT_EQ(nodes[1], graph2.getNearestNode(*(nodes[0])));
    EXPECT_EQ(nodes[1], graph2.getNearestNode(vecs[0]));
}

TEST(GRAPH, NNS) {
    NNS<2>();
    NNS<3>();
    NNS<4>();
    NNS<5>();
    NNS<6>();
    NNS<7>();
    NNS<8>();
    NNS<9>();
}

template <unsigned int dim>
void contains(std::vector<std::shared_ptr<Node<dim>>> result, std::vector<std::shared_ptr<Node<dim>>> nodes) {
    EXPECT_TRUE(util::contains(result, nodes[2]));
    EXPECT_TRUE(util::contains(result, nodes[3]));
    EXPECT_TRUE(util::contains(result, nodes[5]));
    EXPECT_TRUE(util::contains(result, nodes[6]));

    EXPECT_FALSE(util::contains(result, nodes[0]));
    EXPECT_FALSE(util::contains(result, nodes[1]));
    EXPECT_FALSE(util::contains(result, nodes[4]));
    EXPECT_FALSE(util::contains(result, nodes[7]));
    EXPECT_FALSE(util::contains(result, nodes[8]));
    EXPECT_FALSE(util::contains(result, nodes[9]));
}

template <unsigned int dim>
void RS() {
    std::shared_ptr<DistanceMetric<dim>> distanceMetric(new L2Metric<dim>());
    std::shared_ptr<NeighborFinder<dim, std::shared_ptr<Node<dim>>>> neighborFinder1(
        new KDTree<dim, std::shared_ptr<Node<dim>>>(distanceMetric));
    Graph<dim> graph1(0, neighborFinder1);
    std::vector<Vector<dim>> vecs;
    std::vector<std::shared_ptr<Node<dim>>> nodes;
    for (int i = 0; i < 10; ++i) {
        int value = i * 10;
        Vector<dim> vec = Eigen::VectorXd::Constant(dim, 1, value);
        vecs.push_back(vec);
        std::shared_ptr<Node<dim>> node = std::shared_ptr<Node<dim>>(new Node<dim>(vec));
        nodes.push_back(node);
        graph1.addNode(node);
    }
    std::vector<std::shared_ptr<Node<dim>>> result = graph1.getNearNodes(nodes[4], 50);
    contains(result, nodes);
    result = graph1.getNearNodes(*(nodes[4]), 50);
    contains(result, nodes);
    result = graph1.getNearNodes(nodes[4]->getValues(), 50);
    contains(result, nodes);
}

TEST(GRAPH, RS) {
    RS<3>();
    RS<4>();
    RS<5>();
    RS<6>();
}
