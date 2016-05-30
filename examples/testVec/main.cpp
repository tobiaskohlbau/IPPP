#include <cstdint>
#include <iostream>

#include <memory>
#include <ctime>
#include <core/Vec.hpp>
#include <core/Node.h>
#include <core/Graph.h>
#include <core/Sampling.h>

int main(int argc, char** argv)
{
    const unsigned int dim = 2;

    float temp = 10.0;
    Vec<float> vec1(temp,temp);
    Vec<float> vec2(vec1);
    vec2[1] = 11;

    Vec<float> vec6(2,6,3,6);
    vec6[0] = 3;

    Vec<float> vec3(2,4);

    vec3 = vec1 + vec2;

    //Vec<double> vecx = vec2 + vec6;
    Node node(vec1);
    Graph graph;
    graph.addNode(std::shared_ptr<Node>(new Node(vec1)));
    graph.addNode(std::shared_ptr<Node>(new Node(vec2)));
    graph.addNode(std::shared_ptr<Node>(new Node(vec3)));

    Vec<float> vecX(15,15);
    graph.getNearestNode(vecX);
    Sampling samp;
    Vec<float> minBoundary(0.0,0.0);
    Vec<float> maxBoundary(1000.0,1000.0);
    samp.setBoundaries(minBoundary, maxBoundary);
    samp.getSample(2, 5, 10);


    return 0;
}
