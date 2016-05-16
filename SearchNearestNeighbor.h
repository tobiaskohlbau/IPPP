#ifndef SEARHNEARESTNEIGHBOR_H_
#define SEARHNEARESTNEIGHBOR_H_

#include <cstdint>

#include "Node.h"

using std::shared_ptr;

class SearchNearestNeighbor
{
public:

    template<uint16_t dim>
    static shared_ptr<Node<dim>> search(const std::vector<shared_ptr<Node<dim>>> nodes, const Node<dim> node) {
        float dist = std::numeric_limits<float>::max();
        int index;

        for (auto& elem : nodes) {
            if (dist > elem->getDist(node)) {
                dist = elem->getDist(node);
                index = &elem - &nodes[0];
            }
        }
        return nodes[index];
    }

    template<uint16_t dim>
    static std::vector<shared_ptr<Node<dim>>> searchNearNodes(const std::vector<shared_ptr<Node<dim>>> nodes, const shared_ptr<Node<dim>> &node, const float dist) {
        std::vector<shared_ptr<Node<dim>>> nearNodes;
        for (auto& elem : nodes)
            if (elem->getDist(*node) < dist)
                nearNodes.push_back(elem);

        return nearNodes;
    }

};

#endif /* SEARHNEARESTNEIGHBOR_H_ */
