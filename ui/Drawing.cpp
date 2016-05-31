#include <ui/Drawing.h>

/*!
*  \brief          Draw nodes and there edge to the parent Node
*  \author         Sascha Kaden
*  \param[in]      nodes
*  \param[in, out] image
*  \date           2016-05-25
*/
void Drawing::drawTree(const std::vector<shared_ptr<Node>> &nodes, cv::Mat &image, const Vec<uint8_t> &colorNode, const Vec<uint8_t> &colorEdge, const int &thickness)
{
    assert(nodes[0]->getDim() == 2);
    for (auto& elem : nodes) {
        cv::Point point(elem->getX(), elem->getY());
        cv::circle(image, point, 3, cv::Scalar(colorNode[0], colorNode[1], colorNode[2]), 2);
        if (elem->getParent() != nullptr) {
            cv::Point point2(elem->getParent()->getX(), elem->getParent()->getY());
            cv::line(image, point, point2, cv::Scalar(colorEdge[0], colorEdge[1], colorEdge[2]), thickness);
        }
    }
}

/*!
*  \brief          Draw a path from the goal Node to the init Node
*  \author         Sascha Kaden
*  \param[in]      goal Node
*  \param[in, out] image
*  \date           2016-05-25
*/
void Drawing::drawPath(const shared_ptr<Node> &goalNode, cv::Mat &image, const Vec<uint8_t> &colorNode, const Vec<uint8_t> &colorEdge, const int &thickness)
{
    assert(goalNode->getDim() == 2);
    // build vector for Drawing
    std::vector<shared_ptr<Node>> nodes;
    nodes.push_back(goalNode);
    shared_ptr<Node> temp = goalNode->getParent();
    while(temp != nullptr) {
        nodes.push_back(temp);
        temp = temp->getParent();
    }
    drawTree(nodes, image, colorNode, colorEdge, thickness);
}