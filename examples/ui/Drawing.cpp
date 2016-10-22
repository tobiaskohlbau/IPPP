#include <ui/Drawing.h>

#include <fstream>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief         Draw nodes and there edge to the parent Node
*  \author        Sascha Kaden
*  \param[in]     vector of nodes
*  \param[in,out] image
*  \param[in]     color of the nodes
*  \param[in]     color of the edges
*  \param[in]     thickness of the points
*  \date          2016-05-25
*/
void Drawing::drawTree2D(const std::vector<shared_ptr<Node>> &nodes, cv::Mat &image, const Vec<uint8_t> &colorNode,
                         const Vec<uint8_t> &colorEdge, int thickness) {
    assert(nodes[0]->getDim() == 2);

    for (auto &elem : nodes) {
        cv::Point point(elem->getX(), elem->getY());
        cv::circle(image, point, 3, cv::Scalar(colorNode[0], colorNode[1], colorNode[2]), 1);
        if (elem->getParentNode() != nullptr) {
            cv::Point point2(elem->getParentNode()->getX(), elem->getParentNode()->getY());
            cv::line(image, point, point2, cv::Scalar(colorEdge[0], colorEdge[1], colorEdge[2]), thickness);
        }
    }
}

/*!
*  \brief         Draw nodes and there edge to the child Nodes
*  \author        Sascha Kaden
*  \param[in]     vector of nodes
*  \param[in,out] image
*  \param[in]     color of the nodes
*  \param[in]     color of the edges
*  \param[in]     thickness of the points
*  \date          2016-05-25
*/
void Drawing::drawGraph2D(const std::vector<shared_ptr<Node>> &nodes, cv::Mat &image, const Vec<uint8_t> &colorNode,
                         const Vec<uint8_t> &colorEdge, int thickness) {
    assert(nodes[0]->getDim() == 2);

    for (auto &elem : nodes) {
        cv::Point point(elem->getX(), elem->getY());
        cv::circle(image, point, 3, cv::Scalar(colorNode[0], colorNode[1], colorNode[2]), 1);
        for (auto &child : elem->getChildNodes()) {
            if (child != nullptr) {
                cv::Point point2(child->getX(), child->getY());
                cv::line(image, point, point2, cv::Scalar(colorEdge[0], colorEdge[1], colorEdge[2]), thickness);
            }
        }
    }
}

/*!
*  \brief         Draw a path with the passed points in an image
*  \author        Sascha Kaden
*  \param[in]     vector of points
*  \param[in,out] image
*  \param[in]     color of the points
*  \param[in]     thickness of the points
*  \date          2016-05-25
*/
void Drawing::drawPath2D(const std::vector<Vec<float>> vecs, cv::Mat &image, const Vec<uint8_t> &colorPoint, int thickness) {
    if (vecs.size() == 0)
        return;

    assert(vecs[0].getDim() == 2);

    for (int i = 0; i < vecs.size(); ++i) {
        cv::Point point(vecs[i][0], vecs[i][1]);
        cv::circle(image, point, 2, cv::Scalar(colorPoint[0], colorPoint[1], colorPoint[2]), thickness);
    }
}

void Drawing::writeVecsToFile(const std::vector<rmpl::Vec<float>> &vecs, const std::string &filename, float scale) {
    std::ofstream myfile(filename);
    for (int i = 0; i < vecs.size(); ++i) {
        for (unsigned int j = 0; j < vecs[i].getDim(); ++j)
            myfile << vecs[i][j] * scale << " ";
        myfile << std::endl;
    }
    myfile.close();
}

void Drawing::appendVecsToFile(const std::vector<rmpl::Vec<float>> &vecs, const std::string &filename, float scale) {
    std::ofstream myfile;
    myfile.open(filename, std::ios_base::app);
    for (int i = 0; i < vecs.size(); ++i) {
        for (unsigned int j = 0; j < vecs[i].getDim(); ++j)
            myfile << vecs[i][j] * scale << " ";
        myfile << std::endl;
    }
    myfile.close();
}
