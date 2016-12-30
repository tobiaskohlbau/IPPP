#ifndef DRAWING2D_H_
#define DRAWING2D_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <Eigen/Core>

#include <core/dataObj/Node.hpp>
#include <core/dataObj/PointList.hpp>
#include <core/module/ModuleBase.h>
#include <core/utility/Logging.h>

namespace rmpl {
namespace drawing {

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
template <unsigned int dim>
void drawTree2D(const std::vector<std::shared_ptr<Node<dim>>> &nodes, cv::Mat &image, Eigen::Vector3i colorNode,
                Eigen::Vector3i colorEdge, int thickness) {
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
template <unsigned int dim>
void drawGraph2D(const std::vector<std::shared_ptr<Node<dim>>> &nodes, cv::Mat &image, Eigen::Vector3i colorNode,
                 Eigen::Vector3i colorEdge, int thickness) {
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
void drawPath2D(const std::vector<Eigen::VectorXf> vecs, cv::Mat &image, Eigen::Vector3i colorPoint, int thickness) {
    if (vecs.size() == 0)
        return;

    assert(vecs[0].rows() == 2);

    for (int i = 0; i < vecs.size(); ++i) {
        cv::Point point(vecs[i][0], vecs[i][1]);
        cv::circle(image, point, 2, cv::Scalar(colorPoint[0], colorPoint[1], colorPoint[2]), thickness);
    }
}

/*!
*  \brief         Draw a triangle path with the passed triangles and posses in an image
*  \author        Sascha Kaden
*  \param[in]     vector of transformations
*  \param[in]     vector of triangles
*  \param[in,out] image
*  \param[in]     color of the triangle lines
*  \param[in]     thickness of the lines
*  \date          2016-05-25
*/
void drawTrianglePath(std::vector<Eigen::VectorXf> vecs, std::vector<Triangle2D> triangles, cv::Mat &image,
                      Eigen::Vector3i colorPoint, int thickness) {
    if (vecs.size() == 0)
        return;

    assert(vecs[0].rows() == 3);

    Logging::info("start drawing of triangles", "Drawing2D");

    Eigen::Matrix2f R;
    Eigen::Vector2f t;
    cv::Point2i pt1, pt2, pt3;
    Triangle2D triangle;
    for (auto vec : vecs) {
        for (int i = 0; i < triangles.size(); ++i) {
            triangle = triangles[i];
            utilGeo::poseVecToRandT(vec, R, t);
            triangle.transform(R, t);
            pt1 = cv::Point2i(triangle.getP(1)[0], triangle.getP(1)[1]);
            pt2 = cv::Point2i(triangle.getP(2)[0], triangle.getP(2)[1]);
            pt3 = cv::Point2i(triangle.getP(3)[0], triangle.getP(3)[1]);
            cv::line(image, pt1, pt2, cv::Scalar(colorPoint[0], colorPoint[1], colorPoint[2]), thickness);
            cv::line(image, pt1, pt3, cv::Scalar(colorPoint[0], colorPoint[1], colorPoint[2]), thickness);
            cv::line(image, pt3, pt2, cv::Scalar(colorPoint[0], colorPoint[1], colorPoint[2]), thickness);
        }
    }
}

/*!
*  \brief      Convert cv::Mat to integer Eigen matrix
*  \author     Sascha Kaden
*  \param[in]  image
*  \param[out] Eigen matrix
*  \date       2016-12-18
*/
Eigen::MatrixXi cvToEigen(cv::Mat cvMat) {
    if (cvMat.empty()) {
        Logging::error("Image is empty", "Drawing2D");
        return Eigen::MatrixXi();
    }

    cv::Mat dst;
    cvMat.convertTo(dst, CV_32SC1);

    int rows = dst.rows;
    int cols = dst.cols;
    Eigen::MatrixXi eigenMat(rows, cols);
    std::vector<int> entries;
    int *temp;
    for (int i = 0; i < cols; ++i) {
        temp = dst.ptr<int>(i);
        for (int j = 0; j < rows; ++j) {
            entries.push_back(*temp);
            ++temp;
        }
    }
    eigenMat = Eigen::MatrixXi::Map(&entries[0], rows, cols);
    return eigenMat;
}

} /* namespace drawing */
} /* namespace rmpl */

#endif /* DRAWING2D_H_ */
