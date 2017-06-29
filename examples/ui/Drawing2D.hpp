#ifndef DRAWING2D_HPP
#define DRAWING2D_HPP

#include <type_traits>

#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>

#include <ippp/core/Identifier.h>
#include <ippp/core/dataObj/Node.hpp>
#include <ippp/core/dataObj/PointList.hpp>
#include <ippp/core/types.h>
#include <ippp/core/util/Logging.h>

namespace ippp {
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
    static_assert(dim == 2, "Dimension has to be 2");
    for (auto &elem : nodes) {
        cv::Point point(elem->getValue(0), elem->getValue(1));
        // cv::circle(image, point, 3, cv::Scalar(colorNode[0], colorNode[1], colorNode[2]), 1);
        if (elem->getParentNode() != nullptr) {
            cv::Point point2(elem->getParentNode()->getValue(0), elem->getParentNode()->getValue(1));
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
    static_assert(dim == 2, "Dimension has to be 2");
    for (auto &elem : nodes) {
        cv::Point point(elem->getValue(0), elem->getValue(1));
        // cv::circle(image, point, 3, cv::Scalar(colorNode[0], colorNode[1], colorNode[2]), 1);
        for (auto &child : elem->getChildNodes()) {
            if (child != nullptr) {
                cv::Point point2(child->getValue(0), child->getValue(1));
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
static void drawPath2D(const std::vector<Vector2> vecs, cv::Mat &image, Eigen::Vector3i colorPoint, int thickness) {
    if (vecs.empty())
        return;

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
static void drawTrianglePath(std::vector<Vector3> vecs, std::vector<Triangle2D> triangles, cv::Mat &image,
                             Eigen::Vector3i colorPoint, int thickness) {
    if (vecs.empty())
        return;

    Logging::info("start drawing of triangles", "Drawing2D");

    Matrix2 R;
    Vector2 t;
    cv::Point2i pt1, pt2, pt3;
    Triangle2D triangle;
    for (auto vec : vecs) {
        for (int i = 0; i < triangles.size(); ++i) {
            triangle = triangles[i];
            util::poseVecToRandT(vec, R, t);
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
static Eigen::MatrixXi cvToEigen(cv::Mat cvMat) {
    if (cvMat.empty()) {
        Logging::error("Image is empty", "Drawing2D");
        return Eigen::MatrixXi();
    }

    cv::Mat dst;
    cvMat.convertTo(dst, CV_32SC1);

    Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> A_Eigen(dst.ptr<int>(), dst.rows, dst.cols);
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> B = A_Eigen;

    return B;
}

/*!
*  \brief      Convert cv::Mat to integer Eigen matrix
*  \author     Sascha Kaden
*  \param[in]  image
*  \param[out] Eigen matrix
*  \date       2016-12-18
*/
static cv::Mat eigenToCV(Eigen::MatrixXi eigenMat) {
    cv::Mat cvMat(eigenMat.rows(), eigenMat.cols(), CV_32SC1, eigenMat.data());

    if (Eigen::RowMajorBit) {
        cv::transpose(cvMat, cvMat);
    }

    cv::Mat dst;
    cvMat.convertTo(dst, CV_8UC1);

    return dst;
}

} /* namespace drawing */
} /* namespace ippp */

#endif /* DRAWING2D_HPP */
