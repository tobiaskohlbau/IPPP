#include "Drawing2D.hpp"

namespace ippp {
namespace drawing {

void drawPolygons(cv::Mat &image, const Mesh &mesh, const Transform &pose, const Vector2i &offset, Vector3i color, int lineType) {
    Mesh tmpMesh = mesh;
    cad::transformVertices(pose, tmpMesh.vertices);
    for (auto &face : tmpMesh.faces) {
        std::vector<cv::Point> pts;
        pts.push_back(cv::Point(tmpMesh.vertices[face[0]][0] + offset[0], tmpMesh.vertices[face[0]][1] + offset[1]));
        pts.push_back(cv::Point(tmpMesh.vertices[face[1]][0] + offset[0], tmpMesh.vertices[face[1]][1] + offset[1]));
        pts.push_back(cv::Point(tmpMesh.vertices[face[2]][0] + offset[0], tmpMesh.vertices[face[2]][1] + offset[1]));
        cv::fillConvexPoly(image, pts, cv::Scalar(color[0], color[1], color[2]), lineType);
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
void drawPath2D(cv::Mat &image, const std::vector<Vector2> configs, Vector3i colorPoint, int thickness) {
    if (configs.empty())
        return;

    for (int i = 0; i < configs.size(); ++i) {
        cv::Point point(configs[i][0], configs[i][1]);
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
void drawTrianglePath(cv::Mat &image, const std::vector<Vector3> configs, const Mesh mesh, const Vector2i &offset,
                      Vector3i colorPoint, int lineType) {
    if (configs.empty())
        return;

    Logging::trace("start drawing of triangles", "Drawing2D");

    for (auto &config : configs) {
        auto T = util::toTransform(util::Vecd(config[0], config[1], 0, 0, 0, config[2]));
        drawPolygons(image, mesh, T, offset, colorPoint, lineType);
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
cv::Mat eigenToCV(Eigen::MatrixXi eigenMat) {
    cv::Mat cvMat(eigenMat.rows(), eigenMat.cols(), CV_32SC1, eigenMat.data());

    if (Eigen::RowMajorBit)
        cv::transpose(cvMat, cvMat);

    cv::Mat dst;
    cvMat.convertTo(dst, CV_8UC1);

    return dst;
}

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
void drawTree2D(cv::Mat &image, const std::vector<std::shared_ptr<Node<2>>> &nodes, Vector3i colorNode, Vector3i colorEdge,
                int thickness) {
    for (auto &elem : nodes) {
        cv::Point point(elem->getValue(0), elem->getValue(1));
        if (elem->getParentNode() != nullptr) {
            cv::Point point2(elem->getParentNode()->getValue(0), elem->getParentNode()->getValue(1));
            cv::line(image, point, point2, cv::Scalar(colorEdge[0], colorEdge[1], colorEdge[2]), thickness);
        }
    }
}

/*!
*  \brief         Draw nodes
*  \author        Sascha Kaden
*  \param[in]     vector of nodes
*  \param[in,out] image
*  \param[in]     color of the nodes
*  \param[in]     thickness of the points
*  \date          2018-03-15
*/
void drawNodes2D(cv::Mat &image, const std::vector<std::shared_ptr<Node<2>>> &nodes, Vector2i offset, Vector3i colorNode,
                 int thickness, double scale) {
    for (auto &elem : nodes) {
        double x = (elem->getValue(0) * scale) + offset[0];
        double y = (elem->getValue(1) * scale) + offset[1];
        cv::circle(image, cv::Point(x, y), thickness, cv::Scalar(colorNode[0], colorNode[1], colorNode[2]), thickness);
    }
}

/*!
*  \brief         Draw nodes
*  \author        Sascha Kaden
*  \param[in]     vector of nodes
*  \param[in,out] image
*  \param[in]     color of the nodes
*  \param[in]     thickness of the points
*  \date          2018-03-15
*/
void drawConfigs2D(cv::Mat &image, const std::vector<Vector2> &configs, Vector2i offset, Vector3i colorNode, int thickness,
                   double scale) {
    for (auto &elem : configs) {
        double x = (elem[0] * scale) + offset[0];
        double y = (elem[1] * scale) + offset[1];
        cv::circle(image, cv::Point(x, y), 1, cv::Scalar(colorNode[0], colorNode[1], colorNode[2]), thickness);
    }
}

} /* namespace drawing */
} /* namespace ippp */
