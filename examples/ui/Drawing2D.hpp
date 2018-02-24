#ifndef DRAWING2D_HPP
#define DRAWING2D_HPP

#include <type_traits>

#include <Eigen/Core>

#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <ippp/Identifier.h>
#include <ippp/dataObj/Node.hpp>
#include <ippp/dataObj/PointList.hpp>
#include <ippp/environment/cad/CadDrawing.h>
#include <ippp/environment/robot/SerialRobot.h>
#include <ippp/types.h>
#include <ippp/util/Logging.h>

namespace ippp {
namespace drawing {

static void drawTriangle(cv::Mat &image, cv::Point2i pt1, cv::Point2i pt2, cv::Point2i pt3, const Vector2i &offset,
                         Eigen::Vector3i colorPoint, int thickness) {
    pt1.x += static_cast<int>(offset[0]);
    pt2.x += static_cast<int>(offset[0]);
    pt3.x += static_cast<int>(offset[0]);
    pt1.y += static_cast<int>(offset[1]);
    pt2.y += static_cast<int>(offset[1]);
    pt3.y += static_cast<int>(offset[1]);

    cv::line(image, pt1, pt2, cv::Scalar(colorPoint[0], colorPoint[1], colorPoint[2]), thickness);
    cv::line(image, pt1, pt3, cv::Scalar(colorPoint[0], colorPoint[1], colorPoint[2]), thickness);
    cv::line(image, pt3, pt2, cv::Scalar(colorPoint[0], colorPoint[1], colorPoint[2]), thickness);
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
static void drawPath2D(const std::vector<Vector2> configs, cv::Mat &image, Eigen::Vector3i colorPoint, int thickness) {
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
*  \param[in]     config
*  \param[in]     SerialRobot2D
*  \param[in,out] image
*  \param[in]     color of the triangle lines
*  \param[in]     thickness of the lines
*  \date          2016-05-25
*/
template <unsigned int dim>
static void drawSerialRobot2D(const Vector<dim> &config, const SerialRobot &robot, cv::Mat &image, const Vector2i &offset,
                              Eigen::Vector3i colorPoint, int thickness) {
    Logging::trace("start drawing of SerialRobot2D", "Drawing2D");

    // base model
    if (robot.getBaseModel()) {
        auto baseMesh = robot.getBaseModel()->m_mesh;
        cad::transformVertices(robot.getPose(), baseMesh.vertices);

        for (auto &face : baseMesh.faces) {
            auto pt1 =
                cv::Point2i(static_cast<int>(baseMesh.vertices[face[0]][0]), static_cast<int>(baseMesh.vertices[face[0]][1]));
            auto pt2 =
                cv::Point2i(static_cast<int>(baseMesh.vertices[face[1]][0]), static_cast<int>(baseMesh.vertices[face[1]][1]));
            auto pt3 =
                cv::Point2i(static_cast<int>(baseMesh.vertices[face[2]][0]), static_cast<int>(baseMesh.vertices[face[2]][1]));
            drawTriangle(image, pt1, pt2, pt3, offset, colorPoint, thickness);
        }
    }

    auto linkModel = robot.getLinkModels();
    std::vector<Mesh> jointMeshes;
    for (auto &model : linkModel)
        jointMeshes.push_back(model->m_mesh);

    auto AsLinks = robot.getLinkTrafos(config);
    for (size_t i = 0; i < AsLinks.size(); ++i)
        cad::transformVertices(AsLinks[i], jointMeshes[i].vertices);

    for (auto &mesh : jointMeshes) {
        for (auto &face : mesh.faces) {
            auto pt1 = cv::Point2i(static_cast<int>(mesh.vertices[face[0]][0]), static_cast<int>(mesh.vertices[face[0]][1]));
            auto pt2 = cv::Point2i(static_cast<int>(mesh.vertices[face[1]][0]), static_cast<int>(mesh.vertices[face[1]][1]));
            auto pt3 = cv::Point2i(static_cast<int>(mesh.vertices[face[2]][0]), static_cast<int>(mesh.vertices[face[2]][1]));
            drawTriangle(image, pt1, pt2, pt3, offset, colorPoint, thickness);
        }
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
static void drawTrianglePath(const std::vector<Vector3> configs, const Mesh mesh, cv::Mat &image, const Vector2i &offset,
                             Eigen::Vector3i colorPoint, int thickness) {
    if (configs.empty())
        return;

    Logging::trace("start drawing of triangles", "Drawing2D");

    for (auto &config : configs) {
        auto tempMesh = mesh;
        Vector6 tmpConfig = util::Vecd(config[0], config[1], 0, 0, 0, config[2]);
        auto T = util::poseVecToTransform(tmpConfig);
        cad::transformVertices(T, tempMesh.vertices);
        for (auto &face : tempMesh.faces) {
            auto pt1 =
                cv::Point2i(static_cast<int>(tempMesh.vertices[face[0]][0]), static_cast<int>(tempMesh.vertices[face[0]][1]));
            auto pt2 =
                cv::Point2i(static_cast<int>(tempMesh.vertices[face[1]][0]), static_cast<int>(tempMesh.vertices[face[1]][1]));
            auto pt3 =
                cv::Point2i(static_cast<int>(tempMesh.vertices[face[2]][0]), static_cast<int>(tempMesh.vertices[face[2]][1]));
            drawTriangle(image, pt1, pt2, pt3, offset, colorPoint, thickness);
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
