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
#include <ippp/environment/cad/CadProcessing.h>
#include <ippp/environment/robot/SerialRobot.h>
#include <ippp/types.h>
#include <ippp/util/Logging.h>

namespace ippp {
namespace drawing {

void drawPolygons(cv::Mat &image, const Mesh &mesh, const Transform &pose, const Vector2i &offset, Vector3i color,
                  int lineType = CV_AA);
void drawPath2D(cv::Mat &image, const std::vector<Vector2> configs, Vector3i colorPoint, int thickness);
void drawTrianglePath(cv::Mat &image, const std::vector<Vector3> configs, const Mesh mesh, const Vector2i &offset,
                      Vector3i colorPoint, int lineType = CV_AA);

Eigen::MatrixXi cvToEigen(cv::Mat cvMat);
cv::Mat eigenToCV(Eigen::MatrixXi eigenMat);

void drawTree2D(cv::Mat &image, const std::vector<std::shared_ptr<Node<2>>> &nodes, Vector3i colorNode, Vector3i colorEdge,
                int thickness);
void drawGraph2D(cv::Mat &image, const std::vector<std::shared_ptr<Node<2>>> &nodes, Vector3i colorNode, Vector3i colorEdge,
                 int thickness);
void drawNodes2D(cv::Mat &image, const std::vector<std::shared_ptr<Node<2>>> &nodes, Vector2i offset, Vector3i colorNode,
                 int thickness = 1, double scale = 1);
void drawConfigs2D(cv::Mat &image, const std::vector<Vector2> &configs, Vector2i offset, Vector3i colorNode, int thickness,
                   double scale);

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
void drawSerialRobot2D(const Vector<dim> &config, const SerialRobot &robot, cv::Mat &image, const Vector2i &offset,
                       Vector3i colorPoint, int lineType = CV_AA) {
    Logging::trace("start drawing of SerialRobot2D", "Drawing2D");

    if (robot.getBaseModel())
        drawPolygons(image, robot.getBaseModel()->m_mesh, robot.getPose(), offset, colorPoint, lineType);

    auto linkModel = robot.getLinkModels();
    auto AsLinks = robot.getLinkTrafos(config);
    for (size_t i = 0; i < AsLinks.size(); ++i)
        drawPolygons(image, linkModel[i]->m_mesh, AsLinks[i], offset, colorPoint, lineType);
}

} /* namespace drawing */
} /* namespace ippp */

#endif /* DRAWING2D_HPP */
