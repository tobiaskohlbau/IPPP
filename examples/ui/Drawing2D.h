#ifndef DRAWING2D_H_
#define DRAWING2D_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <Eigen/Core>

#include <core/dataObj/Node.h>
#include <core/dataObj/PointList.hpp>
#include <core/module/ModuleBase.h>

/*!
* \brief   Provides methods of drawing form 2D paths with OpenCV
* \author  Sascha Kaden
* \date    2016-05-25
*/

class Drawing2D : public rmpl::ModuleBase {
  public:
    static void drawTree2D(const std::vector<std::shared_ptr<rmpl::Node>> &nodes, cv::Mat &image,
                           Eigen::Vector3i colorNode, Eigen::Vector3i colorEdge, int thickness);
    static void drawGraph2D(const std::vector<std::shared_ptr<rmpl::Node>> &nodes, cv::Mat &image,
                            Eigen::Vector3i colorNode, Eigen::Vector3i colorEdge, int thickness);
    static void drawPath2D(const std::vector<Eigen::VectorXf> vecs, cv::Mat &image, Eigen::Vector3i colorEdge,
                           int thickness);
    static void drawTrianglePath(std::vector<Eigen::VectorXf> vecs, std::vector<rmpl::Triangle2D> triangles, cv::Mat &image,
                                 Eigen::Vector3i colorEdge, int thickness);

    static Eigen::MatrixXi cvToEigen(cv::Mat image);
};

#endif /* DRAWING2D_H_ */
