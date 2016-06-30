#ifndef DRAWING_H_
#define DRAWING_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <assert.h>

#include <core/Base.h>
#include <core/Node.h>
#include <robot/RobotBase.h>

/*!
* \brief   Class Drawing for drawing 2D paths with OpenCV
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Drawing : public rmpl::Base
{
public:
    Drawing(int argc, char** argv);
    static void drawTree(const std::vector<std::shared_ptr<rmpl::Node>> &nodes, cv::Mat &image, const rmpl::Vec<uint8_t> &colorNode, const rmpl::Vec<uint8_t> &colorEdge, const int &thickness);
    static void drawPath(const std::shared_ptr<rmpl::Node> &goalNode, cv::Mat &image, const rmpl::Vec<uint8_t> &colorNode, const rmpl::Vec<uint8_t> &colorEdge, const int &thickness);
    static void drawPath(const std::vector<rmpl::Vec<float>> vecs, cv::Mat &image, const rmpl::Vec<uint8_t> &colorEdge, const int &thickness);

private:
};

#endif /* DRAWING_H_ */
