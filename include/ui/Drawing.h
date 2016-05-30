#ifndef DRAWING_H_
#define DRAWING_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <assert.h>
#include <cstdint>
#include <memory>

#include <core/Node.h>

using std::shared_ptr;

/*!
* \brief   Class Drawing for drawing 2D paths with OpenCV
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Drawing
{
public:
    static void drawTree(const std::vector<shared_ptr<Node>> &nodes, cv::Mat &image, const Vec<uint8_t> &colorNode, const Vec<uint8_t> &colorEdge, const int &thickness);
    static void drawPath(const shared_ptr<Node> &goalNode, cv::Mat &image, const Vec<uint8_t> &colorNode, const Vec<uint8_t> &colorEdge, const int &thickness);
};

#endif /* DRAWING_H_ */
