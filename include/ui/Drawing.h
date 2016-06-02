#ifndef DRAWING_H_
#define DRAWING_H_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <assert.h>

#include <core/Base.h>
#include <core/Node.h>

namespace rmpl {

/*!
* \brief   Class Drawing for drawing 2D paths with OpenCV
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Drawing : public Base
{
public:
    static void drawTree(const std::vector<std::shared_ptr<Node>> &nodes, cv::Mat &image, const Vec<uint8_t> &colorNode, const Vec<uint8_t> &colorEdge, const int &thickness);
    static void drawPath(const std::shared_ptr<Node> &goalNode, cv::Mat &image, const Vec<uint8_t> &colorNode, const Vec<uint8_t> &colorEdge, const int &thickness);
    static void drawPath(const std::vector<Vec<float>> vecs, cv::Mat &image, const Vec<uint8_t> &colorEdge, const int &thickness);
};

} /* namespace rmpl */

#endif /* DRAWING_H_ */
