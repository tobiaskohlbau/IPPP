#ifndef DRAWING_H_
#define DRAWING_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <core/Base.h>
#include <core/Node.h>

/*!
* \brief   Class Drawing for drawing 2D paths with OpenCV
* \author  Sascha Kaden
* \date    2016-05-25
*/
class Drawing : public rmpl::Base {
  public:
    static void drawTree2D(const std::vector<std::shared_ptr<rmpl::Node>> &nodes, cv::Mat &image,
                           const rmpl::Vec<uint8_t> &colorNode, const rmpl::Vec<uint8_t> &colorEdge, int thickness);
    static void drawGraph2D(const std::vector<std::shared_ptr<rmpl::Node>> &nodes, cv::Mat &image,
                            const rmpl::Vec<uint8_t> &colorNode, const rmpl::Vec<uint8_t> &colorEdge, int thickness);
    static void drawPath2D(const std::vector<rmpl::Vec<float>> vecs, cv::Mat &image, const rmpl::Vec<uint8_t> &colorEdge,
                           int thickness);

    static void writeVecsToFile(const std::vector<rmpl::Vec<float>> &vecs, const std::string &filename, const float scale = 1);
    static void appendVecsToFile(const std::vector<rmpl::Vec<float>> &vecs, const std::string &filename, const float scale = 1);

  private:
};

#endif /* DRAWING_H_ */
