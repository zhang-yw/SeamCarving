/** Let users to mark a mask area
 */

#ifndef UI_H_
#define UI_H_

#include <opencv2/opencv.hpp>

void getMask(const cv::Mat3b img, cv::Mat1b &delMask, cv::Mat1b &protectMask);

#endif // UI_H_

