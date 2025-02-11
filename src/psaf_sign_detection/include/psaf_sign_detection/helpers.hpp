/**
 * @file helpers.cpp
 * @brief Definition of helper functions
 * @author sven.kalmbach
 * @date 2024-02-12
 */
#ifndef PSAF_SIGN_DETECTION__HELPERS_HPP_
#define PSAF_SIGN_DETECTION__HELPERS_HPP_

#include "opencv4/opencv2/opencv.hpp"

namespace helpers
{

/**
 * @brief Check if the depth image we got is in the format we expect
 * @details 8 bit unsigned triple channel
 * @param image the image to check
 */
void expectedColorFormat(const cv::Mat & image);

/**
 * @brief Moves a rectangle x/y so it fits inside fitTo
 * @param rectangle the rectangle to move
 * @param fitTo the size to fit into
 * @returns cv::Rect & reference to the input rectangle
 */
cv::Rect & moveToFit(cv::Rect & rectangle, const cv::Size fitTo);

/**
 * @brief Crop a rectangle to fit in the specified size
 * @details also makes sure rect.x or rect.y are not negative
 * @param rectangle the rectangle to fit
 * @param fitIn the size to fit in
 * @returns cv::Rect & reference to the input rectangle
 */
cv::Rect & cropToFit(cv::Rect & rectangle, const cv::Size & fitIn);

}  // namespace helpers

#endif  // PSAF_SIGN_DETECTION__HELPERS_HPP_
