/**
 * @file image_saver.hpp
 * @brief Definition of the ImageSaver class
 * @author PSAF
 * @date 2024-01-29
 */
#ifndef PSAF_SHARED__IMAGESAVER_HPP_
#define PSAF_SHARED__IMAGESAVER_HPP_

#include <filesystem>
#include <map>
#include <string>

#include "opencv4/opencv2/opencv.hpp"

namespace psaf_shared
{

/**
 * @name ImageSaver
 * @brief This class implements a simple way to save cv::Mat images with timestamps
 */
class ImageSaver
{
public:
  /**
   * @brief Construct a new ImageSaver object
   * @param nodeName name of the node, used for subfolder creation
   * @param basePath path to create the subfolder for this node in, defaults to $CWD/images
   */
  explicit ImageSaver(
    const std::string & nodeName,
    const std::filesystem::path & basePath = std::filesystem::current_path() / "images");

  /**
   * @brief Save a image
   * @param img the image to save
   * @param key the type of image, used for image comparison to previous and subfolder
   * @param onlySaveOnChange whether to only save if the image differs from previous image
   * @return true if the image was saved, else false
   */
  bool save(
    const cv::Mat & img, const std::string & key = "default",
    const bool onlySaveOnChange = true);
  std::filesystem::path getOutputPath() const;

  // deleted copy constructor
  ImageSaver(const ImageSaver &) = delete;
  // deleted assign operator
  ImageSaver & operator=(ImageSaver const &) = delete;

protected:
  /**
   * @brief Get a formatted timestamp string
   * @param date Whether to include the date or not
   * @param ms Wheter to inclue milliseconds or not
   * @return std::string the timestamp string
   */
  std::string getIsoDateTime(const bool date = true, const bool ms = false) const;

private:
  // base path for saving images
  std::filesystem::path mBasePath;
  // map containg previous image for each key
  std::map<std::string, cv::Mat> mPreviousImages;
};

}  // namespace psaf_shared

#endif  // PSAF_SHARED__IMAGESAVER_HPP_
