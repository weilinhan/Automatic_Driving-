#include "psaf_shared/image_saver.hpp"

#include <chrono>
#include <sstream>


psaf_shared::ImageSaver::ImageSaver(
  const std::string & nodeName,
  const std::filesystem::path & basePath)
: mBasePath((basePath / nodeName) / getIsoDateTime())
{
  // check if dir exists, it should'nt since its timestamped
  if (std::filesystem::exists(mBasePath)) {
    throw std::runtime_error("ImageSaver tried to create an already existing output folder!");
  } else {
    std::filesystem::create_directories(mBasePath);
  }
}

bool psaf_shared::ImageSaver::save(
  const cv::Mat & img, const std::string & key,
  const bool onlySaveOnChange)
{
  bool first = false;

  // if we dont have this element yet create an entry for it
  // and note that we just created it so we can ignore onlySaveOnChange
  if (mPreviousImages.count(key) == 0) {
    mPreviousImages.emplace(key, img);
    first = true;
    std::filesystem::create_directory(mBasePath / key);
  }

  // load previous image reference
  auto & previousImg = mPreviousImages.at(key);
  bool changed = false;

  // only check for changes if flag is set and we already saved one image
  if (onlySaveOnChange && !first) {
    // check if image changed
    changed = !(cv::sum(img != previousImg) == cv::Scalar(0, 0, 0, 0));
  } else {
    changed = true;
  }

  if (changed) {
    // save image as previous
    previousImg = img.clone();

    // create the output path with timestamp and key
    const std::string path =
      ((mBasePath / key) / (getIsoDateTime(false, true) + ".png")).string();

    // if file already exists, we tried to save two images of same kind
    // in the same millisecond. This doesnt happen except when testing
    if (std::filesystem::exists(path)) {
      return false;
    } else {
      // save the image
      cv::imwrite(path, img);
    }
  }

  return changed;
}


std::filesystem::path psaf_shared::ImageSaver::getOutputPath() const
{
  return mBasePath;
}


std::string psaf_shared::ImageSaver::getIsoDateTime(const bool date, const bool ms) const
{
  // I love c++ time handling :) /s
  // get time values
  const auto timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count() % 1000;

  // create string
  std::stringstream timeString;
  timeString << std::put_time(std::localtime(&timestamp), date ? "%F-%T" : "%T");
  if (ms) {
    timeString << ":" << std::setfill('0') << std::setw(3) << milliseconds;
  }

  return timeString.str();
}
