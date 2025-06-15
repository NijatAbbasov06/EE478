#include "qr_detector/detector.h"

#include <cv_bridge/cv_bridge.h>

namespace qr_detector {

Detector::Detector() : scanner_()
{
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

Tags Detector::detect(const cv::Mat& image, size_t timeout)
{
  Tags tags;

  // Check if the image is valid
  if (image.empty()) {
    std::cerr << "[Detector::detect] Empty input image!" << std::endl;
    return tags;
  }

  // Check if the input image has 3 channels (RGB or BGR)
  if (image.channels() != 3) {
    std::cerr << "[Detector::detect] Input image must have 3 channels (RGB/BGR), but has " 
              << image.channels() << " channels!" << std::endl;
    return tags;
  }

  // Log image size for debugging
  // ROS_INFO("Image Size: %dx%d", image.cols, image.rows);

  // Check for valid image size
  if (image.cols <= 0 || image.rows <= 0) {
    std::cerr << "[Detector::detect] Invalid image size!" << std::endl;
    return tags;
  }

  // Initialize grayImg with the same size as the input image
  cv::Mat grayImg;

  try {
    // Convert the image to grayscale (Handle BGR by default)
    if (image.channels() == 3) {
      cv::cvtColor(image, grayImg, cv::COLOR_BGR2GRAY); // BGR to Gray
    } else {
      std::cerr << "[Detector::detect] Unsupported image format with " << image.channels() << " channels!" << std::endl;
      return tags;
    }

    // Log the resulting gray image size
    // ROS_INFO("Gray Image Size: %dx%d", grayImg.cols, grayImg.rows);
  } catch (const cv::Exception& e) {
    std::cerr << "[Detector::detect] OpenCV error during color conversion: " << e.what() << std::endl;
    return tags;
  }

  // Create ZBar image
  const auto width = grayImg.cols;
  const auto height = grayImg.rows;

  // If the image is still invalid after the conversion, return early
  if (width <= 0 || height <= 0) {
    std::cerr << "[Detector::detect] Gray image has invalid size after conversion!" << std::endl;
    return tags;
  }

  zbar::Image img(width, height, "Y800", grayImg.data, width * height);
  scanner_.scan(img);

  // Process the detected QR codes
  for (auto s = img.symbol_begin(); s != img.symbol_end(); ++s)
  {
    Tag tag;
    tag.message = s->get_data();

    for (int i = 0; i < s->get_location_size(); i++) {
      tag.polygon.push_back(cv::Point(s->get_location_x(i), s->get_location_y(i)));
    }
    tags.push_back(tag);
  }

  return tags;
}




// Tags Detector::detect(const cv::Mat& image, size_t timeout)
// {
//   cv::Mat grayImg;
//   cv::cvtColor(image, grayImg, CV_BGR2GRAY);

//   const auto width = image.cols;
//   const auto height = image.rows;

//   zbar::Image img(width, height, "Y800", grayImg.data, width * height);
//   scanner_.scan(img);

//   Tags tags;
//   for (auto s = img.symbol_begin(); s != img.symbol_end(); ++s)
//   {
//     Tag tag;
//     tag.message = s->get_data();

//     for(int i = 0; i < s->get_location_size(); i++) {
//       tag.polygon.push_back(cv::Point(s->get_location_x(i), s->get_location_y(i)));
//     }
//     tags.push_back(tag);
//   }

//   return tags;
// }

}
