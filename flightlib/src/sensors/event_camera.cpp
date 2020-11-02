#include "flightlib/sensors/event_camera.hpp"

namespace flightlib {

EventCamera::EventCamera()
  : channels_(3), width_(720), height_(480), fov_{70.0} {}

EventCamera::~EventCamera() {}

bool EventCamera::feedEventImageQueue(const cv::Mat& event) {
  queue_mutex_.lock();
  event_image_queue_.push_back(event);
  queue_mutex_.unlock();
  return true;
}

bool EventCamera::feedEventQueue(const std::vector<Event_t>& events) {
  // TODO:sort and order the events
  queue_mutex_.lock();
  event_queue_.push_back(events);
  // std::vector<Event> e(events.size()) ;
  event_queue_for_img.resize(events.size());
  event_queue_for_img = events;
  queue_mutex_.unlock();
  std::string amount = std::to_string(event_queue_for_img.size());
  logger_.warn(amount);
  // for (auto event : event_queue_for_img) {
  //   if (event.polarity != 0) logger_.error("ueppa");
  // }
  return true;
}

bool EventCamera::setRelPose(const Ref<Vector<3>> B_r_BC,
                             const Ref<Matrix<3, 3>> R_BC) {
  if (!B_r_BC.allFinite() || !R_BC.allFinite()) {
    logger_.error(
      "The setting value for Camera Relative Pose Matrix is not valid, discard "
      "the setting.");
    return false;
  }
  B_r_BC_ = B_r_BC;
  T_BC_.block<3, 3>(0, 0) = R_BC;
  T_BC_.block<3, 1>(0, 3) = B_r_BC;
  T_BC_.row(3) << 0.0, 0.0, 0.0, 1.0;
  return true;
}

bool EventCamera::setWidth(const int width) {
  if (width <= 0.0) {
    logger_.warn(
      "The setting value for Image Width is not valid, discard the setting.");
    return false;
  }
  width_ = width;
  return true;
}

bool EventCamera::setHeight(const int height) {
  if (height <= 0.0) {
    logger_.warn(
      "The setting value for Image Height is not valid, discard the "
      "setting.");
    return false;
  }
  height_ = height;
  return true;
}

bool EventCamera::setFOV(const Scalar fov) {
  if (fov <= 0.0) {
    logger_.warn(
      "The setting value for Camera Field-of-View is not valid, discard the "
      "setting.");
    return false;
  }
  fov_ = fov;
  return true;
}

// bool EventCamera::setDepthScale(const Scalar depth_scale) {
//   if (depth_scale_ < 0.0 || depth_scale_ > 1.0) {
//     logger_.warn(
//       "The setting value for Camera Depth Scale is not valid, discard the "
//       "setting.");
//     return false;
//   }
//   depth_scale_ = depth_scale;
//   return true;
// }

// bool EventCamera::setPostProcesscing(const std::vector<bool>& enabled_layers)
// {
//   if (enabled_layers_.size() != enabled_layers.size()) {
//     logger_.warn(
//       "Vector size does not match. The vector size should be equal to %d.",
//       enabled_layers_.size());
//     return false;
//   }
//   enabled_layers_ = enabled_layers;
//   return true;
// }

// std::vector<bool> EventCamera::getEnabledLayers(void) const {
//   return enabled_layers_;
// }

Matrix<4, 4> EventCamera::getRelPose(void) const { return T_BC_; }

// int EventCamera::getChannels(void) const { return channels_; }

int EventCamera::getWidth(void) const { return width_; }

int EventCamera::getHeight(void) const { return height_; }

Scalar EventCamera::getFOV(void) const { return fov_; }

// Scalar EventCamera::getDepthScale(void) const { return depth_scale_; }

bool EventCamera::getEventImages(cv::Mat& event) {
  if (!event_queue_.empty()) {
    // seems wrong here
    event = event_image_queue_.front();
    event_image_queue_.pop_front();
    return true;
  }
  return false;
}
cv::Mat EventCamera::createEventimages() {
  int wid = getWidth();
  int hei = getHeight();
  cv::Mat image = cv::Mat::zeros(cv::Size(wid, hei), CV_64FC1);
  std::vector<Event_t> events;

  // red
  // image.at<cv::Vec3b>(10, 10)[0] = 0;
  // image.at<cv::Vec3b>(10, 10)[1] = 0;
  // image.at<cv::Vec3b>(10, 10)[2] = 255;

  int count = 0;
  events = event_queue_for_img;
  // event_queue_for_img.clear();
  for (auto event : events) {
    if (event.coord_x > wid || event.coord_y > hei) {
      logger_.error("coord out of the image");
    }
    // if (event.polarity == 0);
    if (event.polarity == 1) {
      // std::string amount = std::to_string(event.polarity);
      // logger_.warn(amount);

      // logger_.warn("polneg");
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[0] = 0;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[1] = 0;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[2] = 255;
    } else if (event.polarity == -1) {
      count++;
      // logger_.warn("polpos");
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[0] = 255;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[1] = 0;
      image.at<cv::Vec3b>(event.coord_y, event.coord_x)[2] = 0;
    }
  }
  std::string amount = std::to_string(count);
  logger_.warn(amount);
  // cv::Mat flipped_image = cv::Mat::zeros(cv::Size(wid, hei), CV_64FC1);

  // cv::flip(flipped_image, image, 0);

  return image;
}

bool EventCamera::feedImageQueue(const cv::Mat& image_mat) {
  queue_mutex_.lock();

  if (rgb_queue_.size() > queue_size_) rgb_queue_.resize(queue_size_);
  rgb_queue_.push_back(image_mat);

  queue_mutex_.unlock();
  return true;
}


// take event cue and choose first 50events

// for eac event check that time is different thn zero
// put those event into matrix

bool EventCamera::getRGBImage(cv::Mat& rgb_img) {
  if (!rgb_queue_.empty()) {
    rgb_img = rgb_queue_.front();
    rgb_queue_.pop_front();
    return true;
  }
  return false;
}

// bool EventCamera::getDepthMap(cv::Mat& depth_map) {
//   if (!depth_queue_.empty()) {
//     depth_map = depth_queue_.front();
//     depth_queue_.pop_front();
//     return true;
//   }
//   return false;
// }

// bool EventCamera::getSegmentation(cv::Mat& segmentation) {
//   if (!segmentation_queue_.empty()) {
//     segmentation = segmentation_queue_.front();
//     segmentation_queue_.pop_front();
//     return true;
//   }
//   return false;
// }

// bool EventCamera::getOpticalFlow(cv::Mat& opticalflow) {
//   if (!opticalflow_queue_.empty()) {
//     opticalflow = opticalflow_queue_.front();
//     opticalflow_queue_.pop_front();
//     return true;
//   }
//   return false;
// }

}  // namespace flightlib