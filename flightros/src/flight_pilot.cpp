#include "flightros/flight_pilot.hpp"


namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    my_image_transport(nh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(346);
  rgb_camera_->setHeight(260);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostProcesscing(std::vector<bool>{false, false, false});
  quad_ptr_->addRGBCamera(rgb_camera_);

  // add event camera
  event_camera_ = std::make_shared<EventCamera>();
  // Vector<3> B_r_BCe(0.0, 0.0, 0.3);
  // Matrix<3, 3> R_BCe = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  event_camera_->setFOV(90);
  event_camera_->setWidth(346);
  event_camera_->setHeight(260);
  event_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addEventCamera(event_camera_);

  //   event_camera_2 = std::make_shared<EventCamera>();
  // // Vector<3> B_r_BCe(0.0, 0.0, 0.3);
  // // Matrix<3, 3> R_BCe = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  // std::cout << R_BC << std::endl;
  // event_camera_2->setFOV(90);
  // event_camera_2->setWidth(720);
  // event_camera_2->setHeight(480);
  // event_camera_2->setRelPose(B_r_BC, R_BC);
  // quad_ptr_->addEventCamera(event_camera_2);
  // int nume = quad_ptr_->getEventCameras().size();
  // ROS_WARN_STREAM("events"<<nume);
  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);
  quad_ptr_->getCameras().size();
  // initialize Publisher
  rgb_pub_ = my_image_transport.advertise("camera/rgb_image", 1);
  of_pub_ = my_image_transport.advertise("camera/of_image", 1);
  cv_pub_ = my_image_transport.advertise("camera/event_image", 1);
  // my_detected_line_image_pub = my_image_transport.advertise("camera/image",
  // 3); initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {

  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    // cv::Mat rgb_img;

    rgb_camera_->getOpticalFlow(optical_flow_image);
    rgb_camera_->getRGBImage(rgb_img);
    // event_camera_->getEvents(event_image);
    cv::split(optical_flow_image, bgr);
    // handle events to put in main loop, maybe put some checks
    // this function should be removed in the future
    event_image = event_camera_->createEventimages();

    // // // calculate the opticalflow with opencv
    // if (counter != 0) {
    //   calcopticalFlow();
    // }
    // cv::cvtColor(rgb_img, prev, cv::COLOR_BGR2GRAY);


    // // // publish the images of server in ros env
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img).toImageMsg();
    sensor_msgs::ImagePtr of_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", optical_flow_image).toImageMsg();
    sensor_msgs::ImagePtr ev_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", event_image).toImageMsg();
    cv_pub_.publish(ev_msg);
    rgb_pub_.publish(rgb_msg);
    of_pub_.publish(of_msg);

    // saveImages();
    if (counter % 100 == 0) {
      auto time = std::chrono::system_clock::now();
      std::time_t end_time = std::chrono::system_clock::to_time_t(time);
      ROS_INFO_STREAM("time " << std::ctime(&end_time));
    }
    saveImages();
    // saveCSV();  // only useful if checking the motion vectors
    counter++;
  }
}

void FlightPilot::calcopticalFlow() {
  // cv::Mat next;
  // curr = rgb_img;
  // cv::cvtColor(curr, next, cv::COLOR_BGR2GRAY);
  // cv::Mat flow(prev.size(), CV_32FC2);

  // calcOpticalFlowFarneback(prev, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
  // // visualization
  // cv::Mat flow_parts[2];
  // split(flow, flow_parts);
  // cv::Mat magnitude, angle, magn_norm;
  // cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
  // normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
  // angle *= ((1.f / 360.f) * (180.f / 255.f));
  // // build hsv image
  // cv::Mat _hsv[3], hsv, hsv8;
  // _hsv[0] = angle;
  // _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
  // _hsv[2] = magn_norm;
  // cv::merge(_hsv, 3, hsv);
  // hsv.convertTo(hsv8, CV_8U, 255.0);
  // cvtColor(hsv8, bgr_, cv::COLOR_HSV2BGR);

  // if (counter == 100) {
  //   calculateHist(flow_parts);
  // }

  // // publish the image
  // sensor_msgs::ImagePtr cv_msg =
  //   cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_).toImageMsg();
  // cv_pub_.publish(cv_msg);
}

void FlightPilot::saveCSV() {
  // // save csv files of motion
  std::string path_csvx =
    "/home/gian/flightmare/csv/motion_x_" + std::to_string(counter);
  std::string path_csvy =
    "/home/gian/flightmare/csv/motion_y_" + std::to_string(counter);
  writeCSV(path_csvx, bgr[1]);
  writeCSV(path_csvy, bgr[2]);
  // // save images
}

void FlightPilot::saveImages() {
  std::string path = "/home/gian/flightmare/opticalflow/opticalflow_img_" +
                     std::to_string(counter) + ".png";
  std::string path_rgb =
    "/home/gian/flightmare/rgb/rgb" + std::to_string(counter) + ".png";
  ROS_INFO_STREAM("written to " << path);
  cv::imwrite(path_rgb, rgb_img);
  cv::imwrite(path, event_image);
  // std::string path_ = "/home/gian/flightmare/calculated_of/calculated_img_" +
  //                     std::to_string(counter) + ".png";
  // cv::imwrite(path_, bgr_);
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // handle events
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
    // ROS_WARN_STREAM("events "<<quad_ptr_->getEventCameras->size());
  }
  return true;
}

// FloatType FlightPilot::maxMagnitudeOpticFlow(const OpticFlowPtr &flow) {
//   FloatType max_squared_magnitude = 0;
//   for (int y = 0; y < flow->rows; ++y) {
//     for (int x = 0; x < flow->cols; ++x) {
//       const FloatType squared_magnitude =
//         cv::norm((*flow)(y, x), cv::NORM_L2SQR);
//       if (squared_magnitude > max_squared_magnitude) {
//         max_squared_magnitude = squared_magnitude;
//       }
//     }
//   }
//   return std::sqrt(max_squared_magnitude);
// }

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}
void FlightPilot::writeCSV(std::string filename, cv::Mat m) {
  std::ofstream myfile;
  myfile.open(filename.c_str());
  myfile << cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
  myfile.close();
}

void FlightPilot::calculateHist(cv::Mat bgr_planes[3]) {
  int histSize = 256;
  float range[] = {-100, 156};  // the upper boundary is exclusive
  const float *histRange = {range};
  bool uniform = true, accumulate = false;
  cv::Mat x_hist, y_hist;
  cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), x_hist, 1, &histSize,
               &histRange, uniform, accumulate);
  cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), y_hist, 1, &histSize,
               &histRange, uniform, accumulate);
  // cv::calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize,
  // &histRange, uniform, accumulate );
  int hist_w = 512, hist_h = 400;
  int bin_w = cvRound((double)hist_w / histSize);
  cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
  normalize(x_hist, x_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
  normalize(y_hist, y_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
  // normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, Mat() );
  for (int i = 1; i < histSize; i++) {
    line(histImage,
         cv::Point(bin_w * (i - 1), hist_h - cvRound(x_hist.at<float>(i - 1))),
         cv::Point(bin_w * (i), hist_h - cvRound(x_hist.at<float>(i))),
         cv::Scalar(255, 0, 0), 2, 8, 0);
    line(histImage,
         cv::Point(bin_w * (i - 1), hist_h - cvRound(y_hist.at<float>(i - 1))),
         cv::Point(bin_w * (i), hist_h - cvRound(y_hist.at<float>(i))),
         cv::Scalar(0, 255, 0), 2, 8, 0);
    // line( histImage, Point( bin_w*(i-1), hist_h -
    // cvRound(r_hist.at<float>(i-1)) ),
    //       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
    //       Scalar( 0, 0, 255), 2, 8, 0  );
  }
  cv::imshow("calcHist Demo", histImage);
  std::string path_ =
    "/home/gian/flightmare/hist_" + std::to_string(counter) + ".png";
  cv::imwrite(path_, histImage);
}

}  // namespace flightros