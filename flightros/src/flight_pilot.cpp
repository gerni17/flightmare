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
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setPostPrecesscing(std::vector<bool>{true, true, true});
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);
  // initialize Publisher
  rgb_pub_ = my_image_transport.advertise("camera/rgb_image", 1);
  of_pub_ = my_image_transport.advertise("camera/of_image", 1);
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
    cv::Mat optical_flow_image;
    cv::Mat rgb_img;
    cv::Mat bgr[3];
    cv::Mat motion;
    rgb_camera_->getOpticalFlow(optical_flow_image);
    rgb_camera_->getSegmentation(rgb_img);
    cv::split(optical_flow_image, bgr);

    bgr->pop_back();

    // cv::merge(bgr, motion);

    // save rgb and opticalflow images
    std::string path = "/home/gian/flightmare/opticalflow/opticalflow_img_" +
                       std::to_string(counter) + ".jpg";
    std::string path_rgb =
      "/home/gian/flightmare/rgb/rgb" + std::to_string(counter) + ".jpg";
    // ROS_INFO_STREAM("written to " << path);
    cv::imwrite(path_rgb, rgb_img);
    cv::imwrite(path, optical_flow_image);
    // publish the images in ros env
    sensor_msgs::ImagePtr rgb_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img).toImageMsg();
    sensor_msgs::ImagePtr of_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", optical_flow_image)
        .toImageMsg();

    rgb_pub_.publish(rgb_msg);
    of_pub_.publish(of_msg);
    // extract maximal flow
    // create pointer

    // FloatType max = FlightPilot::maxMagnitudeOpticFlow();

    counter++;
  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // empty
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
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

}  // namespace flightros