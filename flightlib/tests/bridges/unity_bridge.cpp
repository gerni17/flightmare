#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/logger.hpp"

#include <gtest/gtest.h>

using namespace flightlib;

TEST(UnityBridge, Constructor) {
  Logger logger{"Test Unity Bridge"};
  UnityBridge unity_bridge;
  // bool unity_ready = false;

  // // need to add a quad to connect to Flightmare
  // QuadrotorDynamics dyn = QuadrotorDynamics(1.0, 0.2);
  // std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  // unity_bridge.addQuadrotor(quad);

  // unity_ready = unity_bridge.connectUnity(UnityScene::GARAGE);

  // if (unity_ready) logger.info("Unity Rendering is connected");
  // EXPECT_TRUE(unity_ready);
}

TEST(UnityBridge, PointCloud) {
  Logger logger{"Test PointCloud"};
  UnityBridge unity_bridge;

  // need to add a quad to connect to Flightmare
  // QuadrotorDynamics dyn = QuadrotorDynamics(1.0, 0.2);
  // std::shared_ptr<Quadrotor> quad = std::make_shared<Quadrotor>(dyn);
  // unity_bridge.addQuadrotor(quad);

  // unity_bridge.connectUnity(UnityScene::GARAGE);
  // PointCloudMessage_t pointcloud_msg;
  // pointcloud_msg.path = "/tmp/";
  // pointcloud_msg.file_name = "unity-bridge" + std::to_string(::rand());
  // EXPECT_TRUE(unity_bridge.getPointCloud(pointcloud_msg));
  // std::experimental::filesystem::remove(pointcloud_msg.path +
  //                                       pointcloud_msg.file_name + ".ply");
  // unity_bridge.disconnectUnity();
}