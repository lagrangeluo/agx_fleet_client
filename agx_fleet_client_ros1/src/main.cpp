/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "ClientNodeConfig.hpp"
#include "ClientNode.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "free_fleet_client_ros1");
  ros::NodeHandle ros_node_handle;

  std::cout << "-------------------------------------------" << std::endl;
  std::cout << "----Hi!This is agx_fleet_client in ros1----" << std::endl;
  
  auto config = free_fleet::ros1::ClientNodeConfig::make();
  
  ROS_INFO("get param down!");

  auto client_node = free_fleet::ros1::ClientNode::make(config);

  if (!client_node)
  {
    ROS_ERROR("agx_fleet_client_ros1: unable to initialize.");
    return 1;
  }
  return 0;
}
