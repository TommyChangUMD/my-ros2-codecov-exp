
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
using std::placeholders::_1;
using std_msgs::msg::String;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;

class MinimalSubscriber : public rclcpp::Node {
public:

  MinimalSubscriber(const std::string& node_name      = "my_node",
                    const std::string& node_namespace = "/my_ns/",
                    const std::string& topic_name     = "my_topic") :
    Node(node_name, node_namespace),
    count_(0)
  {
    auto callback = std::bind(&MinimalSubscriber::topic_callback, this, _1);

    subscription_ = this->create_subscription<String>(topic_name, 10, callback);
  }

private:

  void topic_callback(const std_msgs::msg::String& msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    // exit nicely so that coverage data can be saved properly
    if (++count_ > 3)
      exit(EXIT_SUCCESS);
  }

  size_t count_;
  SUBSCRIBER subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::spin(std::make_shared<MinimalSubscriber>());
  // rclcpp::shutdown();

  rclcpp::executors::MultiThreadedExecutor executor;

  int  numNodes = 5;
  auto nodes    = std::vector<RCL_NODE_PTR>(numNodes);

  for (int idx = 0; idx < numNodes; idx++)
  {
    std::string nodeName = "my_node" + std::to_string(idx);
    nodes[idx] = std::make_shared<MinimalSubscriber>(nodeName, "/", "topic");
    executor.add_node(nodes[idx]);
  }

  executor.spin();
  rclcpp::shutdown();

  return 0;
}
