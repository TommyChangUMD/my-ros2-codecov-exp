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
using std::placeholders::_1;
using std_msgs::msg::String;

using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;

class MinimalSubscriber : public rclcpp::Node {
public:

  MinimalSubscriber() : Node("minimal_subscriber")
  {
    auto callback = std::bind(&MinimalSubscriber::topic_callback, this, _1);

    subscription_ = this->create_subscription<String>("topic", 10, callback);
  }

private:

  void topic_callback(const std_msgs::msg::String& msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  SUBSCRIBER subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
