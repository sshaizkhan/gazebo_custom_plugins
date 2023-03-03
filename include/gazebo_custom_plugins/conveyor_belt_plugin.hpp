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

#ifndef TEST_GAZEBO_PLUGIN__TEST_GAZEBO_PLUGIN_HPP_
#define TEST_GAZEBO_PLUGIN__TEST_GAZEBO_PLUGIN_HPP_

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sdf/sdf.hh>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace test_gazebo_plugin
{

  /// @brief
  class TestGazeboPlugin : public gazebo::ModelPlugin
  {
  public:
    /// @brief Construction class
    TestGazeboPlugin();

    /// @brief Destructor class
    ~TestGazeboPlugin();

    /// @brief Load the model plugin
    /// @param model Pointer to the model that loaded this plugin
    /// @param sdf Pointer to SDF element that describes this plugin
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
    /// @brief Callback to be called at every simulation iteration
    void Update();

    /// @brief Set belt power and belt velocity
    /// @param power Power from SDF element
    void SetPowerAndVelocity(const double power);

    /// @brief Function to switch conveyor on/off
    /// @param req SetBool Request
    /// @param res SetBool Response
    void OnSwitch(
        std_srvs::srv::SetBool::Request::SharedPtr req,
        std_srvs::srv::SetBool::Response::SharedPtr res);

    /// @brief Robot namespace from SDF element
    std::string robot_namespace_;

    /// @brief Pointer to loaded model
    gazebo::physics::ModelPtr model_;

    /// @brief Pointer to world
    gazebo::physics::WorldPtr world_;

    double belt_power_;

    double belt_velocity_;

    double max_belt_velocity_;

    bool activated_;

    gazebo::physics::JointPtr joint_;
    gazebo::physics::LinkPtr link_;

    ignition::math::Angle angle_limit_;

    gazebo::event::ConnectionPtr update_connection_;
    gazebo_ros::Node::SharedPtr ros_node_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    /// @brief variables accessed on callbacks.
    std::mutex lock_;
  };

}
#endif // TEST_GAZEBO_PLUGIN__TEST_GAZEBO_PLUGIN_HPP_
