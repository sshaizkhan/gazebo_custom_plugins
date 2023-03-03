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

#include <string>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
    class ContactPlugin : public SensorPlugin
    {
    public:
        ContactPlugin();
        ~ContactPlugin();

        void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

    private:
        void OnUpdate();
        std::vector<std::string> contacts_collision_name_;
        gazebo_ros::Node::SharedPtr ros_node_;
        physics::WorldPtr world_;
        sensors::ContactSensorPtr parent_sensor_;
        event::ConnectionPtr update_connection_;
        std::vector<std::string> skip_models_;
    };
} // namespace gazebo
