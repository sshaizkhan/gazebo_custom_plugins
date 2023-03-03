#include "gazebo_custom_plugins/contact_plugin.hpp"

namespace gazebo
{

    ContactPlugin::ContactPlugin() : SensorPlugin() {}

    ContactPlugin::~ContactPlugin() {}

    void ContactPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
    {
        ros_node_ = gazebo_ros::Node::Get(sdf);
        RCLCPP_INFO(ros_node_->get_logger(), "Initializing...");

        if (sdf->HasElement("models_to_filter"))
        {
            auto iterate_sdf =
                sdf->GetElement("models_to_filter")->GetElement("name");
            while (iterate_sdf)
            {
                skip_models_.push_back(iterate_sdf->Get<std::string>());
                iterate_sdf = iterate_sdf->GetNextElement("name");
            }
        }

        parent_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(sensor);
        if (!parent_sensor_)
        {
            RCLCPP_ERROR(
                ros_node_->get_logger(),
                "ContactPlugin requires a ContactSensor.");
            return;
        }

        for (unsigned int i = 0; i < parent_sensor_->GetCollisionCount(); ++i)
        {
            contacts_collision_name_.push_back(parent_sensor_->GetCollisionName(i));
        }
        world_ = gazebo::physics::get_world(parent_sensor_->WorldName());

        update_connection_ = parent_sensor_->ConnectUpdated(
            std::bind(&ContactPlugin::OnUpdate, this));
        parent_sensor_->SetActive(true);
        RCLCPP_INFO(ros_node_->get_logger(), "Initialized");
    }

    void ContactPlugin::OnUpdate()
    {
        
    }

} // namespace gazebo
