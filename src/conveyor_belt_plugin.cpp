#include "gazebo_custom_plugins/conveyor_belt_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <iostream>
#include <gazebo_ros/node.hpp>

test_gazebo_plugin::TestGazeboPlugin::TestGazeboPlugin()
    : robot_namespace_{""},
      belt_power_{0.0},
      belt_velocity_{0.0},
      max_belt_velocity_{0.0},
      activated_{false} {}

test_gazebo_plugin::TestGazeboPlugin::~TestGazeboPlugin() {}

void test_gazebo_plugin::TestGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  ros_node_ = gazebo_ros::Node::Get(sdf);
  RCLCPP_INFO(ros_node_->get_logger(), "Initializing Test Gazebo Plugin");

  model_ = model;
  world_ = model_->GetWorld();

  // Get QoS profiles
  const gazebo_ros::QoS &qos = ros_node_->get_qos();

  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    RCLCPP_INFO(ros_node_->get_logger(), "Found robot namesapace: [%s]", robot_namespace_.c_str());
  }

  if (sdf->HasElement("max_velocity"))
  {
    max_belt_velocity_ = sdf->Get<double>("max_velocity");
    RCLCPP_INFO(ros_node_->get_logger(), "Setting belt max velocity as: [%.2f]", max_belt_velocity_);
  }
  else
  {
    max_belt_velocity_ = 0.5;
    RCLCPP_INFO(ros_node_->get_logger(), "Using default max velocity as: [%.2f]", max_belt_velocity_);
  }

  if (sdf->HasElement("power"))
  {
    auto power = sdf->Get<double>("power");
    test_gazebo_plugin::TestGazeboPlugin::SetPowerAndVelocity(power);
    RCLCPP_INFO(ros_node_->get_logger(), "Found belt power as: [%.2f]", belt_power_);
  }
  else
  {
    belt_power_ = 0.5;
    RCLCPP_INFO(ros_node_->get_logger(), "Using default belt power as: [%.2f]", belt_power_);
  }

  std::string joint_name = "belt_joint";
  if (sdf->HasElement("joint"))
  {
    joint_name = sdf->Get<std::string>("joint");
    RCLCPP_INFO(ros_node_->get_logger(), "Found joint name: [%s]", joint_name.c_str());
  }
  else
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Using default joint name: [%s]", joint_name.c_str());
  }

  joint_ = model_->GetJoint(joint_name);

  if (!joint_)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Joint not found, belt disabled\n: [%s]", joint_name.c_str());
    return;
  }

  // Read and set the belt's link.
  std::string link_name = "belt_link";
  if (sdf->HasElement("link"))
  {
    link_name = sdf->Get<std::string>("link");
    RCLCPP_INFO(ros_node_->get_logger(), "Found link name: [%s]", link_name.c_str());
  }
  else
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Using default link name: [%s]", link_name.c_str());
  }

  link_ = boost::static_pointer_cast<gazebo::physics::Link>(world_->EntityByName(link_name));
  if (!link_)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Link not found, belt disabled\n: [%s]", link_name.c_str());
    return;
  }

  angle_limit_ = joint_->UpperLimit(0) * 0.001;

  // Initialize publisher
  pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("moving", qos.get_publisher_qos("moving", rclcpp::QoS(1)));

  RCLCPP_INFO(
      ros_node_->get_logger(),
      "Advertise gripper status on [%s]", pub_->get_topic_name());

  // Initialize service
  service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "switch",
      std::bind(
          &test_gazebo_plugin::TestGazeboPlugin::OnSwitch, this,
          std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
      ros_node_->get_logger(),
      "Advertise gripper switch service on [%s]", service_->get_service_name());

  RCLCPP_INFO(ros_node_->get_logger(), "Inititalized Plugin");

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&test_gazebo_plugin::TestGazeboPlugin::Update, this));
}

void test_gazebo_plugin::TestGazeboPlugin::Update()
{
    #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("GazeboRosVacuumGripper::OnUpdate");
  #endif
  std_msgs::msg::Bool actuation_msg;
  if (!activated_)
  {
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_BEGIN("publish");
    #endif
    actuation_msg.data = false;
    pub_->publish(actuation_msg);
    #ifdef IGN_PROFILER_ENABLE
        IGN_PROFILE_END();
    #endif
    return;
  }

  if (!joint_ || !link_)
    return;

  std::lock_guard<std::mutex> lock(lock_);

  joint_->SetVelocity(0, belt_velocity_);
  actuation_msg.data = true;

  if (ignition::math::Angle(joint_->Position(0)) >= angle_limit_)
  {
    joint_->SetPosition(0, 0);
  }

    #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish actuation_msg");
  #endif
    pub_->publish(actuation_msg);
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
  #endif
}

void test_gazebo_plugin::TestGazeboPlugin::SetPowerAndVelocity(const double power)
{

  if (power < 0 || power > 100)
  {
    RCLCPP_INFO(ros_node_->get_logger(), "Incorrect power value: [%.2f]", power);
    RCLCPP_INFO(ros_node_->get_logger(), "Accepted values are in range [0 - 1]");
    return;
  }
  belt_power_ = power;
  belt_velocity_ = max_belt_velocity_ * belt_power_;

  RCLCPP_INFO_STREAM(ros_node_->get_logger(),
                     "Received belt power as: " << belt_power_ << ", setting belt velocity to: " << belt_velocity_);
}

void test_gazebo_plugin::TestGazeboPlugin::OnSwitch(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
{
  res->success = false;
  if (req->data)
  {
    if (!activated_)
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Activating conveyor");
      activated_ = true;
      res->success = true;
    }
    else
    {
      RCLCPP_WARN(ros_node_->get_logger(), "Conveyor is already running");
    }
  }
  else
  {
    if (activated_)
    {
      RCLCPP_INFO(ros_node_->get_logger(), "Deactivating conveyor");
      activated_ = false;
      res->success = true;
    }
    else
    {
      RCLCPP_WARN(ros_node_->get_logger(), "Conveyor is already stopped");
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(test_gazebo_plugin::TestGazeboPlugin)
