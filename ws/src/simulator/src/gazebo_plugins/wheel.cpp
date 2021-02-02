#include "simulator/gazebo_plugins/wheel.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>


#include <rcpputils/asserts.hpp>

#include "vehicle_interface/msg/wheel_command.hpp"
#include "vehicle_interface/msg/wheel_state.hpp"

#include <memory>

namespace gazebo_plugins
{
class WheelPrivate
{
public:
    void Load(gazebo::physics::ModelPtr model_p, sdf::ElementPtr sdf_p);
    void Reset();

private:
    gazebo::event::ConnectionPtr update_connection_p;
    gazebo::physics::JointPtr drive_joint_p, steer_joint_p;

    gazebo_ros::Node::SharedPtr ros_node_p;
    rclcpp::Subscription<vehicle_interface::msg::WheelCommand>::SharedPtr command_sub_p;
    rclcpp::Publisher<vehicle_interface::msg::WheelState>::SharedPtr wheel_state_pub_p;

    std::string name = "";
    double drive_torque = 0.0;
    double steer_torque = 0.0;

    
    void OnUpdate(const gazebo::common::UpdateInfo& info) 
    {
        constexpr unsigned int z_axis_index = 2;
        drive_joint_p->SetForce(z_axis_index, drive_torque);
        steer_joint_p->SetForce(z_axis_index, steer_torque);
        
        vehicle_interface::msg::WheelState wheel_state_msg;
        wheel_state_msg.angular_velocity = drive_joint_p->GetVelocity(z_axis_index);
        
        wheel_state_msg.steering_angle = steer_joint_p->Position();
        wheel_state_msg.steering_angle_rate = steer_joint_p->GetVelocity(z_axis_index);


        wheel_state_pub_p->publish(wheel_state_msg);
    }

    void OnCommand(vehicle_interface::msg::WheelCommand::SharedPtr msg_p)
    {
        drive_torque = msg_p->drive_torque;
        steer_torque = msg_p->steer_torque;
    }

};

Wheel::Wheel() 
: impl_p(std::make_unique<WheelPrivate>()) 
{
}

Wheel::~Wheel()
{
}

void Wheel::Load(gazebo::physics::ModelPtr model_p, sdf::ElementPtr sdf_p)
{
    impl_p->Load(model_p, sdf_p);
}

void Wheel::Reset()
{
    impl_p->Reset();
}


void WheelPrivate::Load(gazebo::physics::ModelPtr model_p, sdf::ElementPtr sdf_p)
{
    ros_node_p  = gazebo_ros::Node::Get(sdf_p);
    const gazebo_ros::QoS& qos = ros_node_p->get_qos();
    // 

    name = sdf_p->GetAttribute("name")->GetAsString();
    RCLCPP_INFO(
        ros_node_p->get_logger(),
        "Loading Wheel plugin for " + name);
    
    const std::string steer_joint_name = sdf_p->GetElement("steer_joint")->GetAttribute("name")->GetAsString();
    const std::string drive_joint_name = sdf_p->GetElement("drive_joint")->GetAttribute("name")->GetAsString();

    steer_joint_p = model_p->GetJoint(steer_joint_name);
    drive_joint_p = model_p->GetJoint(drive_joint_name);
    
    rcpputils::assert_true(name != "", "name not specified");
    rcpputils::assert_true(steer_joint_p != nullptr, "failed to load steer joint");
    rcpputils::assert_true(drive_joint_p != nullptr, "failed to load drive joint");
    
    // ros_node_p->create_subscriber()
    const auto command_topic = name + "/command";
    command_sub_p = ros_node_p->create_subscription<vehicle_interface::msg::WheelCommand>(
        command_topic, qos.get_subscription_qos(command_topic, rclcpp::QoS(1)),
        std::bind(&WheelPrivate::OnCommand, this, std::placeholders::_1)
    );
    
    const auto state_topic = name + "/state";
    wheel_state_pub_p = ros_node_p->create_publisher<vehicle_interface::msg::WheelState>(
        state_topic, qos.get_publisher_qos(state_topic, rclcpp::QoS(1))
    );

    update_connection_p = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&WheelPrivate::OnUpdate, this, std::placeholders::_1)
    );
}

void WheelPrivate::Reset()
{
    steer_torque = 0.0;
    drive_torque = 0.0;
}

GZ_REGISTER_MODEL_PLUGIN(Wheel)
} // namespace 