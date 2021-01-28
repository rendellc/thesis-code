#include "simulator/gazebo_plugins/simple_car.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>

#include <rcpputils/asserts.hpp>

#include "simulator/msg/float_array.hpp"

#include <iostream>
#include <array>

namespace gazebo_plugins
{
class SimpleCarPrivate
{
public:
    gazebo::event::ConnectionPtr update_connection_p;
    gazebo::physics::ModelPtr model_p;
    gazebo_ros::Node::SharedPtr ros_node_p;
    std::array<gazebo::physics::JointPtr,4> steer_joints;
    std::array<gazebo::physics::JointPtr,4> drive_joints;
    
    rclcpp::Subscription<simulator::msg::FloatArray>::SharedPtr drive_cmd_sub_p, steer_cmd_sub_p;
    
    void OnUpdate(const gazebo::common::UpdateInfo& info) {
        constexpr unsigned int z_axis_index = 2;
        for (size_t i = 0; i < steer_torques.size(); i++){
            steer_joints[i]->SetForce(z_axis_index, steer_torques[i]);
            drive_joints[i]->SetForce(z_axis_index, drive_torques[i]);
        }
    }

    void OnCommand(simulator::msg::FloatArray::SharedPtr msg_p){
        rcpputils::assert_true(msg_p->drive_torques.size() == drive_joints.size(), "drive commands don't match drive joints");
        rcpputils::assert_true(msg_p->steer_torques.size() == steer_joints.size(), "steer commands don't match steer joints");
        
        std::copy(msg_p->drive_torques.cbegin(), msg_p->drive_torques.cend(), drive_torques.begin());
        std::copy(msg_p->steer_torques.cbegin(), msg_p->steer_torques.cend(), steer_torques.begin());
    }

private:
    std::array<double,4> drive_torques = {0.0};
    std::array<double,4> steer_torques = {0.0};
};

SimpleCar::SimpleCar()
: impl_p(std::make_unique<SimpleCarPrivate>())
{
}

SimpleCar::~SimpleCar()
{
}

void SimpleCar::Load(gazebo::physics::ModelPtr model_p, sdf::ElementPtr sdf_p)
{
    impl_p->model_p = model_p;
    impl_p->ros_node_p  = gazebo_ros::Node::Get(sdf_p);
    const gazebo_ros::QoS& qos = impl_p->ros_node_p->get_qos();
    
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Loading SimpleCar plugin");
    
    impl_p->steer_joints[0] = model_p->GetJoint("joint_wheel_fl_base_to_wheel_fl_steer");
    impl_p->steer_joints[1] = model_p->GetJoint("joint_wheel_rl_base_to_wheel_rl_steer");
    impl_p->steer_joints[2] = model_p->GetJoint("joint_wheel_rr_base_to_wheel_rr_steer");
    impl_p->steer_joints[3] = model_p->GetJoint("joint_wheel_fr_base_to_wheel_fr_steer");

    impl_p->drive_joints[0] = model_p->GetJoint("joint_wheel_fl_steer_to_wheel_fl_tire");
    impl_p->drive_joints[1] = model_p->GetJoint("joint_wheel_rl_steer_to_wheel_rl_tire");
    impl_p->drive_joints[2] = model_p->GetJoint("joint_wheel_rr_steer_to_wheel_rr_tire");
    impl_p->drive_joints[3] = model_p->GetJoint("joint_wheel_fr_steer_to_wheel_fr_tire");

    impl_p->drive_cmd_sub_p = impl_p->ros_node_p->create_subscription<simulator::msg::FloatArray>(
        "command_torque", qos.get_subscription_qos("command_torque", rclcpp::QoS(1)),
        std::bind(&SimpleCarPrivate::OnCommand, impl_p.get(), std::placeholders::_1)
    );
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Subscribed to [%s]", impl_p->drive_cmd_sub_p->get_topic_name()
    );


    impl_p->update_connection_p = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimpleCarPrivate::OnUpdate, impl_p.get(), std::placeholders::_1)
    );
    
    RCLCPP_WARN(
        impl_p->ros_node_p->get_logger(),
        "SimpleCar plugin not implemented");
    
}

void SimpleCar::Reset()
{
}

GZ_REGISTER_MODEL_PLUGIN(SimpleCar)
} // namespace 