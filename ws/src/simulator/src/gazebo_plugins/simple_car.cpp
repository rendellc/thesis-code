#include "simulator/gazebo_plugins/simple_car.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>

#include <rcpputils/asserts.hpp>

#include "simulator/msg/float_array.hpp"

#include <iostream>

namespace gazebo_plugins
{
class SimpleCarPrivate
{
public:
    gazebo::physics::ModelPtr model_p;
    gazebo_ros::Node::SharedPtr ros_node_p;
    std::vector<gazebo::physics::JointPtr> steer_joints;
    std::vector<gazebo::physics::JointPtr> drive_joints;
    
    
    rclcpp::Subscription<simulator::msg::FloatArray>::SharedPtr drive_cmd_sub_p;
    rclcpp::Subscription<simulator::msg::FloatArray>::SharedPtr steer_cmd_sub_p;

    void OnDriveCommand(simulator::msg::FloatArray::SharedPtr msg_p){
        rcpputils::assert_true(msg_p->data.size() == drive_joints.size(), "drive commands don't match drive joints");

        RCLCPP_INFO(
            ros_node_p->get_logger(),
            "Recieved [%.3f, %.3f, %.3f %.3f]", msg_p->data[0], msg_p->data[1], msg_p->data[2], msg_p->data[3]
        );

    }
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
    
    impl_p->steer_joints.insert(impl_p->steer_joints.end(), {
        model_p->GetJoint("joint_wheel_fl_base_to_wheel_fl_steer"),
        model_p->GetJoint("joint_wheel_rl_base_to_wheel_rl_steer"),
        model_p->GetJoint("joint_wheel_rr_base_to_wheel_rr_steer"),
        model_p->GetJoint("joint_wheel_fr_base_to_wheel_fr_steer")
    });

    impl_p->drive_joints.insert(impl_p->drive_joints.end(), {
        model_p->GetJoint("joint_wheel_fl_steer_to_wheel_fl_tire"),
        model_p->GetJoint("joint_wheel_rl_steer_to_wheel_rl_tire"),
        model_p->GetJoint("joint_wheel_rr_steer_to_wheel_rr_tire"),
        model_p->GetJoint("joint_wheel_fr_steer_to_wheel_fr_tire")
    });
    

    impl_p->drive_cmd_sub_p = impl_p->ros_node_p->create_subscription<simulator::msg::FloatArray>(
        "cmd_drive", qos.get_subscription_qos("cmd_drive", rclcpp::QoS(1)),
        std::bind(&SimpleCarPrivate::OnDriveCommand, impl_p.get(), std::placeholders::_1)
    );
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Subscribed to [%s]", impl_p->drive_cmd_sub_p->get_topic_name()
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