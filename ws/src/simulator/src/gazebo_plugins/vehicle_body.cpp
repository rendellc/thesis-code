#include "simulator/gazebo_plugins/vehicle_body.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include <ignition/math/Pose3.hh>

#include <rcpputils/asserts.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <iostream>
#include <array>

namespace gazebo_plugins
{
class VehicleBodyPrivate
{
public:
    gazebo::event::ConnectionPtr update_connection_p;
    gazebo::physics::ModelPtr model_p;
    gazebo_ros::Node::SharedPtr ros_node_p;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_p;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_twist_pub_p;

    
    void OnUpdate(const gazebo::common::UpdateInfo& info) {
        UpdatePoses();
        vehicle_pose_pub_p->publish(vehicle_pose_msg);
        vehicle_twist_pub_p->publish(vehicle_twist_msg);
    }
private:
    geometry_msgs::msg::TwistStamped vehicle_twist_msg;
    geometry_msgs::msg::PoseStamped vehicle_pose_msg;

    void UpdatePoses() {
        // Read out baselink pose/twist
        const auto baselink_p = model_p->GetChildLink("base_link");
        const auto pose = baselink_p->WorldCoGPose();
        
        vehicle_pose_msg.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);

        vehicle_twist_msg.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
            baselink_p->RelativeLinearVel()
        );
        vehicle_twist_msg.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
            baselink_p->RelativeAngularVel()
        );

        // update headers
        const auto stamp = ros_node_p->now();
        vehicle_pose_msg.header.frame_id = "map";
        vehicle_pose_msg.header.stamp = stamp;
        
        vehicle_twist_msg.header.frame_id = "body";
        vehicle_twist_msg.header.stamp = stamp;
    }
};

VehicleBody::VehicleBody()
: impl_p(std::make_unique<VehicleBodyPrivate>())
{
}

VehicleBody::~VehicleBody()
{
}

void VehicleBody::Load(gazebo::physics::ModelPtr model_p, sdf::ElementPtr sdf_p)
{
    impl_p->model_p = model_p;
    impl_p->ros_node_p  = gazebo_ros::Node::Get(sdf_p);
    const gazebo_ros::QoS& qos = impl_p->ros_node_p->get_qos();
    
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Loading VehicleBody plugin for " + model_p->GetName());
    

    impl_p->vehicle_pose_pub_p = impl_p->ros_node_p->create_publisher<geometry_msgs::msg::PoseStamped>(
        "pose", qos.get_publisher_qos("pose", rclcpp::QoS(1))
    );
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Advertise pose to [%s]", impl_p->vehicle_pose_pub_p->get_topic_name()
    );

    impl_p->vehicle_twist_pub_p = impl_p->ros_node_p->create_publisher<geometry_msgs::msg::TwistStamped>(
        "twist", qos.get_publisher_qos("twist", rclcpp::QoS(1))
    );
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Advertise twist to [%s]", impl_p->vehicle_twist_pub_p->get_topic_name()
    );

    impl_p->update_connection_p = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&VehicleBodyPrivate::OnUpdate, impl_p.get(), std::placeholders::_1)
    );
}

void VehicleBody::Reset()
{
}

GZ_REGISTER_MODEL_PLUGIN(VehicleBody)
} // namespace 