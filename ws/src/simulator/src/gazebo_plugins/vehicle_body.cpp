#include "simulator/gazebo_plugins/vehicle_body.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include <ignition/math/Pose3.hh>

#include <rcpputils/asserts.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
    
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr vehicle_pose_pub_p, baselink_pose_pub_p;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vehicle_twist_pub_p;

    
    void OnUpdate(const gazebo::common::UpdateInfo& info) {
        UpdatePoses();
        vehicle_pose_pub_p->publish(
            gazebo_ros::Convert<geometry_msgs::msg::Pose>(vehicle_pose)
        );
        vehicle_twist_pub_p->publish(vehicle_twist);
        baselink_pose_pub_p->publish(
            gazebo_ros::Convert<geometry_msgs::msg::Pose>(vehicle_pose)
        );
    }

    
private:
    ignition::math::Pose3d vehicle_pose, baselink_pose;
    geometry_msgs::msg::Twist vehicle_twist;

    void UpdatePoses() {
        const auto link_ps = model_p->GetLinks();

        double vehicle_mass = 0.0;
        
        
        vehicle_pose.Reset();
        baselink_pose.Reset();
        ignition::math::Vector3d twist_linear;
        // TODO: twist should be computed as derivative of vehicle_pose
        // and not as a weighted average as done below. The computation 
        // below carelessly addsquantities from different coordinate
        // frames.
        for (const auto link_p : link_ps) {
            const auto link_pos = link_p->WorldCoGPose().Pos();
            const auto link_vel = link_p->RelativeLinearVel();
            const double link_mass = link_p->GetInertial()->Mass();
            
            vehicle_pose.Pos() += link_mass*link_pos;
            twist_linear += link_mass*link_vel;
            vehicle_mass += link_mass;
            
        }
        vehicle_pose.Pos() /= vehicle_mass;
        twist_linear /= vehicle_mass;
        vehicle_pose.Rot() = model_p->GetChildLink("base_link")->WorldCoGPose().Rot();
        auto baselink_p = model_p->GetChildLink("base_link");
        baselink_pose = baselink_p->WorldCoGPose();
        vehicle_twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(twist_linear);
        vehicle_twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
            baselink_p->RelativeAngularVel()
        );
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
    

    impl_p->vehicle_pose_pub_p = impl_p->ros_node_p->create_publisher<geometry_msgs::msg::Pose>(
        "pose", qos.get_publisher_qos("pose", rclcpp::QoS(1))
    );
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Advertise pose to [%s]", impl_p->vehicle_pose_pub_p->get_topic_name()
    );

    impl_p->vehicle_twist_pub_p = impl_p->ros_node_p->create_publisher<geometry_msgs::msg::Twist>(
        "twist", qos.get_publisher_qos("twist", rclcpp::QoS(1))
    );
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Advertise twist to [%s]", impl_p->vehicle_twist_pub_p->get_topic_name()
    );

    impl_p->baselink_pose_pub_p = impl_p->ros_node_p->create_publisher<geometry_msgs::msg::Pose>(
        "pose/baselink", qos.get_publisher_qos("pose/baselink", rclcpp::QoS(1))
    );
    RCLCPP_INFO(
        impl_p->ros_node_p->get_logger(),
        "Advertise baselink pose to [%s]", impl_p->baselink_pose_pub_p->get_topic_name()
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