#include "simulator/gazebo_plugins/vehicle_body.hpp"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>

#include <ignition/math/Pose3.hh>

#include <rcpputils/asserts.hpp>

#include <geometry_msgs/msg/pose.h>

#include "simulator/msg/float_array.hpp"

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
    
    void OnUpdate(const gazebo::common::UpdateInfo& info) {
        UpdatePoses();
        vehicle_pose_pub_p->publish(
            gazebo_ros::Convert<geometry_msgs::msg::Pose>(vehicle_pose)
        );
        baselink_pose_pub_p->publish(
            gazebo_ros::Convert<geometry_msgs::msg::Pose>(vehicle_pose)
        );
    }

    
private:
    ignition::math::Pose3d vehicle_pose, baselink_pose;

    void UpdatePoses() {
        const auto link_ps = model_p->GetLinks();

        double vehicle_mass = 0.0;
        vehicle_pose.Reset();
        baselink_pose.Reset();
        for (const auto link_p : link_ps) {
            const auto link_pos = link_p->WorldCoGPose().Pos();
            const double link_mass = link_p->GetInertial()->Mass();
            
            vehicle_pose.Pos() += link_mass*link_pos;
            vehicle_mass += link_mass;
            
        }
        vehicle_pose.Pos() /= vehicle_mass;
        vehicle_pose.Rot() = model_p->GetChildLink("base_link")->WorldCoGPose().Rot();
        baselink_pose = model_p->GetChildLink("base_link")->WorldCoGPose();
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