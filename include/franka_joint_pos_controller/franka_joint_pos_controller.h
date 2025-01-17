#pragma once

#include <array>
#include <string>
#include <vector>
#include <math.h>

#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller_base.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>

#include <pluginlib/class_list_macros.h>

namespace franka_joint_pos_controller{

    class JointPosController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface>
    {
        private:
            hardware_interface::PositionJointInterface* position_joint_interface_;
            std::vector<hardware_interface::JointHandle> position_joint_handles_;

            ros::Duration elapsed_time_;

            std::array<double, 7> initial_pose_{};

            ros::Subscriber position_command_sub;  // Subscriber for position commands

        public:
            bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
            void starting(const ros::Time&) override;
            void update(const ros::Time&, const ros::Duration& period) override;

            void CommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    };
}