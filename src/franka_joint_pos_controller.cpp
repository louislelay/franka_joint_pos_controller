#include <franka_joint_pos_controller/franka_joint_pos_controller.h>

namespace franka_joint_pos_controller{

    void JointPosController::CommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) 
    {
        if (msg->data.size() != 7) {
            ROS_ERROR("IL FAUT 7 ENTREE CEST TT.");
            return;
        }

        for (size_t i = 0; i < 7; ++i) {
            position_joint_handles_[i].setCommand(initial_pose_[i] + msg->data[i]);
        }
    }

    bool JointPosController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh)
    {
        ROS_INFO_STREAM(" ******* \n Starting Franka Robot Joint Position Controller \n *******");

        // Subscribe to position_command_sub_ topic
        position_command_sub = nh.subscribe("/franka_robot/position_command_sub", 1000, &JointPosController::CommandCallback, this);

        position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (position_joint_interface_ == nullptr) {
            ROS_ERROR(
                "JointPositionExampleController: Error getting position joint interface from hardware!");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!nh.getParam("joint_names", joint_names)) {
            ROS_ERROR("JointPositionExampleController: Could not parse joint names");
        }

        if (joint_names.size() != 7) {
            ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                            << joint_names.size() << " instead of 7 names!");
            return false;
        }

        position_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
            position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                "JointPositionExampleController: Exception getting joint handles: " << e.what());
            return false;
            }
        }

        std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        for (size_t i = 0; i < q_start.size(); i++) {
            if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
            ROS_ERROR_STREAM(
                "JointPositionExampleController: Robot is not in the expected starting position for "
                "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
                "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
            return false;
            }
        }

        return true;
    }
  

    void JointPosController::starting(const ros::Time&) 
    {
        for (size_t i = 0; i < 7; ++i) {
            initial_pose_[i] = position_joint_handles_[i].getPosition();
        }
        elapsed_time_ = ros::Duration(0.0);
    }

    void JointPosController::update(const ros::Time&, const ros::Duration& period) 
    {
        elapsed_time_ += period;
    }
}

PLUGINLIB_EXPORT_CLASS(franka_joint_pos_controller::JointPosController, controller_interface::ControllerBase)


































