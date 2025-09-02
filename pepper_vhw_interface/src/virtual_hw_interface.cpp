/*
# ============================================================
# File Name: <virtual_hw_interface.cpp>
# Developer: TAKKIE HALIMI
# Email: takkie8halimi@gmail.com
#
# Description: 
#     <This code is a hardware interface implementation 
#      for a virtual Pepper robot using ROS. It uses the hardware_interface 
#      and controller_manager libraries to manage the robot's joints and 
#      controllers. the Purpose of this code is to define a custom hardware interface 
#      for the virtual Pepper robot. It serves as the bridge between 
#      the robot's joint state information and ROS controllers, simulating 
#      the behavior of Pepper's hardware without requiring a physical robot.>
#
# Version History:
#     v1.0 - Initial version
#
# ============================================================
*/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <vector>
#include <string>
#include <xmlrpcpp/XmlRpc.h>
#include <thread>

class PepperVirtualHWInterface : public hardware_interface::RobotHW {
public:
    PepperVirtualHWInterface()
        : controller_manager_(this, nh_) {
        // Initialize joint names
        joint_names_ = {"HeadYaw", "HeadPitch", "HipRoll", "HipPitch", "KneePitch", "LShoulderPitch", "LShoulderRoll",
                        "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch", "RShoulderRoll",
                        "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "RFinger41", "LFinger42", "RFinger12",
                        "LFinger33", "RFinger31", "LFinger21", "RFinger32", "LFinger13", "LFinger32", "LFinger11",
                        "RFinger22", "RFinger13", "LFinger22", "RFinger21", "LFinger41", "LFinger12", "RFinger23",
                        "RFinger11", "LFinger23", "LFinger43", "RFinger43", "RFinger42", "LFinger31", "RFinger33",
                        "LThumb1", "RThumb2", "RThumb1", "LThumb2", "WheelFL", "WheelB", "WheelFR"};

        // Initial joint positions
        joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7853983250000001, 
                            0.0, -0.7853983250000001, 0.0, 0.5, 0.0, -0.7853983250000001, 0.0, 
                            0.7853983250000001, 0.0, 0.5, 0.4363325, 0.4363325, 0.4363325, 0.4363325,
                            0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 
                            0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 
                            0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 0.4363325, 
                            0.4363325, 0.4363325, 0.4363325, 0.0, 0.0, 0.0};

        joint_velocities_.resize(joint_names_.size(), 0.0);
        joint_efforts_.resize(joint_names_.size(), 0.0);

        // Register joint state interface
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            hardware_interface::JointStateHandle state_handle(
                joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]);
            joint_state_interface_.registerHandle(state_handle);
        }

        // Register all joints as PositionJointInterface
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            hardware_interface::JointHandle position_handle(
                joint_state_interface_.getHandle(joint_names_[i]), &joint_positions_[i]);
            position_joint_interface_.registerHandle(position_handle);
        }

        // Register the interfaces with the robot hardware
        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);

        // Initialize ROS publishers
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

        // Start controllers in a separate thread
        std::thread(&PepperVirtualHWInterface::startControllers, this).detach();
    }

    void startControllers() {
        XmlRpc::XmlRpcValue controller_list;
        std::vector<std::string> controllers_to_start;

        ROS_INFO("Checking for controller parameters...");
        if (nh_.getParam("pepper", controller_list)) {
            ROS_INFO("Controller parameters found.");
            if (controller_list.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                for (const auto& controller : controller_list) {
                    if (controller.second.getType() == XmlRpc::XmlRpcValue::TypeStruct &&
                        controller.second.hasMember("type") &&
                        controller.second.hasMember("joint")) {

                        std::string controller_name = controller.first;
                        std::string controller_type = static_cast<std::string>(controller.second["type"]);
                        std::string joint_name = static_cast<std::string>(controller.second["joint"]);
                        double position = static_cast<double>(controller.second["position"]);

                        // Set initial position for each controller
                        auto it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
                        if (it != joint_names_.end()) {
                            size_t index = std::distance(joint_names_.begin(), it);
                            joint_positions_[index] = position;
                        }

                        // Format the controller name
                        controllers_to_start.push_back("pepper/" + controller_name);

                        // Debugging output to check parameter values
                        ROS_INFO("Controller Name: %s", controller_name.c_str());
                        ROS_INFO("Controller Type: %s", controller_type.c_str());
                        ROS_INFO("Joint Name: %s", joint_name.c_str());
                        ROS_INFO("Initial Position: %f", position);
                    } else {
                        ROS_WARN("Controller definition for %s is invalid.", controller.first.c_str());
                    }
                }

                // Load each controller from the parameter server
                for (const auto& controller_name : controllers_to_start) {
                    try {
                        controller_manager_.loadController(controller_name);
                        ROS_INFO("Loaded controller: %s", controller_name.c_str());
                    } catch (const std::exception& e) {
                        ROS_ERROR("Failed to load controller %s: %s", controller_name.c_str(), e.what());
                    }
                }

                // Optionally start controllers after loading
                try {
                    controller_manager_.switchController(controllers_to_start, {}, 1, true, 0.0); // Start all controllers after loading
                    for (const auto& name : controllers_to_start) {
                        ROS_INFO("Started controller: %s", name.c_str());
                    }
                } catch (const std::exception& e) {
                    ROS_ERROR("Failed to start controllers: %s", e.what());
                }
            } else {
                ROS_WARN("Controller configuration is not a valid structure.");
            }
        } else {
            ROS_WARN("No controller parameters found!");
        }
    }

    void update() {
        // Update joint states here (e.g., reading from sensors, etc.)
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();  // Set the correct timestamp
        joint_state_msg.name = joint_names_;
        joint_state_msg.position = joint_positions_;
        joint_state_msg.velocity = joint_velocities_;
        joint_state_msg.effort = joint_efforts_;
        joint_state_pub_.publish(joint_state_msg);

        // ROS_INFO("Published joint states");  // Debugging output
    }

    void updateControllers() {
        controller_manager_.update(ros::Time::now(), ros::Duration(1.0 / 50.0));
    }

private:
    ros::NodeHandle nh_;
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    controller_manager::ControllerManager controller_manager_;
    ros::Publisher joint_state_pub_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pepper_virtual_hw_interface");
    ros::NodeHandle nh;

    PepperVirtualHWInterface pepper_hw_interface;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        pepper_hw_interface.update();
        pepper_hw_interface.updateControllers();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

