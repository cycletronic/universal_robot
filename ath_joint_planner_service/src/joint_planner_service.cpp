#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetMotionPlan.h>

#include <boost/scoped_ptr.hpp>

#include <ath_msgs/GetTrajectory.h>


class JointPlanner {
private:
  ros::ServiceServer service_;

  moveit::core::RobotStatePtr robot_state_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_;
  const moveit::core::JointModelGroup* joint_model_group_;

  const std::string PLANNING_GROUP = "manipulator";

public:
  JointPlanner(ros::NodeHandle *nh) {

    // Setup moveit objects
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    joint_model_group_ = move_group_interface_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robot_state_ = move_group_interface_->getCurrentState();

    // Advertise service
    service_ = nh->advertiseService("plan_path_to_joint_state", &JointPlanner::cbJointPlan, this);
  }

  bool cbJointPlan(ath_msgs::GetTrajectory::Request &request,
                   ath_msgs::GetTrajectory::Response &response) {

     planning_interface::MotionPlanRequest req;
     planning_interface::MotionPlanResponse res;

     req.group_name = PLANNING_GROUP;

     std::vector<double> joint_group_positions;
     robot_state_->copyJointGroupPositions(joint_model_group_, joint_group_positions);

     ROS_INFO("Received request to plan trajectory, current joint state");
     for(const auto& v : joint_group_positions) {
       ROS_INFO("%1.2f ", v);
     }

     moveit_msgs::MotionPlanResponse planResponse;

     // Assumes that number of joint values given matches number of joints in the robot
     std::vector<double> joint_values = request.joint_state.position;

     ROS_INFO("Plan trajectory to joint state:");
     for(const auto& v : joint_values) {
       ROS_INFO("%1.2f ", v);
     }

     move_group_interface_->setJointValueTarget(joint_values);

     moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

     bool success = (move_group_interface_->plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

     response.success = success;

     if(success) {
       // TODO this is not set up correctly, the robot does not move
       //move_group_interface_->execute(joint_plan);
     }

     return success;
  }
};


int main(int argc, char** argv)
{
  const std::string node_name = "joint_planner_service";
  ros::init(argc, argv, node_name);

  ros::NodeHandle node_handle("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  JointPlanner jointPlanner(&node_handle);

  ros::waitForShutdown();

  return 0;
}
