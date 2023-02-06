#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


int main (int argc , char ** argv )
{
    ros::init ( argc , argv , " rover_arm_joint_space_goal ");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner (1);
    spinner . start ();

    // wait some time to load RVIZ
    sleep (15.0);

    moveit::planning_interface::MoveGroup group ("arm");
    moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface;

    // publisher for RVIZ
    ros::Publisher display_publisher = node_handle . advertise <
    moveit_msgs::DisplayTrajectory >("/move_group/display_planned_path ", 1 , true );
    moveit_msgs::DisplayTrajectory display_trajectory;
    ROS_INFO (" Reference frame for the robot : %s", group.getPlanningFrame().c_str());
    ROS_INFO (" Ref . frame for the End Effector : %s", group.getEndEffectorLink().c_str());

    // planning to a joint space GOAL
    std::vector <double > group_variable_values;

    //I'm writing in the new array defined above the current values of the robot's joints
    group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),group_variable_values);

    // now I change some values
    group_variable_values [1] = -0.8;
    group_variable_values [2] = 1.2;
    group_variable_values [3] = 0.7;
    group_variable_values [6] = -1.2;
    group . setJointValueTarget (group_variable_values);

    // Plan with the new setpoints in group_variable_values []
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group . plan ( my_plan );
    ROS_INFO (" Visualizing plan ( joint space goal ... %s)", success ?"":"== FAILED ==");
    sleep (10.0); // give time to RVIZ to plan

    if( success ) // making use of moveit_msgs::DisplayTrajectory to display trajectory
    {
        ROS_INFO (" let 's see it once again ");
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory . push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        sleep (10.0);
    }

    ros::shutdown ();
    return 0;
}