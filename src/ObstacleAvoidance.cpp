#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main (int argc , char ** argv )
{
    ros::init ( argc , argv , " rover_arm_obstacle_avoidance ");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start ();

    // sleeping to load rviz
    sleep (15.0);

    // the move group interface allows me to control and plan only for the desired group , 
    // in my case the group "arm " defined in the SRDF
    moveit::planning_interface::MoveGroup group ("arm");
    group.allowReplanning(true);

    // now the planning scene interface deals with the world coordinates, objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // hereafter we create a publisher in order to display trajectories on rviz.
    ros::Publisher display_publisher = node_handle.advertise <
    moveit_msgs::DisplayTrajectory >("/move_group/display_planned_path ", 1 , true );
    moveit_msgs::DisplayTrajectory display_trajectory;

    // printing out ref. frame for the robot and end -eff name
    ROS_INFO ("Reference frame:%s", group.getPlanningFrame().c_str());
    ROS_INFO ("Reference frame:%s", group.getEndEffectorLink().c_str());

    //I use the home position designed in the SRDF
    group.setNamedTarget ("home");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan ( my_plan );

    if (success)
    group.execute (my_plan);
    else ROS_INFO ("Impossible to plan for the home position, keep planning from singular position");

    // we plan now from this position.if( success ) the current state is NON - SINGULAR
    group.setStartStateToCurrentState ();


    // ==================================
    // ===== Planning to a pose goal ======
    // ==================================
    // Desired position and orientation for the eef


    geometry_msgs::Pose target_pose1;

    // here I define the message
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.132;
    target_pose1.position.y = 0.545;
    target_pose1.position.z = 0.647;

    // now i give this setpoint to my group object
    group.setPoseTarget (target_pose1);

    // let 's call the planner to compute and visualize this plan
    success = group.plan ( my_plan );
    ROS_INFO (" Visualizing plan 1 ( pose goal for the eef ) %s", success ? "":" FAILED ");

    // give time to rviz to visualize
    sleep (10.0);

    // ************************************************************
    // now we are going to introduce collision objects and see how trajectory changes

    //I define a collision object message first
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.header.frame_id = group.getPlanningFrame ();
    collision_object1.id = " obstacle1 ";

    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;

    primitive1.dimensions.resize (3);
    primitive1.dimensions [0] = 0.3;
    primitive1.dimensions [1] = 0.1;
    primitive1.dimensions [2] = 0.3;

    //I place the box in the space , relatively to frame_id selected
    above
    geometry_msgs::Pose obstacle1_pose;

    obstacle1_pose.orientation.w = 1.0;
    obstacle1_pose.position.x = 0.0;
    obstacle1_pose.position.y = 0.3;
    obstacle1_pose.position.z = 0.75;

    // my shape message
    collision_object1.primitives.push_back (primitive1);
    collision_object1.primitive_poses.push_back (obstacle1_pose);
    collision_object1.operation = collision_object1.ADD;

    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = group.getPlanningFrame ();
    collision_object2.id = " obstacle2 ";

    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;

    primitive2.dimensions.resize (3);
    primitive2.dimensions [0] = 0.3;
    primitive2.dimensions [1] = 0.3;
    primitive2.dimensions [2] = 0.1;

    geometry_msgs::Pose obstacle2_pose;

    obstacle2_pose.orientation.w = 1.0;
    obstacle2_pose.position.x = 0.0;
    obstacle2_pose.position.y = 0.5;
    obstacle2_pose.position.z = 0.2;

    collision_object2.primitives.push_back ( primitive2 );
    collision_object2.primitive_poses.push_back ( obstacle2_pose );
    collision_object2.operation = collision_object2.ADD;

    // now i customize a vector , making it of type moveit_msgs::CollisionObject
    //I push back in it the two messages I created for the collision objects
    std::vector < moveit_msgs::CollisionObject > collision_objects;
    collision_objects.push_back (collision_object1);
    collision_objects.push_back (collision_object2);

    // now we effectively add the objects into the world
    ROS_INFO (" Obstacles spawn in the world ");
    planning_scene_interface.addCollisionObjects (collision_objects);

    // sleep to see the obstacles in rviz
    sleep (5.0);

    // let 's increase allotted time for planning when obstacles are present
    group.setPlanningTime (8.0);

    // now we give THE SAME pose setpoint and the arm avoids the obstacle
    group.setPoseTarget ( target_pose1 );
    success = group.plan ( my_plan );

    ROS_INFO (" Visualizing same target pose avoiding obstacles ...% s", success ? "" : " FAILED ");
    sleep (20.0);

    return 0;
}