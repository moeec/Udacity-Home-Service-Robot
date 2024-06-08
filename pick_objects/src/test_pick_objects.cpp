#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



/* This code is designed to control a robot, guiding it to pick up and drop off locations using ROS's move_base action server.*/
// Action client created to send goal requests to the move_base server via SimpleActionClient
// Based off of https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient

// Define a convenient alias MoveBaseClient, this communicates with the move_base server using MoveBaseAction 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// main function 
int main(int argc, char** argv)
  {
  // Initialize simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  /* Creates an instance of MoveBaseClient named ac that connects to the move_base action server.
    (true) tells the client to start a new thread to process action messages, allowing the main thread to continue execution 
    without being blocked by action communication.*/
  MoveBaseClient ac("move_base", true);

  // while loop waits for the move_base action server to be available, checking every 5 seconds.
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    //logs message every 5 seconds indicating the wait for the action server to start.
    ROS_INFO("Waiting for the move_base action server...(5 seconds)"); 
  }

  //  Create an instance of MoveBaseGoal. This is used to define the target position and orientation for the robot.
  move_base_msgs::MoveBaseGoal goal;		

  // Sets the coordinate frame for the goal as "map".
  goal.target_pose.header.frame_id = "map";

  // Sets timestamp for the goal to the current time. Used for synchronizing the goal with the robot's internal state.
  goal.target_pose.header.stamp = ros::Time::now();

  /* node-handle used to interface with the ROS parameter server, retrieve parameters, 
  and manage the node's communication.*/
  ros::NodeHandle n; 

  
  // Setting and Sending the Pick-up Goal
  
  /* Retrieves the X, Y & Z position & Orientation of the pick-up location from the parameter server 
  and assigns it to the goal position's X coordinate.*/
  n.getParam("/pick_up_loc/tx", goal.target_pose.pose.position.x);
  n.getParam("/pick_up_loc/ty", goal.target_pose.pose.position.y);
  n.getParam("/pick_up_loc/tz", goal.target_pose.pose.position.z);
  n.getParam("/pick_up_loc/qx", goal.target_pose.pose.orientation.x);
  n.getParam("/pick_up_loc/qy", goal.target_pose.pose.orientation.y);
  n.getParam("/pick_up_loc/qz", goal.target_pose.pose.orientation.z);
  n.getParam("/pick_up_loc/qw", goal.target_pose.pose.orientation.w);

  // Sends defined goal to the move_base action server. Instructs robot to navigate specified pick-up location.
  ac.sendGoal(goal);

  // Logs a message indicating that the pick-up goal has been sent.
  ROS_INFO("Sending pick-up goal");
  // Waits indefinitely for the result of the goal
  ac.waitForResult();

  // Checks if the robot successfully reached the pick-up location.
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
  {
    // Logs success message if the robot reached the pick-up location.
    ROS_INFO("The robot has reached PICK-UP location");
    /* Pauses execution (5 seconds) before continuing, this gives ample time between 
    reaching the pick-up location and sending the next goal.*/
    ros::Duration(5.0).sleep();
  }
  else 
  {
    // Logs a failure message if the robot did not reach the pick-up location.
    ROS_INFO("The robot failed to reach pick-up location");
    
    // Exits the program if the robot failed to reach the pick-up location.
    return 0;
  }

  // Setting and Sending the Drop-off Goal

  // Retrieves the X, Y & Z position & Orientation for the drop-off location from the parameter server.
  n.getParam("/drop_off_loc/tx", goal.target_pose.pose.position.x);
  n.getParam("/drop_off_loc/ty", goal.target_pose.pose.position.y);
  n.getParam("/drop_off_loc/tz", goal.target_pose.pose.position.z);
  n.getParam("/drop_off_loc/qx", goal.target_pose.pose.orientation.x);
  n.getParam("/drop_off_loc/qy", goal.target_pose.pose.orientation.y);
  n.getParam("/drop_off_loc/qz", goal.target_pose.pose.orientation.z);
  n.getParam("/drop_off_loc/qw", goal.target_pose.pose.orientation.w);

  // Logs message indicating that the drop-off goal is being sent.
  ROS_INFO("Sending drop-off goal");

  // Sends the drop-off goal to the move_base action server.
  ac.sendGoal(goal);
  
  // Waits indefinitely for robot to reach the drop-off goal or fail.
  ac.waitForResult();

  // Checks if robot successfully reached the drop-off location.
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
  {
    // Logs a success message if the robot reached the drop-off location
    ROS_INFO("The robot has reached DROP-OFF location");
  }
  else 
  {
    // Logs a failure message if robot did not reach the drop-off location.
    ROS_INFO("The robot FAILED to reach drop-off location");
  }
  
  // Final exit; Ends the program, indicating successful or unsuccessful completion of the task.
  return 0;
}
