#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // NodeHandle created interacting with the ROS system. This provides a way to create publishers, subscribers, services, and other interactions with the ROS master. 
  ros::NodeHandle nh;

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // This line sets up a publisher that will publish UInt8 messages to the topic "/goal_reached" with a queue size of 1.
   ros::Publisher goal_reach_pub = n.advertise<std_msgs::UInt8>("/goal_reached!!!", 1);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Declare a MoveBaseGoal object named goal 
  move_base_msgs::MoveBaseGoal goal;
  
  // Decalre a UInt8 message object named status_msg.
  std_msgs::UInt8 status_msg;
 

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Set the frame ID to "map" and the timestamp to the current time for the goal's target pose.
  ROS_INFO("publishing pick-up goal");

  std::vector<std::pair<std::string, double*>> params = 
  {
    {"/pick_up_loc/tx", &goal.target_pose.pose.position.x},
    {"/pick_up_loc/ty", &goal.target_pose.pose.position.y},
    {"/pick_up_loc/tz", &goal.target_pose.pose.position.z},
    {"/pick_up_loc/qx", &goal.target_pose.pose.orientation.x},
    {"/pick_up_loc/qy", &goal.target_pose.pose.orientation.y},
    {"/pick_up_loc/qz", &goal.target_pose.pose.orientation.z},
    {"/pick_up_loc/qw", &goal.target_pose.pose.orientation.w}
  };

  for (const auto& param : params) 
  {
    n.getParam(param.first, *(param.second));
  }
  ac.sendGoal(goal);

  // Wait indefinitely for the action server to finish processing the goal.
  ac.waitForResult();

  // Checks to see if the goal was successfully achieved. 
  // If so, it logs success messages, sets the status_msg to 1 (indicating the goal was reached), and publishes this status. 
  // If the goal was not achieved, it logs a failure message and returns.

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
  {
    ROS_INFO("Robot has reached PICK-UP location!");
    ROS_INFO("Package being picked up");
    status_msg.data = 1;
    goal_reach_pub.publish(status_msg);
  }
  else 
  {
    ROS_INFO("The robot did not arrive at the pick-up location");
    return 0;
  }

  // Send drop off the goal position and orientation for the robot to reach
  ROS_INFO("Publishing drop-off goal");
  // wait for next message
  ros::Duration(5.0).sleep();

  // Define a position and orientation for the robot to reach
  std::vector<std::pair<std::string, double*>> drop_off_params = 
  {
    {"/drop_off_loc/tx", &goal.target_pose.pose.position.x},
    {"/drop_off_loc/ty", &goal.target_pose.pose.position.y},
    {"/drop_off_loc/tz", &goal.target_pose.pose.position.z},
    {"/drop_off_loc/qx", &goal.target_pose.pose.orientation.x},
    {"/drop_off_loc/qy", &goal.target_pose.pose.orientation.y},
    {"/drop_off_loc/qz", &goal.target_pose.pose.orientation.z},
    {"/drop_off_loc/qw", &goal.target_pose.pose.orientation.w}
  };

  for (const auto& param : drop_off_params) 
  {
    n.getParam(param.first, *(param.second));
  }

  ac.sendGoal(goal);

  ROS_INFO("Heading to Drop-off site");

  
  // Wait indefinitely for the action server to finish processing the goal.
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray! dropping off package");
// Publish that goal has been reached status
    status_msg.data = 3;  
    goal_reach_pub.publish(status_msg);
  } 
  else
  {
    ROS_INFO("The robot did not arrive at the drop-up location");
  }

  // wait for next message
  ros::Duration(5.0).sleep();

  return 0;
}
