#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>



/*http://wiki.ros.org/rviz/Tutorials/MarkersBasicShapes*/
/* Initial Setup */

// integer indicator marker_reached_state = 0;
uint8_t goal_reached_state = 0; 

/* robot goal proximity callback function; callback function goalReachCallback updates the goal_reached_state variable whenever a new message is received on the /goal_reached topic.
Callback functions in ROS are triggered when a new message is received on a topic to which this callback is subscribed */

/* 8 bit integer (& - efficiently passed by reference, avoids copying entire message) 
this is a constant shared pointer is used, the content of this pointer cannot be modified */
void goalReachedCallback(const std_msgs::UInt8::ConstPtr& msg) 
{
   goal_reached_state = msg->data; // used to access actual data contained in std_msgs::UInt8 message
   return;
}

/* Main function; entry point of C++ prgm */

/* argc stands for argument count, this integer represents the number of command-line arguments passed to the prgm
this includes the prgm itself
argv stand for argument vector, this is an array of C-style strings representing actual command-line arguments*/

int main( int argc, char** argv ) 
{
// Initializes the ROS node named "add_markers"
  ros::init(argc, argv, "add_markers");

// Creates a NodeHandle for communication with the ROS system and sets the loop rate to 5 Hz(5 times per second).
  ros::NodeHandle n;
  ros::Rate r(5);

/* Sets up a Publisher to publish marker messages on the visualization_marker topic and a 
Subscriber to subscribe to the /goal_reached topic, with goalReachCallback as the callback function.*/
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/goal_reached", 1, goalReachedCallback);

// Initializes a boolean done to indicate if the process is complete and sets the initial shape type to a cube.
  bool done = false;

// Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

// Logs an info message indicating subscription to the goal position topic
  ROS_INFO("Subscribed to requested goal-position");

  visualization_msgs::Marker marker;
  // Setting of frame ID and timestamp
  marker.header.frame_id = "/map";
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.5; // sets size of marker; all dimension are 0.5
  marker.color.r = 0.0f; // sets red component to 0.0 (no red)
  marker.color.g = 0.0f; // sets green component to 0.0 (no green)
  marker.color.b = 1.0f; // sets blue component to 1.0 (full blue)
  marker.color.a = 1.0;  // sets the aplha (transparency) component to 1.0 (Opaque)


/* Main Loop */

  while (ros::ok()) // loop runs while ROS is running
  {
      ros::spinOnce(); // processes incoming messages
      marker.header.stamp = ros::Time::now(); // updates marker timestamp 
      marker.type = shape; // updates marker type
  }

// Marker Publishing Logic based on goal_reached_state

/*    
case 0 adds a marker at the pick-up location.
case 1 deletes the pick-up marker.
case 2 waits without publishing any marker.
case 3 adds a marker at the drop-off location and sets done to true.
*/

  switch (goal_reached_state) {
    case 0: // publish pick-up marker
        marker.action = visualization_msgs::Marker::ADD; // sets action (ADD) that will be performed on the marker; 
        n.getParam("/pick_up_location/tx", marker.pose.position.x);
        n.getParam("/pick_up_location/ty", marker.pose.position.y);
        n.getParam("/pick_up_location/tz", marker.pose.position.z);
        n.getParam("/pick_up_location/qx", marker.pose.orientation.x);
        n.getParam("/pick_up_location/qy", marker.pose.orientation.y);
        n.getParam("/pick_up_location/qz", marker.pose.orientation.z);
        n.getParam("/pick_up_location/qw", marker.pose.orientation.w);
        break;

    case 1: // robot has reached pickup site, pick-up marker is deleted 
        sleep(2);
        marker.action = visualization_msgs::Marker::DELETE; // sets action (DELETE) that will be performed on the marker; 
        break;

    case 2: // waiting for robot to reach drop-off location...
        marker.action = visualization_msgs::Marker::DELETE;
        break;

    case 3: //  publishing drop-off marker
        sleep(5);
        marker.action = visualization_msgs::Marker::ADD;
        n.getParam("/drop_off_location/tx", marker.pose.position.x);
        n.getParam("/drop_off_location/ty", marker.pose.position.y);
        n.getParam("/drop_off_location/tz", marker.pose.position.z);
        n.getParam("/drop_off_location/qx", marker.pose.orientation.x);
        n.getParam("/drop_off_location/qy", marker.pose.orientation.y);
        n.getParam("/drop_off_location/qz", marker.pose.orientation.z);
        n.getParam("/drop_off_location/qw", marker.pose.orientation.w);
        done = true;
        break;
}

// Final marker Publishing 

/*    
Ensures there is at least 1 subscriber before publishing the marker
If there are no subscribers it waits and logs a warning
*/

while (marker_pub.getNumSubscribers() < 1) 
{
    if (!ros::ok()) 
    {
        return 0;
    }
    
    ROS_WARN_ONCE("Subscriber required for marker");
    sleep(1);
}

// Publish marker

/*

If done is true, logs that the destination is reached, waits for 7 seconds, and exits the program.
r.sleep() ensures the loop runs at the defined rate (5 Hz).
*/

marker_pub.publish(marker);

// If last marker published and noted as done, exit
if (done) {
    ROS_INFO("### Goal Achieved!###");
    sleep(10);
    return 0;
}

r.sleep();

return 0;
}






