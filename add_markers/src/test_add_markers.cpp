#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv ) 
{
// Initializes the ROS node named "add_markers"
  ros::init(argc, argv, "add_markers");

// Creates a NodeHandle for communication with the ROS system and sets the loop rate to 5 Hz(5 times per second).
  ros::NodeHandle n;
  ros::Rate r(1); // Loop at 1Hz

/* Sets up a Publisher to publish marker messages on the visualization_marker topic and a 
Subscriber to subscribe to the /goal_reached topic, with goalReachCallback as the callback function.*/
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

// counter used for marker
  uint32_t marker_counter = 0;


// Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    visualization_msgs::Marker marker;
    // Setting of frame ID and timestamp
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape; // updates marker type
    marker.scale.x = marker.scale.y = marker.scale.z = 0.5; // sets size of marker; all dimension are 0.5
    marker.color.r = 0.0f; // sets red component to 0.0 (no red)
    marker.color.g = 0.0f; // sets green component to 0.0 (no green)
    marker.color.b = 1.0f; // sets blue component to 1.0 (full blue)
    marker.color.a = 1.0;  // sets the aplha (transparency) component to 1.0 (Opaque)

    switch (marker_counter) {
      case 0: // Publish pick-up marker
        {
          ROS_INFO("publishing marker at PICK-UP zone ");
          marker.action = visualization_msgs::Marker::ADD;

          // Set the pose of the marker from parameters
          std::vector<std::string> params = {"/pick_up_loc/tx", "/pick_up_loc/ty", "/pick_up_loc/tz",
                                             "/pick_up_loc/qx", "/pick_up_loc/qy", "/pick_up_loc/qz", "/pick_up_loc/qw"};
          std::vector<double*> pose_values = {&marker.pose.position.x, &marker.pose.position.y, &marker.pose.position.z,
                                              &marker.pose.orientation.x, &marker.pose.orientation.y, &marker.pose.orientation.z, &marker.pose.orientation.w};
        
          for (size_t i = 0; i < params.size(); ++i) 
          {
              n.getParam(params[i], *pose_values[i]);
          } 

        // Publish marker and increment marker_counter
          marker_pub.publish(marker);
          marker_counter++;
          break;
         }

      case 1: // Pause for 5 seconds and hide pick-up marker
        {
          sleep(5);
          ROS_INFO("Hiding the PICK-UP marker");
          marker.action = visualization_msgs::Marker::DELETE;
          marker_pub.publish(marker);
          marker_counter++;
          break;
        }

      case 2: // Publish drop-off marker
        {
          sleep(5);
          ROS_INFO("Drop-off marker added");
          marker.action = visualization_msgs::Marker::ADD;

          // Set the pose of the marker from parameters
          std::vector<std::string> params = {"/drop_off_loc/tx", "/drop_off_loc/ty", "/drop_off_loc/tz",
                                             "/drop_off_loc/qx", "/drop_off_loc/qy", "/drop_off_loc/qz", "/drop_off_loc/qw"};
          
          std::vector<double*> pose_values = {&marker.pose.position.x, &marker.pose.position.y, &marker.pose.position.z,
                                              &marker.pose.orientation.x, &marker.pose.orientation.y, &marker.pose.orientation.z, &marker.pose.orientation.w};
        
          for (size_t i = 0; i < params.size(); ++i) 
          {
              n.getParam(params[i], *pose_values[i]);
          }
         
        }

          // Publish marker and increment marker_counter
          marker_pub.publish(marker);
          marker_counter++;
          break;

      case 3: // Wrap-up
        {
          ROS_INFO("Task Completed");
          break;
        }

      default:
        {
          ROS_INFO("ERROR: add-marker has encountered an error");
          break;
        }

      }

      r.sleep();
    }

 }
