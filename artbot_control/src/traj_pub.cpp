#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "traj_pub");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher traj_pub_j1 = n.advertise<std_msgs::Float64>("/artbot/joint1_position_controller/command", 1000);
  ros::Publisher traj_pub_j2 = n.advertise<std_msgs::Float64>("/artbot/joint2_position_controller/command", 1000);
  ros::Publisher traj_pub_j3 = n.advertise<std_msgs::Float64>("/artbot/joint3_position_controller/command", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many sets of messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {

  // Check ROS time and calculate desired cartesian (task-space) position
  double t = ros::Time::now().toSec();

  double x_pos, y_pos, z_pos, s, r, D, L1, L2, L3, pi;  // create cartesian position variables
  pi = 3.14159265359;
  L1 = 0.346;
  L2 = 0.55;
  L3 = 0.45;
  x_pos = 0.1*sin(t/2.5);
  y_pos = 0.1*cos(t/2.5);
  z_pos = L1 + L2 + L3 - 0.1;

// Create message objects
  std_msgs::Float64 j1_command, j2_command, j3_command;

// Perform calculations to translate task-space to joint-space
  j1_command.data = atan2(y_pos,x_pos);

  s = z_pos - L1; //intermediate calculations
  r = sqrt(pow(x_pos,2.0) + pow(y_pos,2));
  D = (pow(r,2.0) + pow(s,2.0) - pow(L2,2.0) - pow(L3,2.0))/(2.0*L2*L3);

  j3_command.data = atan2(-1*sqrt(1.0-pow(D,2.0)),D);

  j2_command.data = atan2(s,r) - atan2(L3*sin(j3_command.data),L2 + L3*cos(j3_command.data));

  // Adjust position and direction to match conventions assumed by DH parameterization
  j2_command.data = -1*j2_command.data + pi/2;
  j3_command.data = -1*j3_command.data;

//    ROS_INFO("%lf", j1_command.data);
//    ROS_INFO("%lf", j2_command.data);
//    ROS_INFO("%lf", j3_command.data);

    // Use publish() function to send message objects
    traj_pub_j1.publish(j1_command);
    traj_pub_j2.publish(j2_command);
    traj_pub_j3.publish(j3_command);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
