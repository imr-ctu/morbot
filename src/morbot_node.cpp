#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <pthread.h>
#include <math.h>
#include <sstream>
#include <string>

#include <morbot/avr.h>

AVR _avr;

void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_DEBUG("I heard: twist = [%.3f  %.3f]", msg->linear.x, msg->angular.z);

  float a = floor(msg->angular.z * 13.68) ;
  float v = msg->linear.x * 125; 
  if (v > 127)
    v = 127;
  if (v < -128)
    v = -128;
  if (a > 127)
    a = 127;
  if (a < -128)
    a = -128;
  _avr.setSpeedAndTurn((signed char) v, (signed char) a);
}

int main(int argc, char **argv)
{
  float x;
  float y;
  float yaw;

  //   _avr.setSpeedAndTurn(10,0);
  ros::init(argc, argv, "morbot");
  ros::NodeHandle nh("~");

  std::string device_name;
  nh.param<std::string>("serial_port", device_name, "/dev/ttyUSB0");

  _avr.openSerial(device_name.c_str());
  _avr.setMotorsOn();
  _avr.setSpeedAndTurn(0, 0);

  ros::Subscriber vel = nh.subscribe("cmd_vel", 1000, velCallback);

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("pose", 1000);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

  ROS_INFO("MORBOT is ready");

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    _avr.getPosition(&x, &y, &yaw);

    ros::Time timestamp = ros::Time::now();

    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = yaw;
    pose_pub.publish(pose); 

    nav_msgs::Odometry odom;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    geometry_msgs::Quaternion q_yaw = tf::createQuaternionMsgFromYaw(yaw);
    odom.pose.pose.orientation = q_yaw;
    odom.header.stamp = timestamp;
    odom_pub.publish(odom);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(tf::createQuaternionFromYaw(yaw));
    br.sendTransform(tf::StampedTransform(transform, timestamp, "world", "base_link"));
    loop_rate.sleep();
  }
  _avr.setMotorsOff();

  return 0;
}

