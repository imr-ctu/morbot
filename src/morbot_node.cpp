#include <string>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

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
  signed char v_char = static_cast<signed char>(v);
  signed char a_char = static_cast<signed char>(a);
  ROS_DEBUG("Set linear and angular velocities: (%d, %d)", v_char, a_char);
  if (!_avr.setSpeedAndTurn(v_char, a_char))
  {
    ROS_ERROR("Communication error on serial port");
  }
}

bool resetOdometryCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return _avr.setPosition(0, 0, 0);
}

int main(int argc, char **argv)
{
  float x;
  float y;
  float yaw;

  ros::init(argc, argv, "morbot");
  ros::NodeHandle nh("~");

  std::string device_name;
  nh.param<std::string>("serial_port", device_name, "/dev/ttyUSB0");

  if (!_avr.openSerial(device_name.c_str()))
  {
    ROS_FATAL_STREAM("Communication error on " << device_name << ", exiting");
    return 1;
  }
  if (!_avr.setMotorsOn())
  {
    ROS_FATAL_STREAM("Communication error on " << device_name << " when enabling motors, exiting");
    return 1;
  }
  if (!_avr.setSpeedAndTurn(0, 0))
  {
    ROS_FATAL_STREAM("Communication error on " << device_name << " when setting velocities, exiting");
    return 1;
  }

  ros::ServiceServer reset_odom_server = nh.advertiseService("reset_odometry", resetOdometryCallback);

  ros::Subscriber vel = nh.subscribe("cmd_vel", 1, velCallback);

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("pose", 1000);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);

  ROS_INFO("MORBOT is ready");

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    ROS_DEBUG_NAMED("superdebug", "Getting position");
    if (!_avr.getPosition(&x, &y, &yaw))
    {
      ROS_ERROR_STREAM("Communication error on " << device_name << " when getting position");
    }
    else
    {
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
    }
    loop_rate.sleep();
  }

  if (!_avr.setMotorsOff())
  {
    ROS_FATAL_STREAM("Communication error on " << device_name << " when disabling motors, exiting");
    return 1;
  }

  return 0;
}


