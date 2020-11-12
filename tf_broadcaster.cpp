/*
  tf_broadcaster.cpp

  Author: Cesar Abraham Acevedo Ruiz
  
  Description: This code was made from the tutorial of the ROS navigation stack on how to configure tf in order to use it for mapping
  and navigation. It comunicates with the tf node and reports  the position and orientation of sensors with in relation with a
  reference. For the PiBOT, it's just the laser position.

  For more information on this, visit the page: http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;

  ros::Rate r(10);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    //broadcaster.sendTransform() is the function used for sending the transform data to the tf node
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.218703, 0, 0.5480455)),
        ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}