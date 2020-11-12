#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <Math.h>

geometry_msgs::Vector3 v;
geometry_msgs::Vector3 w;

double ppr = 7;//pulses per rotation
double wl = 0;
double wr = 0;
double l = 0.470642;
double r = 0.0508;
double nr = 0;
double nl = 0;

void velCallback(const geometry_msgs::Twist::ConstPtr &values)
{
    v = values->linear;
    w = values->angular;
    wr = (2*v.x + w.z*l)/(2*r);
    wl = (2*v.x - w.z*l)/(2*r);
    nr = (wr/(2*M_PI))*ppr;
    nl = (wl/(2*M_PI))*ppr;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "diff_mov");

  ros::NodeHandle n;
  ros::Suscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, velCallback)
  ros::Publisher wr_pub = n.advertise<std_msgs::Float64>("wr",10);
  ros::Publisher wl_pub = n.advertise<std_msgs::Float64>("wl",10);
  std_msgs::Float64 wrm;
  std_msgs::Float64 wlm;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(20);
  while(n.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    wrm.data = nr;
    wlm.data = nl;
    wr_pub.publish(wrm);
    wl_pub.publish(wlm);
    last_time = current_time;
    r.sleep();
  }
} 