/*
  odom_broadcaster.cpp

  Authors: Cesar Abraham Acevedo Ruiz and Juan Angel Gonzales Aguirre

  Description: This code is for computing odometry information from acelerometer information of the "acc" topic in the
  linearAcceleration_and_angularVelocity.cpp node, and orientation information of the "AIkits/imu" topic in the AIkit_republisher.cpp
  node. It also aplies a moving average filter to the acceleration, and a diferential band in order to determine when to integrate,
  in order to reduce the risk of "drift" in operation.
*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <new>
#include <iostream>

#define NODERATE 50
#define NUM 2

//Moving average filter declaration
using namespace std;
struct movingAverage
{
    private:
        float aux1, aux2;
        bool initializedQ = false;
    public:
    int numberOfSamples;
    float accumulative = 0;
    float * pointerToSamples;

    bool initialize()
    {
        if(!initializedQ)
        {
            pointerToSamples = new (nothrow) float [numberOfSamples];
            if (pointerToSamples == nullptr)
            {
                return false;
            }
            else
            {
                initializedQ = true;
            for (float * i = &pointerToSamples[0]; i != &pointerToSamples[numberOfSamples] ; ++i)
            {
                *i = 0;
            }
                return true;
            }
        }
    }

    float mean(float actualSample)
    {
        //First; move the data and append the new sample to the begining of the array
        accumulative = 0;
        accumulative += actualSample;
        for (float * i = &pointerToSamples[0]; i != &pointerToSamples[numberOfSamples] ; ++i)
        {
            accumulative += *i;
            *i = *(i+1);
        }
        *(pointerToSamples + numberOfSamples-1) = actualSample;
        return accumulative/numberOfSamples;
    }

    void printArray()
    {
        for (float * i = &pointerToSamples[0]; i != &pointerToSamples[numberOfSamples] ; ++i)
        {
            cout << *i << endl; 
        }
    }
};
//Moving average code ends

//orientation variable
double imuOrientation[4] = {0,0,0,1};

//angular velocity variable
double w = 0;

//acceleration variables
double accx[NUM] = {};
double accy[NUM] = {};

double accxl[NUM] = {};
double accyl[NUM] = {};

//number of samples for the moving average filter
int numberOfSamplesForMean = 20;

//declaring movingAverage objects
movingAverage mean1,
              mean2, 
              mean3,
              mean4;

//OPTIONAL: This virtual low pass filter can be used instead of the moving average, if computational resources are low
void lowPassFilter(double dt,double rc)
{
  double a = dt/(rc + dt);
  accxl[0] = accxl[1];//a*accx[0];
  accxl[1] = a*accx[1] + (1-a)*accxl[0];
  accyl[0] = accyl[1];//a*accy[0];
  accyl[1] = a*accy[1] + (1-a)*accyl[0];
}

//Receiving orientation data
void imuCallback(const sensorsMsgs::Imu::ConstPtr& msg)
{
  imuOrientation[0] = msg->orientation.x;
  imuOrientation[1] = msg->orientation.y;
  imuOrientation[2] = msg->orientation.z;
  imuOrientation[3] = msg->orientation.w;
}

//Receiving acceleration data
void accelerationCallback(const geometry_msgs::Vector3::ConstPtr& values)
{
  geometry_msgs::Vector3 acc;
  acc.x = values->x;
  acc.y = values->y;
  acc.z = values->z;
  accx[0] = accx[1];
  accy[0] = accy[1];
  accx[1] = acc.x;
  accy[1] = acc.y;
}

//receiving angular velocity data
void angularVelocityCallback(const geometry_msgs::Vector3::ConstPtr& values)
{
  w = values->z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber acc1_sub = n.subscribe("acc",10,accelerationCallback);
  ros::Subscriber ang1_sub = n.subscribe("ang",10,accelerationCallback);
  ros::Subscriber imu1_sub = n.subscribe("AIkits/imu",10,imuCallback);


  //initialization of variables for pose calculation
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0;
  double vy = 0;
  double vth = 0;

  //time variables for integration
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();


  mean1.numberOfSamples = numberOfSamplesForMean;
  mean1.initialize();
  mean2.numberOfSamples = numberOfSamplesForMean;
  mean2.initialize();
  mean3.numberOfSamples = numberOfSamplesForMean;
  mean3.initialize();
  mean4.numberOfSamples = numberOfSamplesForMean;
  mean4.initialize();


  ros::Rate r(NODERATE);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    //TODO: Revisar si las ruedas se están moviendo, si es asi: integrar
    
    double dt = (current_time - last_time).toSec();
    //lowPassFilter(dt,1);//filtering using low pass filter
    
    //filtering using moving average
    accxl[1] = mean1.mean(accx[1]);
    accyl[1] = mean2.mean(accy[1]);
    

    double delta_vx = 0;
    double delta_vy = 0;

    //determination of whether to integrate or not
    if(accxl[1]<0.02 && accxl[1]>-0.02)
    {
      delta_vx = 0; 
    }
    else
    {
      delta_vx = accxl[1]*dt;
    }

    if(accyl[1]<0.02 && accyl[1]>-0.02)
    {
      delta_vy = 0; 
    }
    else
    {
      delta_vy = accyl[1]*dt;
    }

    vx += delta_vx;
    vy += delta_vx;
    
    //second integration
    double delta_x = vx * dt;
    double delta_y = vy * dt;
    
    x += delta_x;
    y += delta_y;
    geometry_msgs::Quaternion odom_quat(imuOrientation[0],imuOrientation[1],imuOrientation[2],imuOrientation[3]);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = w;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}