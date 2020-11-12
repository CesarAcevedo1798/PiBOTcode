/*
    linearAcceleration_and_angularVelocity.cpp

    Authors: Cesar Abraham Acevedo Ruiz and Juan Angel Gonzales Aguirre

    Description: This node receives both the linear acceleration and angular velocity from the AIkits/imu topic, and with the
    orientation data, normalize it to the global frame of reference, in order to reduce "drift" that occurs when an
    accelerometer isn't fully leveled, for this it uses a rotational matrix multiplicating the column vectors of both acceleration
    and angular velocity. On top of that, it performs a moving average filter on these data to reduce noice from the orientation
    information.
*/

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <new>
#include <iostream>

#define NODERATE 200

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

double imuValues1[4] = {0,0,0,1};

//Declaration of both the acceleration message and the angular velocity message
geometry_msgs::Vector3 a1;
geometry_msgs::Vector3 w1;

//moving average object definition
movingAverage mean1;
movingAverage mean2;
movingAverage mean3;
movingAverage mean4;
movingAverage mean5;
movingAverage mean6;

int numberOfSamplesForMean = 20;//Declaration of the number of samples that the moving average filter will perfom

void AIkitCallback(const sensor_msgs::Imu::ConstPtr &values)
{
  //creation of the rotational matrix
  imuValues1[0] = values->orientation.x;
  imuValues1[1] = values->orientation.y;
  imuValues1[2] = values->orientation.z;
  imuValues1[3] = values->orientation.w;
  tf::Quaternion q(imuValues1[0],imuValues1[1],imuValues1[2],imuValues1[3]);
  tf::Matrix3x3 m(q);
  
  //acceleration and velocity handler of the callback function
  geometry_msgs::Vector3 a;
  geometry_msgs::Vector3 w;
  a = values->linear_acceleration;
  w = values->angular_velocity;
  
  //filtering stage
  double axf = mean1.mean(a.x);
  double ayf = mean2.mean(a.y);
  double azf = mean3.mean(a.z);
  double wxf = mean4.mean(w.x);
  double wyf = mean5.mean(w.y);
  double wzf = mean6.mean(w.z);
  
  //linear acceleration normalization
  a1.x = axf*m[0][0] + ayf*m[0][1] + azf*m[0][2];
  a1.y = axf*m[1][0] + ayf*m[1][1] + azf*m[1][2];
  a1.z = axf*m[2][0] + ayf*m[2][1] + azf*m[2][2];
  
  //angular velocity normalization
  w1.x = wxf*m[0][0] + wyf*m[0][1] + wzf*m[0][2];
  w1.y = wxf*m[1][0] + wyf*m[1][1] + wzf*m[1][2];
  w1.z = wxf*m[2][0] + wyf*m[2][1] + wzf*m[2][2];
}

int main(int argc, char** argv){
  ros::init(argc, argv, "linearAcceleration_and_angularVelocity");
  ros::NodeHandle n;
  
  //Inicialization of the movingAverage objects
  mean1.numberOfSamples = numberOfSamplesForMean;
  mean1.initialize();
  mean2.numberOfSamples = numberOfSamplesForMean;
  mean2.initialize();
  mean3.numberOfSamples = numberOfSamplesForMean;
  mean3.initialize();
  mean4.numberOfSamples = numberOfSamplesForMean;
  mean4.initialize();
  mean5.numberOfSamples = numberOfSamplesForMean;
  mean5.initialize();
  mean6.numberOfSamples = numberOfSamplesForMean;
  mean6.initialize();

  ros::Rate r(NODERATE);

  //Subscribers and Publishers declaration
  ros::Subscriber AIkit_sub = n.subscribe("AIkits/imu",10,AIkit1Callback);
  ros::Publisher acc_pub = n.advertise<geometry_msgs::Vector3>("acc", 10);
  ros::Publisher ang_pub = n.advertise<geometry_msgs::Vector3>("ang", 10);

  while(n.ok()){
    acc_pub.publish(a1);//Publishes normalized linear acceleration data on "acc" topic
    ang_pub.publish(w1);//Publishes normalized angular velocity data on "ang" topic
    ros::spinOnce();
    r.sleep();
  }
}