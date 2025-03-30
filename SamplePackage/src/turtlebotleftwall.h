#ifndef TURTLEBOTLEFTWALL_H
#define TURTLEBOTLEFTWALL_H

#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <string>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <turtlebot3_msgs/SensorState.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define MINDIST 0.25
#define MAXDIST 0.30
#define FARDIST 0.5




class TurtleBotLeftWall
{
    public:
        TurtleBotLeftWall(int argc, char **argv);
        ~TurtleBotLeftWall();

        void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
        void sensorStateCallback(const turtlebot3_msgs::SensorState::ConstPtr &msg);
        void PublishVelocity(float lin, float ang) const;

        bool buttonPressed() const;

        void ControlLoop();

    private:

        float mFwdDist;
        float mFLDist;
        float mLeftDist;
        float mFRDist;
        float mRightDist;

        float mPose;
        float mPrev_pose;

        bool mButtonFlag;

        ros::Publisher mPub_vel_;
        ros::Subscriber mSub_laser_;
        ros::Subscriber mOdom_sub_;
        ros::Subscriber mSnsrState_sub;

        ros::NodeHandle nh;
        ros::NodeHandle pn;

        enum State{
            FOLLOWWALL,
            WALLEND,
            RIGHTTURN
        };
};

#endif // TURTLEBOTLEFTWALL_H
