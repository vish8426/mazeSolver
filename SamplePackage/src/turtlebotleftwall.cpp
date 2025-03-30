#include "turtlebotleftwall.h"

float testInf(float range)
{
    if(std::isinf(range) || range == 0)
        return 100.0;
    else
        return range;
}


//------------------------------------------------------------------------------------------------
TurtleBotLeftWall::TurtleBotLeftWall(int argc, char **argv)
    : pn("~"),
      mButtonFlag(false)
{
    mPub_vel_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    mSub_laser_ = nh.subscribe("/scan", 100, &TurtleBotLeftWall::LaserCallback, this);
    mOdom_sub_ = nh.subscribe("/odom", 100, &TurtleBotLeftWall::odomMsgCallBack, this);
    mSnsrState_sub = nh.subscribe("/sensor_state", 10, &TurtleBotLeftWall::sensorStateCallback, this);
}

//--------------------------------------------------------------
TurtleBotLeftWall::~TurtleBotLeftWall(){}

//-----------------------------------------------
void TurtleBotLeftWall::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int F = 0;
    int FL = 45-5;
    int L = 90-5;
    int R = 270-5;
    int FR = 315-5;

    mFwdDist = testInf(msg->ranges[F]);
    mFLDist = testInf(msg->ranges[FL]);
    mLeftDist = testInf(msg->ranges[L]);
    mRightDist = testInf(msg->ranges[R]);
    mFRDist = testInf(msg->ranges[FR]);

    for(int i = 0; i < 10; i++)
    {
        if(mFLDist > testInf(msg->ranges[FL+i]))
            mFLDist = testInf(msg->ranges[FL+i]);

        if(mLeftDist > testInf(msg->ranges[L+i]))
            mLeftDist = testInf(msg->ranges[L+i]);

        if(mRightDist > testInf(msg->ranges[R+i]))
            mRightDist = testInf(msg->ranges[R+i]);

        if(mFRDist > testInf(msg->ranges[FR+i]))
            mFRDist = testInf(msg->ranges[FR+i]);

        if(i <= 9)
        {
            if(mFwdDist > testInf(msg->ranges[F+i]))
                mFwdDist = testInf(msg->ranges[F+i]);
        }
        else
        {
            if(mFwdDist > testInf(msg->ranges[F+340+i]))
                mFwdDist = testInf(msg->ranges[F+340+i]);
        }

    }
    //ROS_INFO("F:%f FL:%f L:%f", mFwdDist, mFLDist, mLeftDist);
    return;
}

//-------------------
void TurtleBotLeftWall::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double yaw,pitch,roll;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    mPose = yaw;
}

//------------------------------------------------------------------
void TurtleBotLeftWall::sensorStateCallback(const turtlebot3_msgs::SensorState::ConstPtr &msg)
{
    if(msg->button == 2)
        mButtonFlag = true;
}

//---------------------------------------------------------------------------------------------
void TurtleBotLeftWall::PublishVelocity(float lin, float ang) const
{
        geometry_msgs::Twist msg;

        msg.linear.x = lin;
        msg.angular.z = ang;

        mPub_vel_.publish(msg);
}

//-----------------------------------
bool TurtleBotLeftWall::buttonPressed() const
{
    return mButtonFlag;
}

//-------------------------------------------------------------------------------------------------
void TurtleBotLeftWall::ControlLoop()
{
    float lin, ang;

    static State TurtleState = FOLLOWWALL;

    switch(TurtleState)
    {

        // Follow the wall and make corrections while traveling
        case FOLLOWWALL:
            if(mFwdDist > MAXDIST)
            {
                if(mFLDist < FARDIST )
                {
                    if(mFLDist < MINDIST){
                        lin = 0.2;
                        ang = -0.3;
                        PublishVelocity(lin, ang);
                        break;

                    } else if(mFLDist > MAXDIST) {
                        lin = 0.2;
                        ang = 0.3;
                        PublishVelocity(lin, ang);
                        break;

                    } else {
                        PublishVelocity(0.3,0);
                        break;
                    }
                } else
                    TurtleState = WALLEND;
                    break;
            } else
            {
                TurtleState = RIGHTTURN;
                break;
            }
        case WALLEND:
            if(mFLDist > MAXDIST)
            {
                PublishVelocity(0.03,0.6);
                break;
            } else if(mLeftDist > MAXDIST && mFLDist > MINDIST)
            {
                PublishVelocity(0.1,0);
                break;
            } else
            {
                TurtleState = FOLLOWWALL;
                break;
            }
        case RIGHTTURN:
            if(mFwdDist > MAXDIST)
            {
                TurtleState = FOLLOWWALL;
            } else
                PublishVelocity(0,-0.8);
            break;
        default:
            PublishVelocity(0,0);
    }

    ROS_INFO("STATE: %d",TurtleState);

    return;
}

