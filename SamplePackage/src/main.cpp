#include "turtlebotleftwall.h"

int main(int argc, char **argv)
{
    // Create LeftWallFollower
    // create a ROS node
    ros::init(argc, argv, "SampleTurtlebotNode");
    TurtleBotLeftWall turtle1(argc, argv);

    bool buttonPressed = false;

    ros::Rate loop_rate(30);
    while (ros::ok())
    { 
        if(buttonPressed){
            turtle1.ControlLoop();
        } else {
            if(turtle1.buttonPressed())
                buttonPressed = true;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
