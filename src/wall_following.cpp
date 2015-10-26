#include "ros/ros.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "behaviour.hpp"
#include "seqsel/WallFollow.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_houston_wall_following");
    ros::NodeHandle n;

    behaviour<WallFollow> wf(n);

    wf.behave();

    return 0;
}
