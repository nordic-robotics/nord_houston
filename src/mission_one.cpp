#include "ros/ros.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "behaviour.hpp"
#include "mission_one_behaviour.hpp"
#include "seqsel/MissionOne.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_houston_wall_following");
    ros::NodeHandle n;

    mission_one_behaviour<MissionOne> b(n);

    b.behave();

    return 0;
}
