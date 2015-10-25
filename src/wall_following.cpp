#include "ros/ros.h"
#include <cstdint>
#include <iostream>

enum class action : uint16_t
{
    forwards_speed,
    backwards_distance,
    turn_left,
    turn_right
};

bool publish(action, float);

#include "seqsel/WallFollow.hpp"

bool publish(action a, float f)
{
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_houston_wall_following");
    ros::NodeHandle n;

    WallFollow wf;
    std::cout << wf.run() << std::endl;

    return 0;
}