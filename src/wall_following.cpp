#include "ros/ros.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include "nord_messages/Command.h"

enum class action : uint32_t
{
    none = 0,
    forwards_speed,
    backwards_distance,
    turn_left,
    turn_right
};

bool publish(action, const std::vector<float>&);
bool publish(action, float);

#include "seqsel/WallFollow.hpp"

ros::Publisher mission_control;

bool publish(action a, const std::vector<float>& values)
{
    nord_messages::Command msg;
    msg.action = uint32_t(a);
    msg.values = values;
    mission_control.publish(msg);
    return true;
}
bool publish(action a, float value)
{
    return publish(a, std::vector<float>({value}));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_houston_wall_following");
    ros::NodeHandle n;

    WallFollow wf;

    mission_control = n.advertise<nord_messages::Command>("/nord/houston/mission_control", 10);

    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        wf.run();
        r.sleep();
    }

    return 0;
}
