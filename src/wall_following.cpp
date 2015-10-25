#include "ros/ros.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "nord_messages/Command.h"

enum class action : uint32_t
{
    none = 0,
    forwards_speed,
    backwards_distance,
    turn_left,
    turn_right
};

bool publish(action a, const std::vector<float>& values, bool await_result = false);
bool publish(action a, float, bool await_result = false);

#include "seqsel/WallFollow.hpp"

WallFollow wf;
ros::Publisher mission_control;
bool has_result = false;
bool result;
bool should_abort = false;

bool publish(action a, const std::vector<float>& values, bool await_result)
{
    std::cout << "running " << uint32_t(a);
    for (auto value : values)
        std::cout << " " << value;
    if (await_result)
        std::cout << ", waiting for reply";
    std::cout << std::endl;

    has_result = false;

    nord_messages::Command msg;
    msg.action = uint32_t(a);
    msg.values = values;
    mission_control.publish(msg);

    if (await_result)
    {
        static ros::Rate r(10);
        while (!has_result && ros::ok())
        {
            if (should_abort)
            {
                should_abort = false;
                std::cout << "abort!" << std::endl;
                return wf.run();
            }

            ros::spinOnce();
            r.sleep();
        }
        std::cout << (result ? "success!" : "failed..") << std::endl;
        return result;
    }
    else
        return true;
}
bool publish(action a, float value, bool await_result)
{
    return publish(a, std::vector<float>({value}), await_result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nord_houston_wall_following");
    ros::NodeHandle n;

    mission_control = n.advertise<nord_messages::Command>("/nord/houston/mission_control", 10);
    ros::Subscriber mission_result =
        n.subscribe<std_msgs::Bool>("/nord/houston/mission_result", 10,
                                    [&](const std_msgs::Bool::ConstPtr b) {
                                        has_result = true;
                                        result = b->data;
                                    });
    ros::Subscriber mission_abort =
        n.subscribe<std_msgs::Empty>("/nord/houston/mission_abort", 10,
                                     [&](const std_msgs::Empty::ConstPtr e) {
                                         should_abort = true;
                                     });

    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        wf.run();
        r.sleep();
    }

    return 0;
}
