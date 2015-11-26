#include "ros/ros.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "behaviour.hpp"
#include "mission_one_behaviour.hpp"
#include "seqsel/MissionOne.hpp"
#include <thread>

int main(int argc, char** argv)
{
    using namespace std::literals;
    ros::init(argc, argv, "nord_houston_mission_one");
    ros::NodeHandle n;

    std::vector<point<2>> path({
        point<2>(0.17, 2.1),
        point<2>(0.17, 1.5),
        point<2>(0.17, 1.04),
        point<2>(0.5, 1.04),
        point<2>(0.99, 1.04),
        point<2>(0.99, 1.5),
        point<2>(0.99, 2.11),
        point<2>(0.17, 2.1),
        point<2>(0.17, 1.5),
        point<2>(0.17, 1.04),
        point<2>(0.5, 1.04),
        point<2>(0.99, 1.04),
        point<2>(0.99, 1.5),
        point<2>(0.99, 2.11),
        point<2>(0.17, 2.1),
        point<2>(0.17, 1.5),
        point<2>(0.17, 1.04),
        point<2>(0.5, 1.04),
        point<2>(0.99, 1.04),
        point<2>(0.99, 1.5),
        point<2>(0.99, 2.11)
    });

    mission_one_behaviour<MissionOne> b(n, path);

    std::this_thread::sleep_for(2s);
    b.behave();


    return 0;
}
