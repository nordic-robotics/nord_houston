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

    std::vector<point<2>> path_U({
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

    std::vector<point<2>> path_maze({
       // point<2>(0.15, 0.23),
        point<2>(2.22, 0.23),
        point<2>(2.22, 0.68),
        point<2>(1.83, 0.68),
        point<2>(1.83, 1.06),
        point<2>(2.22, 1.06),
        point<2>(2.22, 1.53),
        point<2>(1.83, 1.53),
        point<2>(1.83, 2.22),
        point<2>(1.01, 2.22),
        point<2>(0.17, 2.1),
        point<2>(0.17, 1.5),
        point<2>(0.17, 1.04),
        point<2>(0.5, 1.04),
        point<2>(0.99, 1.04),
        point<2>(0.99, 1.5),
        point<2>(0.99, 2.11),
        point<2>(0.17, 2.1)
    });

    mission_one_behaviour<MissionOne> b(n, path_maze);

    std::this_thread::sleep_for(2s);
    b.behave();


    return 0;
}
