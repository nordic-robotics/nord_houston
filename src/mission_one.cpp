#include "ros/ros.h"
#include "ros/package.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include <fstream>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "behaviour.hpp"
#include "mission_one_behaviour.hpp"
#include "seqsel/MissionOne.hpp"
#include <thread>

std::vector<point<2>> read_path(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<point<2>> path;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        float x, y;
        iss >> x >> y;
        path.emplace_back(x, y);
    }
    return path;
}

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

std::vector<point<2>> path_fuck_you({
       // point<2>(0.15, 0.23),
        point<2>(2.22, 0.23),
        point<2>(2.22, 0.68),
point<2>(2.22, 0.23),
point<2>(0.20, 0.21),
point<2>(2.22, 0.23),
point<2>(2.22, 0.68),
        point<2>(1.83, 0.68),
        point<2>(1.83, 1.1),
        point<2>(2.22, 1.1),
        point<2>(2.22, 1.53),
        point<2>(1.83, 1.53),
        point<2>(1.83, 2.25),
        point<2>(1.01, 2.25),
        point<2>(0.17, 2.1),
        point<2>(0.17, 1.5),
        point<2>(0.17, 1.04),
        point<2>(0.5, 1.04),
        point<2>(0.99, 1.04),
        point<2>(0.99, 1.5),
        point<2>(0.99, 2.11),
        point<2>(0.17, 2.1)
    });

    std::vector<point<2>> path_planned({
       // point<2>(0.15, 0.23),
        point<2>(0.5, 0.25),
        point<2>(1.1, 0.15),
        point<2>(1.6, 0.15),
        point<2>(2.09, 0.15),
        point<2>(2.22, 0.72),
        point<2>(2.03, 0.58),
        point<2>(1.75, 0.75),
        point<2>(2.25, 1.25),
        point<2>(1.91, 1.66),
        point<2>(1.75, 2.25),
        point<2>(1.25, 2.25),
        point<2>(1.0, 1.75),
        point<2>(1.0, 1.25),
        point<2>(0.5, 0.75),
        point<2>(0.15, 1.35),
        point<2>(0.19, 1.75),
        point<2>(0.75, 2.0),
	point<2>(1.19, 1.69),
        point<2>(1.0, 1.25),
        point<2>(1.44, 0.69),
        point<2>(1.13, 1.13),
        point<2>(0.75, 1.0),
        point<2>(0.51, 1.47),
        point<2>(0.75, 1.0),
        point<2>(1.25, 1.5),
        point<2>(0.75, 2.0),
        point<2>(1.0, 2.0),
point<2>(1.5, 2.25),
        point<2>(2.0, 2.25),
        point<2>(1.91, 1.66),
        point<2>(1.75, 1.5),
        point<2>(1.75, 1.75),
        point<2>(2.25, 1.25)
    });


    std::vector<point<2>> path_custom({
       // point<2>(0.2, 0.2),
        point<2>(2.18, 0.20),
        point<2>(2.18, 0.56),
        point<2>(0.20, 0.56),
        point<2>(0.20, 1.62),
        point<2>(1.20, 1.62),
        point<2>(0.50, 1.62),
        point<2>(0.50, 2.20),
        point<2>(2.62, 2.20),
        point<2>(2.62, 0.20),
        point<2>(4.60, 0.20),
        point<2>(4.60, 2.20),
        point<2>(2.62, 0.50),
        point<2>(0.20, 0.50),
        point<2>(2.62, 0.20),
        point<2>(2.62, 2.20),
        point<2>(0.50, 2.20),
        point<2>(0.50, 1.62),
        point<2>(1.20, 1.62),
        point<2>(0.20, 1.62),
        point<2>(0.20, 0.56),
        point<2>(2.87, 0.56),
        point<2>(2.87, 0.20),
        point<2>(0.2, 0.2)
    });

    auto actual_path = read_path(ros::package::getPath("nord_houston")+"/data/plan.txt");

    mission_one_behaviour<MissionOne> b(n, path_fuck_you);

    std::this_thread::sleep_for(2s);
    b.behave();


    return 0;
}
