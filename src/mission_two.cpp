#include "ros/ros.h"
#include "ros/package.h"
#include <cstdint>
#include <iostream>
#include <vector>
#include <fstream>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "behaviour.hpp"
#include "mission_two_behaviour.hpp"
#include "seqsel/MissionTwo.hpp"
#include "common.hpp"
#include "dijkstra/map.hpp"
#include <thread>
#include <cassert>

std::vector<std::pair<std::string, point<2>>> load_objects(std::string filename)
{
    std::ifstream file(filename);
    std::string l;
    std::vector<std::pair<std::string, point<2>>> objects;
    while (std::getline(file, l))
    {
        std::istringstream iss(l);
        float x, y;
        iss >> x >> y;
        std::string name;
        std::getline(iss, name);
        name.erase(name.begin());
        objects.emplace_back(name, point<2>(x, y));
        std::cout << x << " " << y << " " << name << std::endl;
    }
    return objects;
}

int main(int argc, char** argv)
{
    using namespace std::literals;
    ros::init(argc, argv, "nord_houston_mission_two");
    ros::NodeHandle n;

    assert(argc == 3);
    point<2> exit_point(std::stod(argv[1]), std::stod(argv[2]));

    auto objects = load_objects(ros::package::getPath("nord_vision")+"/data/objects.txt");

    mission_two_behaviour<MissionTwo> b(n, exit_point, objects);

    std::this_thread::sleep_for(2s);
    b.behave();


    return 0;
}
