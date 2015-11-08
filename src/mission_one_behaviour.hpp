#pragma once

#include "behaviour.hpp"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/Vector2.h"
#include "std_msgs/String.h"
#include <iostream>
#include <cmath>
#include <algorithm>

class point
{
public:
    point() { }
    point(float x, float y) : x(x), y(y) { }

    float distance_to(const point& p) const
    {
        return std::sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
    }

    float x = 0;
    float y = 0;
};

template<class BT>
class mission_one_behaviour : public behaviour<BT, mission_one_behaviour<BT>>
{
    using base = behaviour<BT, mission_one_behaviour>;
public:
    mission_one_behaviour(ros::NodeHandle& n) : base(*this, n)
    {
        this->template subscribe<nord_messages::PoseEstimate>("/nord/estimation/gaussian", 1,
            [&](const nord_messages::PoseEstimate::ConstPtr& msg) {
                base::tree.pose = *msg;
            });
        this->template advertise<nord_messages::Vector2>("/nord/control/point", 1);
        this->template advertise<std_msgs::String>("/espeak/string", 10);

        tick = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
            base::tree.time_left -= 1;
        });

        base::tree.time_left = 60 * 5;

        base::tree.unknown.push_back(point(1, 0));
        base::tree.path.push_back(point(1, 0));
        base::tree.pose.x.mean = base::tree.pose.y.mean = 0;
    }

    float distance_to_exit_heuristic()
    {
        return 0;
    }

    bool exit()
    {
        std::cout << "exit" << std::endl;
        return true;
    }

    float distance_to(const point& target)
    {
        return point(base::tree.pose.x.mean, base::tree.pose.y.mean).distance_to(target);
    }

    bool align(const point& target)
    {
        std::cout << "align" << std::endl;
        // TODO: make sure this always picks a new point to prevent infinite loops
        // maybe consult map?
        return true;
    }

    bool classify()
    {
        std::cout << "classify" << std::endl;
        // TODO: call classify service
        std::vector<std::pair<point, std::string>> new_classified;
        if (new_classified.size() == 0)
            return false;

        std::transform(new_classified.begin(), new_classified.end(),
                       std::back_inserter(base::tree.classified),
            [&](const std::pair<point, std::string>& pair) {
                base::tree.unknown.erase(std::remove_if(base::tree.unknown.begin(),
                                                        base::tree.unknown.end(),
                    [&](const point& p1) {
                        return pair.first.distance_to(p1) < 0.05;
                    }));
                return pair;
            });

        return true;
    }

    bool announce()
    {
        std::cout << "announce" << std::endl;
        std::transform(base::tree.classified.begin(), base::tree.classified.end(),
                       std::back_inserter(base::tree.announced),
                       [&](const std::pair<point, std::string>& pair) {
                           std_msgs::String msg;
                           msg.data = pair.second;
                           std::cout << "\t" << msg.data << std::endl;
                           base::publish("/espeak/string", msg);
                           return pair;
                       });
        base::tree.classified.clear();
        return true;
    }

    bool go_to(const point& p)
    {
        std::cout << "go_to (" << p.x << ", " << p.y << ")" << std::endl;
        nord_messages::Vector2 msg;
        msg.x = p.x;
        msg.y = p.y;

        // temp
        base::tree.pose.x.mean = p.x;
        base::tree.pose.y.mean = p.y;
        return true;
        
        return base::publish("/nord/control/point", msg, true);
    }

private:
    ros::Timer tick;
};
