#pragma once

#include "behaviour.hpp"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/Vector2.h"
#include "nord_messages/Classification.h"
#include "nord_messages/ClassificationArray.h"
#include "nord_messages/ClassificationSrv.h"
#include "std_msgs/String.h"
#include "point.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

/*namespace
{
    float distance_between(const nord_messages::Classification& c0,
                           const nord_messages::Classification& c1)
    {
        return std::hypot(c0.loc.x - c1.loc.x, c0.loc.y - c1.loc.y);
    }
}*/

template<class BT>
class mission_one_behaviour : public behaviour<BT, mission_one_behaviour<BT>>
{
    using base = behaviour<BT, mission_one_behaviour>;
    using Classification = nord_messages::Classification;
public:
    mission_one_behaviour(ros::NodeHandle& n) : n(n), base(*this, n)
    {
        this->template subscribe<nord_messages::PoseEstimate>("/nord/estimation/gaussian", 1,
            [&](const nord_messages::PoseEstimate::ConstPtr& msg) {
                base::tree.pose = *msg;
            });
        this->template subscribe<nord_messages::PoseEstimate>("/nord/estimation/gaussian", 1,
            [&](const nord_messages::PoseEstimate::ConstPtr& msg) {
                base::tree.unknown.emplace_back(msg->x.mean, msg->y.mean);
            });
        this->template advertise<nord_messages::Vector2>("/nord/control/point", 1);
        this->template advertise<std_msgs::String>("/espeak/string", 10);

        tick = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
            base::tree.time_left -= 1;
        });

        base::tree.time_left = 60 * 5;
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

    float distance_to(const point<2>& target)
    {
        return (target - point<2>(base::tree.pose.x.mean, base::tree.pose.y.mean)).length();
    }

    bool align(const point<2>& target)
    {
        std::cout << "align" << std::endl;
        // TODO: make sure this always picks a new point to prevent infinite loops
        // maybe consult map?
        point<2> current(base::tree.pose.x.mean, base::tree.pose.y.mean);
        point<2> dir = target - current;
        point<2> aligned;
        if (dir.length() > base::tree.align_range)
        {
            aligned = target + (-dir).normalized();
            return go_to(aligned);
        }
        return true;
    }

    bool classify()
    {
        std::cout << "classify" << std::endl;
        ros::ServiceClient client = n.serviceClient<nord_messages::ClassificationSrv>(
            "/nord/vision/classification_service", true);
        nord_messages::ClassificationSrv srv;
        if (!client.call(srv))
        {
            std::cout << "\tcall failed!" << std::endl;
            return false;
        }
        std::vector<Classification> new_classified = srv.response.classifications.data;
        if (new_classified.size() == 0)
        {
            std::cout << "\tclassification found nothing!" << std::endl;
            return false;
        }

        std::transform(new_classified.begin(), new_classified.end(),
                       std::back_inserter(base::tree.classified),
            [&](const Classification& c) {
                base::tree.unknown.erase(std::remove_if(base::tree.unknown.begin(),
                                                        base::tree.unknown.end(),
                    [&](const point<2>& p) {
                        return (p - point<2>(c.loc.x, c.loc.y)).length() < 0.05;
                    }));
                return c;
            });

        return true;
    }

    bool announce()
    {
        std::cout << "announce" << std::endl;
        std::transform(base::tree.classified.begin(), base::tree.classified.end(),
                       std::back_inserter(base::tree.announced),
                       [&](const Classification& c) {
                           std_msgs::String msg = c.name;
                           std::cout << "\t" << msg.data << std::endl;
                           base::publish("/espeak/string", msg);
                           return c;
                       });
        base::tree.classified.clear();
        return true;
    }

    bool go_to(const point<2>& p)
    {
        std::cout << "go_to (" << p.x() << ", " << p.y() << ")" << std::endl;
        nord_messages::Vector2 msg;
        msg.x = p.x();
        msg.y = p.y();

        return base::publish("/nord/control/point", msg, true);
    }

private:
    ros::Timer tick;
    ros::NodeHandle& n;
};
