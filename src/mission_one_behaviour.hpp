#pragma once

#include "behaviour.hpp"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/Vector2.h"
#include "nord_messages/NextNode.h"
#include "nord_messages/ClassificationSrv.h"
#include "nord_messages/EvidenceSrv.h"
#include "nord_messages/ObjectArray.h"
#include "std_msgs/String.h"
#include "point.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

template<class BT>
class mission_one_behaviour : public behaviour<BT, mission_one_behaviour<BT>>
{
    using base = behaviour<BT, mission_one_behaviour>;
    using base::tree;
public:
    mission_one_behaviour(ros::NodeHandle& n, const std::vector<point<2>> path)
        : n(n), base(*this, n)
    {
        this->template subscribe<nord_messages::PoseEstimate>("/nord/estimation/pose_estimation", 9,
            [&](const nord_messages::PoseEstimate::ConstPtr& msg) {
                tree.pose = *msg;
                sort_unknown();
            });
        this->template subscribe<nord_messages::ObjectArray>("/nord/estimation/objects", 9,
            [&](const nord_messages::ObjectArray::ConstPtr& msg) {
                bool should_abort = false;
                for (auto& d : msg->data)
                {
                    if (std::find_if(tree.unknown.begin(),
                                  tree.unknown.end(),
                                  [&](const std::pair<size_t, point<2>>& thing) {
                                    return thing.first == d.id;
                                  }) == tree.unknown.end())
                    {
                        tree.unknown.push_back(std::make_pair(d.id, point<2>(d.x, d.y)));
                        should_abort = true;
                    }
                }
                if (should_abort)
                {
                    std::cout << "found new object" << std::endl;
                    this->template abort();
                }
            });
        this->template subscribe<nord_messages::Vector2>("/imu/bump", 9,
            [&](const nord_messages::Vector2::ConstPtr& msg) {
                std::cout << "BUMP!" << std::endl;
                go_to(point<2>(tree.pose.x.mean, tree.pose.y.mean)
                    + point<2>(std::cos(tree.pose.theta.mean),
                               std::sin(tree.pose.theta.mean)) * 0.05,
                      2);
                this->template abort();
            });
        this->template advertise<nord_messages::NextNode>("/nord/control/point", 10);
        this->template advertise<std_msgs::String>("/espeak/string", 10);

        tick = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
            tree.time_left -= 1;
        });

        tree.time_left = 60 * 5;

        tree.path = path;
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
        return (target - point<2>(tree.pose.x.mean, tree.pose.y.mean)).length();
    }

    void sort_unknown()
    {
        std::sort(tree.unknown.begin(), tree.unknown.end(),
            [&](const std::pair<size_t, point<2>>& a,
               const std::pair<size_t, point<2>>& b) {
                   return distance_to(a.second) < distance_to(b.second);
            });
        //if (tree.unknown.size() > 0)
        //    std::cout << "closest is " << distance_to(tree.unknown.front().second) << " away" << std::endl;
    }

    bool align(const point<2>& target)
    {
        std::cout << "align" << std::endl;
        // TODO: make sure this always picks a new point to prevent infinite loops
        // maybe consult map?
        point<2> current(tree.pose.x.mean, tree.pose.y.mean);
        //point<2> dir = target - current;
        point<2> aligned;
        return true;//go_to(target, 3);;
    }

    bool classify(size_t id)
    {
        std::cout << "classify" << std::endl;
        ros::ServiceClient client = n.serviceClient<nord_messages::ClassificationSrv>(
            "/nord/vision/classification_service", false);
        nord_messages::ClassificationSrv srv;
        srv.request.id = id;
        if (!client.call(srv))
        {
            std::cout << "call failed!" << std::endl;
            return false;
        }
        std::string new_class = srv.response.classification.data;

        if (new_class == "")
        {
            std::cout << "classification failed!" << std::endl;
            return false;
        }

        point<2> obj_location(-1, -1);
        for (size_t i = 0; i < tree.unknown.size(); i++)
        {
            if (tree.unknown[i].first == id)
            {
                obj_location = tree.unknown[i].second;
                tree.unknown.erase(tree.unknown.begin() + i);
                i--;
            }
        }

        tree.classified.push_back(new_class);

        std::cout << "snapping evidence" << std::endl;
        ros::ServiceClient client2 = n.serviceClient<nord_messages::EvidenceSrv>(
            "/nord/evidence_service", false);
        nord_messages::EvidenceSrv srv2;
        // fill in
        srv2.request.id = id;
        srv2.request.position.x = obj_location.x();
        srv2.request.position.y = obj_location.y();
        srv2.request.classification.data = new_class;
        if (!client2.call(srv2))
        {
            std::cout << "evidence call failed!" << std::endl;
        }

        return true;
    }

    bool announce()
    {
        std::cout << "announce" << std::endl;
        std::transform(tree.classified.begin(), tree.classified.end(),
                       std::back_inserter(tree.announced),
                       [&](const std::string& name) {
                           std_msgs::String msg;
                           msg.data = name;
                           std::cout << "\t" << msg.data << std::endl;
                           this->template publish("/espeak/string", msg);
                           return name;
                       });
        tree.classified.clear();
        return true;
    }

    bool go_to(const point<2>& p, int move = 1)
    {
        std::cout << "go_to (" << p.x() << ", " << p.y() << ")" << move << std::endl;
        nord_messages::NextNode msg;
        msg.x = p.x();
        msg.y = p.y();
        msg.move = move;

        return this->template publish("/nord/control/point", msg, true);
    }

private:
    ros::Timer tick;
    ros::NodeHandle& n;
    std::vector<size_t> classified_history;
};
