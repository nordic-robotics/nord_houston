#pragma once

#include "behaviour.hpp"
#include "nord_messages/PoseEstimate.h"
#include "nord_messages/Vector2.h"
#include "nord_messages/NextNode.h"
#include "nord_messages/ClassificationSrv.h"
#include "nord_messages/EvidenceSrv.h"
#include "nord_messages/ObjectArray.h"
#include "nord_messages/MotorTwist.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "point.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

// draws the map walls
inline visualization_msgs::Marker create_path_message(const std::vector<point<2>>& path)
{
    visualization_msgs::Marker line_list;
    line_list.id = 204;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.color.a = line_list.color.r = 1.0;
    line_list.color.g = 0.8;
    line_list.color.b = 0.1;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "houston_path";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.lifetime = ros::Duration();
    line_list.scale.x = 0.01;

    for (size_t i = 0; i < path.size() - 1; i++)
    {
        geometry_msgs::Point p0, p1;
        p0.x = path[i].x();
        p0.y = path[i].y();
        p1.x = path[i + 1].x();
        p1.y = path[i + 1].y();
        p0.z = p1.z = 0;
        line_list.points.push_back(p0);
        line_list.points.push_back(p1);
    }

    return line_list;
}


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
                    if (std::find(found_history.begin(),
                                  found_history.end(),
                                  static_cast<size_t>(d.id)) == found_history.end())
                    {
                        tree.unknown.push_back(std::make_pair(d.id, point<2>(d.x, d.y)));
                        found_history.push_back(d.id);
                        should_abort = true;
                    }
                }
                if (should_abort)
                {
                    std::cout << "found new object" << std::endl;
                    std::cout << "num unknown: " << tree.unknown.size() << std::endl;
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
        this->template advertise<nord_messages::MotorTwist>("/nord/motor_controller/twist", 10);
        this->template advertise<std_msgs::String>("/espeak/string", 10);
        this->template advertise<visualization_msgs::Marker>("/nord/map", 10);

        tick = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent& e) {
            tree.time_left -= 1;
            this->template publish("/nord/map", create_path_message(tree.path), false);
        });

        tree.path = path;
    }

    float distance_to_exit_heuristic()
    {
        return 0;
    }

    bool exit()
    {
        nord_messages::MotorTwist msg;
        msg.velocity = 0;
        msg.angular_vel = 0;
        this->template publish("/nord/motor_controller/twist", msg);
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
        return go_to(target, 3);
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
        srv2.request.data.id = id;
        srv2.request.data.x = obj_location.x();
        srv2.request.data.y = obj_location.y();
        srv2.request.data.objectId.data = new_class;
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
    std::vector<size_t> found_history;
};
