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
#include "std_msgs/Int64.h"
#include "visualization_msgs/Marker.h"
#include "dijkstra/map.hpp"
#include "common.hpp"
#include "point.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>

template<class BT>
class mission_two_behaviour : public behaviour<BT, mission_two_behaviour<BT>>
{
    using base = behaviour<BT, mission_two_behaviour>;
    using base::tree;
public:
    mission_two_behaviour(ros::NodeHandle& n, dijkstra::map* maze, const point<2> exit_point,
                          const std::vector<std::pair<std::string, point<2>>>& objects)
        : n(n), maze(maze), base(*this, n)
    {
        this->template subscribe<nord_messages::PoseEstimate>("/nord/estimation/pose_estimation", 9,
            [&](const nord_messages::PoseEstimate::ConstPtr& msg) {
                tree.pose = *msg;
                sort_unannounced();
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
        this->template advertise<visualization_msgs::Marker>("/nord/map", 10);
        this->template advertise<nord_messages::MotorTwist>("/nord/motor_controller/twist", 10);
        this->template advertise<std_msgs::Int64>("/nord/estimation/set_num_particles", 10);

        tick = n.createTimer(ros::Duration(1), [&, objects](const ros::TimerEvent& e) {
            tree.time_left -= 1;
            this->template publish("/nord/map", create_path_message(tree.path));
            this->template publish("/nord/map", create_objects_message(objects));
        });

        tree.exit_point = exit_point;
        tree.pose.x.stddev = tree.pose.y.stddev = tree.pose.theta.stddev = 10;
        tree.unannounced = objects;
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

    void sort_unannounced()
    {
        std::sort(tree.unannounced.begin(), tree.unannounced.end(),
            [&](const std::pair<std::string, point<2>>& a,
                const std::pair<std::string, point<2>>& b) {
                   return distance_to(a.second) < distance_to(b.second);
            });
    }

    bool announce(const std::string& name)
    {
        std::cout << "announce" << std::endl;
        std_msgs::String msg;
        msg.data = name;
        std::cout << "\t" << msg.data << std::endl;
        this->template publish("/espeak/string", msg);
        return true;
    }

    bool record_evidence(std::string& name)
    {
        ros::ServiceClient client = n.serviceClient<nord_messages::EvidenceSrv>(
            "/nord/evidence_service", false);
        nord_messages::EvidenceSrv srv;
        nord_messages::Object o;
        srv.request.data.objectId.data = name;
        if (!client.call(srv))
        {
            std::cout << "evidence call failed!" << std::endl;
        }
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

    std::vector<point<2>> plan_path(const point<2>& current, const point<2>& target)
    {
        auto path = maze->find(dijkstra::point(current.x(), current.y()),
                               dijkstra::point(target.x(), target.y()));
        std::vector<point<2>> result;
        std::transform(path.begin(), path.end(), std::back_inserter(result),
            [](dijkstra::point const* p) {
                std::cout << "plan: " << p->x << " " << p->y << std::endl;
                return point<2>(p->x, p->y);
        });
        return result;
    }

    bool spin(double time, double angular_velocity)
    {
        nord_messages::MotorTwist msg;
        msg.velocity = 0;
        msg.angular_vel = angular_velocity;
        this->template publish("/nord/motor_controller/twist", msg);
        auto duration = std::chrono::duration<double, std::ratio<1, 1>>(time);
        ros::spinOnce();
        std::this_thread::sleep_for(duration);

        return true;
    }

    bool set_num_particles(unsigned int n)
    {
        std_msgs::Int64 msg;
        msg.data = n;
        this->template publish("/nord/estimation/set_num_particles", msg);

        return true;
    }

private:
    ros::Timer tick;
    ros::NodeHandle& n;
    std::vector<size_t> found_history;
    dijkstra::map* maze;
};
