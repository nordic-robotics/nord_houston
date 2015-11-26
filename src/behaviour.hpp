#pragma once

#include "ros/ros.h"
#include "nord_messages/Command.h"
#include "nord_messages/IRSensors.h"
#include "ras_arduino_msgs/Encoders.h"
#include <vector>
#include <utility>
#include <string>
#include <functional>
#include <memory>
#include <cstdint>
#include <unordered_map>

template<class BT, class Derived>
class behaviour
{
public:
    void behave()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            tree.run(derived);
            r.sleep();
        }
    }

protected:
    behaviour(Derived& derived, ros::NodeHandle& n)
        : subs({
            n.subscribe<std_msgs::Bool>("/nord/houston/mission_result", 10,
                [&](const std_msgs::Bool::ConstPtr b) {
                    has_result = true;
                    result = b->data;
                }),
            n.subscribe<std_msgs::Empty>("/nord/houston/mission_abort", 10,
                [&](const std_msgs::Empty::ConstPtr e) {
                    should_abort = true;
                })
          }),
          derived(derived), n(n), r(10)
    { };
    virtual ~behaviour() { }

    template<class MSG>
    bool publish(const std::string& topic, const MSG& msg, bool await_result = false)
    {
        has_result = false;
        std::cout << "sending instructions to " << topic;
        if (await_result)
            std::cout << ", awaiting result";
        std::cout << std::endl;

        publishers[topic].publish(msg);

        if (await_result)
        {
            while (!has_result && ros::ok())
            {
                if (should_abort)
                {
                    should_abort = false;
                    std::cout << "abort!" << std::endl;
                    return tree.run(derived);
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

    template<class MSG>
    void advertise(const std::string& topic, uint32_t queue_size)
    {
        publishers[topic] = n.advertise<MSG>(topic, queue_size);
    }

    template<class MSG, class Callback>
    void subscribe(const std::string& topic, uint32_t queue_size, Callback c)
    {
        subs.push_back(n.subscribe<MSG>(topic, queue_size, c));
    }

    void abort()
    {
        should_abort = true;
    }

    BT tree;

private:
    Derived& derived;
    std::unordered_map<std::string, ros::Publisher> publishers;
    std::vector<ros::Subscriber> subs;
    ros::NodeHandle& n;
    bool has_result = false;
    bool result;
    bool should_abort = false;
    ros::Rate r;
};