#pragma once

#include "ros/ros.h"
#include "nord_messages/Command.h"
#include "nord_messages/IRSensors.h"
#include "ras_arduino_msgs/Encoders.h"
#include "std_msgs/Int32.h"
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
            try
            {
                ros::spinOnce();
                tree.run(derived);
            }
            catch (bool abort)
            {
                for (size_t i = 0; i < 0; i++)
                {
                    r.sleep();
                }
                std::cout << "restarting" << std::endl;
            }
            r.sleep();
        }
    }

protected:
    behaviour(Derived& derived, ros::NodeHandle& n)
        : subs({
            n.subscribe<std_msgs::Int32>("/nord/houston/mission_result", 1,
                [&, this](const std_msgs::Int32::ConstPtr seq) {
                    if (seq->data == sequence_number)
                    {
                        has_result = true;
                        result = true;
                        std::cout << "SUCCESS on order #" << seq->data << std::endl;
                    }
                    else if (seq->data > sequence_number)
                    {
                        std::cout << "OOPS received #" << seq->data << std::endl;
                        exit(1);
                    }
                    else
                    {
                        std::cout << "OLD order #" << seq->data << std::endl;
                    }
                }),
            n.subscribe<std_msgs::Empty>("/nord/houston/mission_abort", 1,
                [&, this](const std_msgs::Empty::ConstPtr e) {
                    abort();
                })
          }),
          derived(derived), n(n), r(10)
    { };
    virtual ~behaviour() { }

    template<class MSG>
    bool publish(const std::string& topic, const MSG& msg, bool await_result = false,
                 bool print = true)
    {
        has_result = false;
        if (print)
        {
            std::cout << "sending instructions to " << topic;
            if (await_result)
                std::cout << ", awaiting result";
            std::cout << std::endl;
        }

        publishers[topic].publish(msg);

        if (await_result)
        {

            while (!has_result)
            {
                if (should_abort)
                {
                    should_abort = false;
                    std::cout << "abort!" << std::endl;
                    throw true;
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

    virtual void abort()
    {
        should_abort = true;
    }

    int32_t get_sequence_number()
    {
        return ++sequence_number;
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
    int32_t sequence_number = 1;
};
