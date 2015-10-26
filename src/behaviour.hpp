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

enum class action : uint32_t
{
    none = 0,
    forwards_speed,
    backwards_distance,
    turn_left,
    turn_right
};

template<class BT>
class behaviour
{
    using Command = nord_messages::Command;
public:
    behaviour(ros::NodeHandle& n)
        : subs({
            n.subscribe<std_msgs::Bool>("/nord/houston/mission_result", 10,
                [&](const std_msgs::Bool::ConstPtr b) {
                    has_result = true;
                    result = b->data;
                }),
            n.subscribe<std_msgs::Empty>("/nord/houston/mission_abort", 10,
                [&](const std_msgs::Empty::ConstPtr e) {
                    should_abort = true;
                }),
            n.subscribe<ras_arduino_msgs::Encoders>("/arduino/encoders", 10,
                [&](const ras_arduino_msgs::Encoders::ConstPtr& e) {
                    tree.delta_encoder_left = e->delta_encoder1;
                    tree.delta_encoder_right = e->delta_encoder2;
                }),
            n.subscribe<nord_messages::IRSensors>("/nord/sensors/ir", 10,
                [&](const nord_messages::IRSensors::ConstPtr& ir) {
                    tree.ir_front = ir->front;
                    tree.ir_back = ir->back;
                    tree.ir_left_front = ir->left_front;
                    tree.ir_left_back = ir->left_back;
                    tree.ir_right_front = ir->right_front;
                    tree.ir_right_back = ir->right_back;
                })
        }),
          mission_control(n.advertise<Command>("/nord/houston/mission_control", 10))
    { };

    void behave()
    {
        ros::Rate r(10);
        while (ros::ok())
        {
            ros::spinOnce();
            tree.run(*this);
            r.sleep();
        }
    }

    bool publish(action a, const std::vector<float>& values, bool await_result = false)
    {
        std::cout << "running " << uint32_t(a);
        for (auto value : values)
            std::cout << " " << value;
        if (await_result)
            std::cout << ", waiting for reply";
        std::cout << std::endl;

        has_result = false;

        Command msg;
        msg.action = uint32_t(a);
        msg.values = values;
        mission_control.publish(msg);

        if (await_result)
        {
            static ros::Rate r(10);
            while (!has_result && ros::ok())
            {
                if (should_abort)
                {
                    should_abort = false;
                    std::cout << "abort!" << std::endl;
                    return tree.run(*this);
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
    bool publish(action a, float value, bool await_result = false)
    {
        return publish(a, std::vector<float>({value}), await_result);
    }

private:
    BT tree;
    std::vector<ros::Subscriber> subs;
    ros::Publisher mission_control;
    bool has_result = false;
    bool result;
    bool should_abort = false;
};