/*
 * Created By: YOUR NAME HERE
 * Created On: September 22, 2016
 * Description: This node is responsible for passing received twist messages over
 *              serial to the arduino controlling the robot
 *
 */

#ifndef DRIVERS_STEERING_DRIVER_H
#define DRIVERS_STEERING_DRIVER_H

#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <SnowBotSerial.hpp>

class SteeringDriver {
public:
    SteeringDriver(int argc, char **argv, std::string node_name, std::string portName,
        SerialStreamBuf::BaudRateEnum baud_rate);
private:
    void commandCallBack(const geometry_msgs::Twist::ConstPtr& twist_msg);

    ros::Subscriber command_subscriber;

    SnowBotSerial snowBot;
};
#endif //DRIVERS_STEERING_DRIVER_H
