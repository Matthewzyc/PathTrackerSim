#include "tracker.hpp"

#include <ros/ros.h>
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>

void mySigintHandler(int signum)
{
    std::cout << "shuting down..." << std::endl;
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker_node", ros::init_options::NoSigintHandler);
    Tracker tracker;
    signal(SIGINT, mySigintHandler);
    while (ros::ok())
    {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}