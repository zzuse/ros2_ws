#include "motion_control_system/motion_control_interface.hpp"
#include <pluginlib/class_loader.hpp>
#include <iostream>

using namespace motion_control_system;

int main(int argc, char** argv)
{
    if(argc!=2) return 0;

    std::string controller_name = argv[1];

    pluginlib::ClassLoader<motion_control_system::MotionController> loader("motion_control_system", "motion_control_system::MotionController");

    try
    {
        std::shared_ptr<motion_control_system::MotionController> controller = loader.createSharedInstance(controller_name);
        controller->start();
        controller->stop();
    }
    catch (const pluginlib::PluginlibException& ex)
    {
        std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
    }

    return 0;
}
