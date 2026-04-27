#include <iostream>
#include "motion_control_system/spin_motion_controller.hpp"

namespace motion_control_system
{
    void SpinMotionController::start()
    {
        if (!is_spinning)
        {
            std::cout << "SpinMotionController: Starting spin motion." << std::endl;
            is_spinning = true;
        }
        else
        {
            std::cout << "SpinMotionController: Already spinning." << std::endl;
        }
    }

    void SpinMotionController::stop()
    {
        if (is_spinning)
        {
            std::cout << "SpinMotionController: Stopping spin motion." << std::endl;
            is_spinning = false;
        }
        else
        {
            std::cout << "SpinMotionController: Not currently spinning." << std::endl;
        }
    }
    
} // namespace motion_control_system

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motion_control_system::SpinMotionController, motion_control_system::MotionController)