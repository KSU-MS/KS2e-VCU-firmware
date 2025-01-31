#include "tc_system.h"
// #include <cstring>
void torque_control_system::toggleController(unsigned long systime)
{
    int current_controller = static_cast<int>(tcType);
    int next_controller = current_controller + 1;
    while (true)
    {
        if (next_controller >= static_cast<int>(torque_control_types_e::TC_NUM_CONTROLLERS))
        {
            next_controller = 0;
        }
        if (enabled_tc_map.find(static_cast<torque_control_types_e>(next_controller)) != enabled_tc_map.end())
        {
            break; // Found an enabled controller
        }
        next_controller++;
    }

    this->setActiveSystem(static_cast<torque_control_types_e>(next_controller));

    // init new system
    this->getController()->inittorque_controller(systime);
}

void torque_control_system::printControllerType()
{
    torque_controller* controller = getController();
    if (controller)
    {
        switch (controller->getType()){
        case (torque_control_types_e::TC_DRIVERCONTROL):
        {
            printf("Controller type is tc_Base");
            break;
        }
        case (torque_control_types_e::TC_PID):
        {
            printf("Controller type is tc_PID");
            break;
        }
        case (torque_control_types_e::TC_SlipTime):
        {
            printf("Controller type is tc_SlipTime");
            break;
        }
        case (torque_control_types_e::TC_NUM_CONTROLLERS):
        {
            printf("? LOL");
            break;
        }
        default:
        break;
        }
    }
}