#include "launch_system.h"
// #include <cstring>
void launchControlSystem::toggleController(unsigned long systime)
{
    int current_controller = static_cast<int>(lcType);
    int next_controller = current_controller + 1;
    while (true)
    {
        if (next_controller >= static_cast<int>(launchControlTypes_e::LC_NUM_CONTROLLERS))
        {
            next_controller = 0;
        }
        if (enabled_lc_map.find(static_cast<launchControlTypes_e>(next_controller)) != enabled_lc_map.end())
        {
            break; // Found an enabled controller
        }
        next_controller++;
    }

    this->setActiveSystem(static_cast<launchControlTypes_e>(next_controller));
    // init new system
    this->getController()->initLaunchController(systime);
}

void launchControlSystem::printControllerType(char buf[60])
{
    launchController* controller = getController();
    if (controller)
    {
        switch (controller->getType()){
        case (launchControlTypes_e::LC_DRIVERCONTROL):
        {
            char string[] = "Controller type is launchControllerBase";
            memcpy(buf,string,sizeof(string));
            break;
        }
        case (launchControlTypes_e::LC_LOOKUP):
        {
            char string[] = "Controller type is launchControllerLookup";
            memcpy(buf,string,sizeof(string));
            break;
        }
        case (launchControlTypes_e::LC_PID):
        {
            char string[] = "Controller type is launchControllerPID";
            memcpy(buf,string,sizeof(string));
            break;
        }
        case (launchControlTypes_e::LC_LINEAR):
        {
            char string[] = "Controller type is launchControllerLinear";
            memcpy(buf,string,sizeof(string));
            break;
        }
        case (launchControlTypes_e::LC_NUM_CONTROLLERS):
        {
            char string[] = "? LOL";
            memcpy(buf,string,sizeof(string));
            break;
        }
        default:
        break;
        }
    }
}