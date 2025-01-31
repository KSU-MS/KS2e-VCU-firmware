#ifndef LAUNCH_SYSTEM_H
#define LAUNCH_SYSTEM_H
#include <unordered_map>
#include "launch_controller.hpp"
#include <parameters.hpp>

class launchControlSystem
{
public:
    launchControlSystem(std::initializer_list<launchControlTypes_e> enabledTypes = {launchControlTypes_e::LC_DRIVERCONTROL, launchControlTypes_e::LC_LOOKUP, launchControlTypes_e::LC_PID, launchControlTypes_e::LC_LINEAR})
    {
        for (auto type : enabledTypes)
        {
            // Add only the enabled types to the map
            if (lc_map.find(type) != lc_map.end())
            {
                enabled_lc_map[type] = lc_map[type];
            }
        }
    }

    void setActiveSystem(launchControlTypes_e systype) { lcType = systype; }
    launchControlTypes_e getActiveSystem() { return lcType; }
    launchController *getController()
    {
        return static_cast<launchController *>(enabled_lc_map[lcType]);
    }
    void toggleController(unsigned long systime);
    void printControllerType(char buf[60]);

private:
    launchControlTypes_e lcType = launchControlTypes_e::LC_DRIVERCONTROL;
    launchController lc_base;
    launchControllerLookup lc_lookup;
    launchControllerPID lc_pid = launchControllerPID(4.0,2.0,1.0);
    launchControllerLinear lc_linear = launchControllerLinear(0, 100, 200, TORQUE_4);
    // This map should contain ALL types
    std::unordered_map<launchControlTypes_e, void *> lc_map = {
        {launchControlTypes_e::LC_DRIVERCONTROL, &lc_base},
        {launchControlTypes_e::LC_LOOKUP, &lc_lookup},
        {launchControlTypes_e::LC_PID, &lc_pid},
        {launchControlTypes_e::LC_LINEAR, &lc_linear}};

    // This map will only have enabled types
    std::unordered_map<launchControlTypes_e, void *> enabled_lc_map;
};

#endif