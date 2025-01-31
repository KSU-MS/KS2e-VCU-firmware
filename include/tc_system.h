#ifndef TC_SYSTEM_H
#define TC_SYSTEM_H
#include <unordered_map>
#include "torque_controller.hpp"
#include <parameters.hpp>
#ifdef ARDUINO
#include <Arduino.h>
#endif
class torque_control_system
{
public:
    torque_control_system(std::initializer_list<torque_control_types_e> enabledTypes = {torque_control_types_e::TC_DRIVERCONTROL, torque_control_types_e::TC_PID, torque_control_types_e::TC_SlipTime})
    {
        for (auto type : enabledTypes)
        {
            // Add only the enabled types to the map
            if (tc_map.find(type) != tc_map.end())
            {
                enabled_tc_map[type] = tc_map[type];
            }
        }
    }

    void setActiveSystem(torque_control_types_e systype) { tcType = systype; }
    torque_control_types_e getActiveSystem() { return tcType; }
    torque_controller *getController()
    {
        return static_cast<torque_controller *>(enabled_tc_map[tcType]);
    }
    void toggleController(unsigned long systime);
    void printControllerType();

private:
    torque_control_types_e tcType = torque_control_types_e::TC_DRIVERCONTROL;
    torque_controller tc_base;
    torque_controllerPID tc_pid = torque_controllerPID(1.0, 0, 0);
    torque_controllerSlipTime tc_sliptime;

    // This map should contain ALL types
    std::unordered_map<torque_control_types_e, void *> tc_map = {
        {torque_control_types_e::TC_DRIVERCONTROL, &tc_base},
        {torque_control_types_e::TC_PID, &tc_pid},
        {torque_control_types_e::TC_SlipTime, &tc_sliptime}};

    // This map will only have enabled types
    std::unordered_map<torque_control_types_e, void *> enabled_tc_map;
};

#endif