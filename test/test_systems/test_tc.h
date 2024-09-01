#ifndef TC_TEST
#define TC_TEST
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <iostream>
#include "torque_controller.hpp"
#include "torque_controller.cpp"
#include "tc_system.h"
#include "tc_system.cpp"
#include "AutoPID.h"

TEST(tcTesting, test_traction_control_exists)
{
    torque_control_system tcSystem;
    EXPECT_EQ(tcSystem.getController()->getTorqueOutput(),0);
}

TEST(tcTesting, test_tc_system_toggle)
{
    torque_control_system tcSystem;
    torque_controller* torqueControl = tcSystem.getController();
    for (int i = 0; i < static_cast<int>(torque_control_types_e::TC_NUM_CONTROLLERS); i++)
    {
        torque_control_types_e c = tcSystem.getController()->getType();
        torque_control_types_e y = static_cast<torque_control_types_e>(i); 
        EXPECT_EQ(c,y);
        tcSystem.toggleController(0);
    }
}

TEST(tcTesting, test_tc_pid)
{

    torque_control_system tcSystem;
    torque_control_types_e typeToSet = torque_control_types_e::TC_PID;
    tcSystem.setActiveSystem(typeToSet);
    torque_control_types_e c = tcSystem.getController()->getType();
    ASSERT_EQ(c,typeToSet);
    torque_controller* torqueControl = tcSystem.getController();

    unsigned long t = 0;
    torqueControl->inittorque_controller(t);
    for (float i = 10; i < 6000; i += 10)
    {
        wheelSpeeds_s wsData = wheelSpeeds_s(i,i,i*1.01,i*1.01);
        int16_t driver_torque = 2400;
        int16_t out = torqueControl->calculate_torque(t+1,driver_torque,wsData);
        printf("time: %dms, slip: %f, output torque: %d\n",t,wsData.rl/wsData.fl,out);
        t++;
    }
}

TEST(tcTesting, test_tc_linear)
{

    torque_control_system tcSystem;
    torque_control_types_e typeToSet = torque_control_types_e::TC_DRIVERCONTROL;
    tcSystem.setActiveSystem(typeToSet);
    torque_control_types_e c = tcSystem.getController()->getType();
    ASSERT_EQ(c,typeToSet);
    torque_controller* torqueControl = tcSystem.getController();

    unsigned long t = 0;
    torqueControl->inittorque_controller(t);
    for (float i = 10; i < 6000; i += 10)
    {
        wheelSpeeds_s wsData = wheelSpeeds_s(i,i,i*1.01,i*1.01);
        int16_t driver_torque = 2400;
        int16_t out = torqueControl->calculate_torque(t+1,driver_torque,wsData);
        printf("time: %dms, slip: %f, output torque: %d\n",t,wsData.rl/wsData.fl,out);
        t++;
    }
}
#endif