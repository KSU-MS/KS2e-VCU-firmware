#ifndef LAUNCH_CONTROL_TEST
#define LAUNCH_CONTROL_TEST
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <iostream>
#include "launch_controller.hpp"
#include "launch_controller.cpp"
#include "launch_system.h"
#include "launch_system.cpp"
#include "AutoPID.h"

TEST(lcTesting, test_launch_control_exists)
{
    launchControlSystem lcSystem;
    launchController* launchControl = lcSystem.getController();
    unsigned long systime = 0;
    launchControl->initLaunchController(systime);
    EXPECT_EQ(launchControl->getState(), launchState::IDLE);
    launchControl->setState(launchState::WAITING_TO_LAUNCH,0);
    EXPECT_EQ(launchControl->getState(), launchState::WAITING_TO_LAUNCH);
    launchControl->setState(launchState::LAUNCHING,0);
    EXPECT_EQ(launchControl->getState(), launchState::LAUNCHING);
    launchControl->setState(launchState::FINISHED,0);
    EXPECT_EQ(launchControl->getState(), launchState::FINISHED);
}

TEST(lcTesting, test_lc_notorque)
{
    wheelSpeeds_s wheelSpeedData = {0,0,0,0};
    launchControlSystem lcSystem;
    launchController* launchControl = lcSystem.getController();
    unsigned long t = 0;
    launchControl->initLaunchController(t);
    int16_t torque = 2400;
    launchControl->run(t,torque,wheelSpeedData);
    launchControl->getTorqueOutput();
    EXPECT_EQ(launchControl->getTorqueOutput(),0);

    torque= 2400;
    t++;
    launchState nextState = launchControl->setState(launchState::WAITING_TO_LAUNCH,t);
    launchControl->run(t,torque,wheelSpeedData);
    EXPECT_EQ(launchControl->getTorqueOutput(),0);

    torque= 2400;
    t++;
    nextState = launchControl->setState(launchState::LAUNCHING,t);
    launchControl->run(t,torque,wheelSpeedData);
    EXPECT_NE(launchControl->getTorqueOutput(),0);

}

TEST(lcTesting, test_lc_ramp)
{
    wheelSpeeds_s wheelSpeedData = {0,0,0,0};
    launchControlSystem lcSystem;
    launchControlTypes_e typeToSet = launchControlTypes_e::LC_LOOKUP;
    lcSystem.setActiveSystem(typeToSet);
    launchControlTypes_e c = lcSystem.getController()->getType();
    ASSERT_EQ(c,typeToSet);
    launchController* launchControl = lcSystem.getController();
    unsigned long t = 0;
    int16_t nm = 2400;
    launchControl->initLaunchController(t);
    launchControl->setState(launchState::LAUNCHING,t);
    std::cout << "time" << "," << "torqueOut" << "\n";

    for (int i = 0; i < 1400; i += 10)
    {
        launchControl->run(i,nm,wheelSpeedData);
        std::cout << i << "," << launchControl->getTorqueOutput() << "\n";

    }

}

TEST(lcTesting, test_lc_system_toggle)
{
    launchControlSystem lcSystem;
    launchController* launchControl = lcSystem.getController();
    for (int i = 0; i < static_cast<int>(launchControlTypes_e::LC_NUM_CONTROLLERS); i++)
    {
        launchControlTypes_e c = lcSystem.getController()->getType();
        launchControlTypes_e y = static_cast<launchControlTypes_e>(i); 
        EXPECT_EQ(c,y);
        lcSystem.toggleController(0);
    }
}

TEST(lcTesting, test_lc_pid)
{

    launchControlSystem lcSystem;
    launchControlTypes_e typeToSet = launchControlTypes_e::LC_PID;
    lcSystem.setActiveSystem(typeToSet);
    launchControlTypes_e c = lcSystem.getController()->getType();
    ASSERT_EQ(c,typeToSet);
    launchController* launchControl = lcSystem.getController();

    unsigned long t = 0;
    launchControl->initLaunchController(t);
    launchControl->setState(launchState::LAUNCHING,t);
    for (float i = 10; i < 6000; i += 10)
    {
        wheelSpeeds_s wsData = wheelSpeeds_s(i,i,i*1.01,i*1.01);
        int16_t driver_torque = 2400;
        launchControl->run(t+1,driver_torque,wsData);
        printf("time: %dms, slip: %f, output torque: %d\n",t,wsData.rl/wsData.fl,launchControl->getTorqueOutput());
        t++;
    }
}

TEST(lcTesting, test_lc_linear)
{

    launchControlSystem lcSystem;
    launchControlTypes_e typeToSet = launchControlTypes_e::LC_LINEAR;
    lcSystem.setActiveSystem(typeToSet);
    launchControlTypes_e c = lcSystem.getController()->getType();
    ASSERT_EQ(c,typeToSet);
    launchController* launchControl = lcSystem.getController();

    unsigned long t = 0;
    launchControl->initLaunchController(t);
    launchControl->setState(launchState::LAUNCHING,t);
    for (float i = 10; i < 6000; i += 10)
    {
        wheelSpeeds_s wsData = wheelSpeeds_s(i,i,i*1.01,i*1.01);
        int16_t driver_torque = 2400;
        launchControl->run(t+1,driver_torque,wsData);
        printf("time: %dms, slip: %f, output torque: %d\n",t,wsData.rl/wsData.fl,launchControl->getTorqueOutput());
        t++;
    }
}
#endif