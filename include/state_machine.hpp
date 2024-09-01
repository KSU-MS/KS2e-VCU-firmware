#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP
#include <Arduino.h>
#include <Metro.h>
#include <EEPROM.h>
#include "MCU_status.hpp"
#include "inverter.hpp"
#include "accumulator.hpp"
#include "pedal_handler.hpp"
#include "dashboard.hpp"
#include "parameters.hpp"
#include "launch_system.h"
#include "tc_system.h"
#include <distance_tracker.h>

#define ODOMETER_EEPROM_ADDR 0
#define ONTIME_EEPROM_ADDR 10
#define TORQUE_MODE_EEPROM_ADDR 20
class StateMachine
{
private:
    Inverter *pm100;
    Accumulator *accumulator;
    Metro *timer_ready_sound; // Time to play RTD sound
    Dashboard *dash_;
    Metro *debug_;
    PedalHandler *pedals;
    launchControlSystem *lcSystem;
    torque_control_system *tcSystem;
    Metro *pedal_check_;
    can_obj_ksu_ev_can_h_t *ksu_can;
    distance_tracker_s distance_tracker_motor;
    distance_tracker_s distance_tracker_fl;
    distance_tracker_s distance_tracker_vectornav;
    vn_data_t vectornav_data;
    // To hold initial values from eeprom:
    unsigned long _lifetime_distance = 0;
    unsigned long _lifetime_on_time = 0;
    time_and_distance_tracker_t time_and_distance_t;
    void set_state(MCU_status &mcu_status, MCU_STATE new_state);
    void send_state_msg(MCU_status &mcu_status);
    Metro can_100hz_timer = Metro(10,1);
    Metro can_20hz_timer = Metro(50,1);
    Metro can_10hz_timer = Metro(100,1);
    Metro _log_distance_timer_10s = Metro(10000,1);
    Metro _log_distance_timer_60s = Metro(60000,1);

public:
    StateMachine(Inverter *inv, Accumulator *acc, Metro *rs_tim, Dashboard *dash, Metro *debug, PedalHandler *pedals, launchControlSystem *lcSys, torque_control_system *tcSys,Metro *ped_t,can_obj_ksu_ev_can_h_t *ksu_can_)
        : pm100(inv), accumulator(acc), timer_ready_sound(rs_tim), dash_(dash), debug_(debug), pedals(pedals), lcSystem(lcSys),tcSystem(tcSys),pedal_check_(ped_t),ksu_can(ksu_can_){};

    void init_state_machine(MCU_status &mcu_status);
    void handle_state_machine(MCU_status &mcu_status);
    void update_daq_can();
    void handle_distance_trackers(MCU_status &mcu_status);
    time_and_distance_tracker_t get_lifetime_data()
    {
        return time_and_distance_t;
    }
// void joe_mock_lc(MCU_status* mcu_status, int torq, bool implaus);

};

#endif