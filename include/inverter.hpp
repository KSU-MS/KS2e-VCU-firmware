#ifndef INVERTER_HPP
#define INVERTER_HPP

#include <Arduino.h>
#include <Metro.h>

#include "FlexCAN_util.hpp"
#include "dashboard.hpp"
#include "inverter/mc_command_message.hpp"
#include "inverter/mc_current_info.hpp"
#include "inverter/mc_fault_codes.hpp"
#include "inverter/mc_internal_states.hpp"
#include "inverter/mc_motor_position_information.hpp"
#include "inverter/mc_temperatures.hpp"
#include "inverter/mc_voltage_information.hpp"
#include "parameters.hpp"
class Inverter {

private:
  bool spinForward = false;
  Metro *mc_kick_tim;
  Metro *timer_inverter_enable;
  Metro *timer_motor_controller_send;
  Metro *timer_current_limit;
  Dashboard *dash;
  void writeControldisableWithZeros();
  void writeEnableNoTorque();
  uint8_t disableWithZeros[8] = {
      0, 0, 0, 0,
      0, 0, 0, 0}; // The message to disable the controller/cancel lockout
  uint8_t enableNoTorque[8] = {
      0,           0, 0, 0,
      spinForward, 1, 0, 0}; // The message to enable the motor with zero torque
  MC_command_message pm100Msg{};
  MC_internal_states pm100State{};
  MC_motor_position_information pm100Speed{};
  MC_voltage_information pm100Voltage{};
  MC_temperatures_1 pm100temp1{};
  MC_temperatures_2 pm100temp2{};
  MC_temperatures_3 pm100temp3{};
  MC_fault_codes pm100Faults{};
  MC_current_information pm100CurrentInfo{};

public:
  // this is a member init list: https://www.youtube.com/watch?v=1nfuYMXjZsA
  Inverter(Metro *mc_kick_timer, Metro *en_tim, Metro *comm_timer,
           Metro *current_lim_tim, Dashboard *dash_)
      : mc_kick_tim(mc_kick_timer), timer_inverter_enable(en_tim),
        timer_motor_controller_send(comm_timer),
        timer_current_limit(current_lim_tim), dash(dash_){};
  uint32_t discharge_power_lim = DISCHARGE_POWER_LIM;
  uint32_t current_power = 0;
  void doStartup();
  void inverter_kick(bool enable);
  void forceMCdischarge();
  void updateInverterCAN();
  int getmcBusVoltage();
  int getmcMotorRPM();
  bool check_TS_active();
  bool check_inverter_disabled();
  bool command_torque(int16_t torque);
  void tryToClearMcFault();
  void enable_inverter();
  bool check_inverter_ready();
  bool check_inverter_enable_timeout();
  void debug_print();
  void update_power() {
    current_power = pm100CurrentInfo.get_dc_bus_current() / 10 *
                    pm100Voltage.get_dc_bus_voltage() / 10;
  }
  bool calc_and_send_current_limit(uint16_t pack_voltage,
                                   uint32_t discharge_power_limit,
                                   uint32_t charge_power_limit);
  bool speedMode = false;
  uint16_t angularVelocityTarget = 2000;
};

#endif
