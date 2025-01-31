#ifndef TORQUE_CONTROLLER_HPP
#define TORQUE_CONTROLLER_HPP
#include <cmath>
#include <cstring>
#include <common_structs.h>
#include <AutoPID.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif

enum torque_control_types_e
{
    TC_DRIVERCONTROL = 0, // No firmware limiting, driver throttle directly
    TC_PID = 1, // PID slip control
    TC_SlipTime = 2, // Time slip control
    TC_NUM_CONTROLLERS
};

struct tc_diag_data_t {
    unsigned long launchElapsedTime;
    int16_t outputTorqueCommand;
    uint8_t joe = 0;
    uint8_t launchType;
    tc_diag_data_t(unsigned long time, int16_t outputTorque, uint8_t type)
        : launchElapsedTime(time), outputTorqueCommand(outputTorque),launchType(type) {}
};

class torque_controller
{
protected:
    unsigned long launchElapsedTime = 0;                 // ms time elapsed from start of launch
    int16_t driverTorqueRequest = 0;                         // Driver torque command if using pedal travel
    int16_t lcTorqueRequest = 0;                             // launch control system torque command
    int16_t outputTorqueCommand = 0;                         // The final torque command that will be output from the launch controller
public:
    void inittorque_controller(unsigned long sysTime);                         // general to all torque_controllers

    virtual int16_t calculate_torque(unsigned long elapsedTime, int16_t maxTorque, wheelSpeeds_s &wheelSpeedData) 
    {
        float torqueOut = 0;
        torqueOut = maxTorque;
        return static_cast<int>(torqueOut);
    };
    virtual torque_control_types_e getType() {return torque_control_types_e::TC_DRIVERCONTROL;}
    int getTorqueOutput() const { return outputTorqueCommand; };                                // general
    tc_diag_data_t getDiagData(); // 8 byte info on torque controller
};

class torque_controllerPID : public torque_controller
{
private:
    const double tireSlipThreshold = 0.3;
    const double tireSlipHigh = 0.5;
    double d_kp = 1.0;
    double d_ki = 1.0;
    double d_kd = 1.0;
    const double output_min = -0.4; // Minimum output of the PID controller
    const double output_max = 0; // Max output of the PID controller
    double input, setpoint, output;
    AutoPID pid;
    struct tc_pid_t
    {
        int16_t _curr_slip = 0;
        int16_t _target_slip = 0;
        int16_t _kp;
        int16_t _ki;
    } tc_pid_t;
    int joe = sizeof(tc_pid_t);
    const uint8_t TC_PID_TIMESTEP = 5; //ms
public:
    torque_controllerPID() : pid(&input, &setpoint, &output, output_min, output_max, d_kp, d_ki, d_kd)
    {
        output=0;
        setpoint = tireSlipHigh;
        pid.setBangBang(0);
        pid.setTimeStep(TC_PID_TIMESTEP); // 5ms is approx how often we send a torque command to the inverter
        // 3ms is the fastest we can do it
    }
    torque_controllerPID(double kp, double ki, double kd) : pid(&input, &setpoint, &output, output_min, output_max, kp, ki, kd)
    {
        tc_pid_t._curr_slip = 0;
        this->d_kd=kd;
        this->d_kp = kp;
        this->d_ki = ki;
        pid.setGains(d_kp,d_ki,d_kd);
        output=0;
        setpoint = tireSlipHigh;
        tc_pid_t._target_slip = static_cast<int16_t>(setpoint*100);

        input = 0;
        pid.setBangBang(0);
        pid.setTimeStep(TC_PID_TIMESTEP);
    }
    int16_t calculate_torque(unsigned long elapsedTime, int16_t maxTorque, wheelSpeeds_s &wheelSpeedData);
    torque_control_types_e getType() {return torque_control_types_e::TC_PID;}
    void setTcPidGains(double kp, double ki, double kd)
    {
        pid.setGains(d_kp,d_ki,d_kd);
    }
};

class torque_controllerSlipTime : public torque_controller
{
private:
    const double tireSlipHigh = 0.15; // Slip threshold
    unsigned long _lastSlip = 0; // Time slip began
    unsigned long slip_dT = 0; // Time delta since slip first began
    double slipTime = 0; // Slip * time, the x axis
    bool slipActive = false; // Flag to indicate if slip is active
    static const int numPoints = 6; // Number of data points
    double yTorqueRTD[numPoints] = {20, 30, 40, 50, 80, 120}; // TQ in Nm to reduce the output by
    double xSlipTime[numPoints] = {100, 200, 300, 500, 750, 1000}; // SlipTime, is % slip * time (ms) 
    double slopes[numPoints]; // Slopes at each point for interpolation
    double outputTorqueRTD = 0; // Torque Retard due to controller
public:
    int16_t calculate_torque(unsigned long elapsedTime, int16_t maxTorque, wheelSpeeds_s &wheelSpeedData);
    torque_control_types_e getType() {return torque_control_types_e::TC_SlipTime;}
};
#endif