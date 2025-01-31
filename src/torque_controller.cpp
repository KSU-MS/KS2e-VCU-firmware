#include "torque_controller.hpp"

void torque_controller::inittorque_controller(unsigned long sysTime)
{
    driverTorqueRequest = 0;
    lcTorqueRequest = 0;
    outputTorqueCommand = 0;
    this->launchElapsedTime = 0;
}


tc_diag_data_t torque_controller::getDiagData()
{
    tc_diag_data_t diagData = tc_diag_data_t{this->launchElapsedTime,this->outputTorqueCommand,static_cast<uint8_t>(this->getType())};
    return diagData;
}

// PID to get optimal slip
int16_t torque_controllerPID::calculate_torque(unsigned long elapsedTime, int16_t maxTorque, wheelSpeeds_s &wheelSpeedData)
{
    driverTorqueRequest = maxTorque;
    float torqueOut = 0;

    // Calculate front and rear wheel speeds - take average of left and right
    float frontRpmAvg = ((wheelSpeedData.fl+wheelSpeedData.fr)/2);
    float rearRpmAvg =  ((wheelSpeedData.rl+wheelSpeedData.rr)/2);
    printf("TC Front avg: %f Rear avg: %f\n",frontRpmAvg,rearRpmAvg);
    printf("TC WHEELSPEEDS \nFL: %f FR: %f\nRL: %f RR: %f\n",wheelSpeedData.fl,wheelSpeedData.fr,wheelSpeedData.rl,wheelSpeedData.rr);
    // avoid zero division
    if (frontRpmAvg <= 10 || rearRpmAvg <= 10)
    {
        this->input = 0; // treat it like 0 slip (maybe this is bad)
        pid.reset(elapsedTime);
    }
    else
    {
        // Slip = (rear / front) - 1
        // ie. 1000rpm/900rpm = 1.111..
        // 1.111 - 1 = 0.111 slip ratio
        this->input = wheelSpeedData.calc_slip();
    }
 
    pid.run(elapsedTime);
    torqueOut = maxTorque + (output * maxTorque);
    lcTorqueRequest = static_cast<int16_t>(torqueOut); // Pre clamping
    if (torqueOut > maxTorque)
    {
        torqueOut = maxTorque;
    }
    if (torqueOut < 0)
    {
        torqueOut = 0;
    }
    tc_pid_t._curr_slip = static_cast<int16_t>(input*100);
    outputTorqueCommand = static_cast<int16_t>(torqueOut); // Post-clamping
    return outputTorqueCommand;  
}

// SlipTime based torque reduction
int16_t torque_controllerSlipTime::calculate_torque(unsigned long elapsedTime, int16_t maxTorque, wheelSpeeds_s &wheelSpeedData)
{
    driverTorqueRequest = maxTorque;
    float torqueOut = 0;
    float slipRatio = 0;

    // Calculate front and rear wheel speeds - take average of left and right
    float frontRpmAvg = ((wheelSpeedData.fl+wheelSpeedData.fr)/2);
    float rearRpmAvg =  ((wheelSpeedData.rl+wheelSpeedData.rr)/2);
    printf("TC Front avg: %f Rear avg: %f\n",frontRpmAvg,rearRpmAvg);
    printf("TC WHEELSPEEDS \nFL: %f FR: %f\nRL: %f RR: %f\n",wheelSpeedData.fl,wheelSpeedData.fr,wheelSpeedData.rl,wheelSpeedData.rr);
    // avoid zero division
    if (frontRpmAvg <= 10|| rearRpmAvg <= 10)
    {
        slipRatio = 0; // treat it like 0 slip (maybe this is bad)
    }
    else
    {
        // Slip = (rear / front) - 1
        // ie. 1000rpm/900rpm = 1.111..
        // 1.111 - 1 = 0.111 slip ratio
        slipRatio = (rearRpmAvg / frontRpmAvg) - 1;
    }
 
    // Set the "state" of the controller to capture the time the slip ratio crossed the threshold (which is used to calculate slip * time)
    if (slipRatio > tireSlipHigh && slipActive == false) { // If the slip ratio crosses the threshold, set the last slip time to now and set the slip active flag to true
        _lastSlip = elapsedTime;
        slipActive = true;
    } else if (slipRatio > tireSlipHigh && slipActive == true) { // If slip ratio above threshold and slip active, calculate time since beginning to slip and slip * time
        slip_dT = elapsedTime - _lastSlip; // Calculate time since slip started
        slipTime = (slipRatio * slip_dT); // Calculate slip * time
    } else {
        slipActive = false; // Catch all, should disable whenever slip ratio falls below threshold and slip active was true
    }
   
    // Check bounds
    if (slipTime < xSlipTime[0] || slipTime > xSlipTime[numPoints - 1]) {
        outputTorqueRTD = 0;
    }

    // Find the interval where 'x' belongs
    for (int i = 0; i < numPoints - 1; i++) {
        if (slipTime <= xSlipTime[i + 1]) {
            double slope = (yTorqueRTD[i + 1] - yTorqueRTD[i]) / (xSlipTime[i + 1] - xSlipTime[i]); // Calculate slope for linear interpolation
            outputTorqueRTD = yTorqueRTD[i] + slope * (slipTime - xSlipTime[i]); // Calculate output based on linear interpolation
        }
    }
    // If slipTime lands on the last point, return the last value
    if (slipTime = xSlipTime[numPoints - 1]) {
        outputTorqueRTD = yTorqueRTD[numPoints - 1];
    }
    if (slipActive)
    {
        torqueOut = driverTorqueRequest - (outputTorqueRTD * 10); // times 10 because your input is coming from the output of calculate_torque based on the APPS which is Nmx10, and subtracts the amount of torque you want to remove from the driver's request
    }
    else
    {
        torqueOut = maxTorque;
    }
    // Limit torque outputs like normal
    lcTorqueRequest = static_cast<int16_t>(torqueOut); // Pre clamping
    if (torqueOut > maxTorque)
    {
        torqueOut = maxTorque;
    }
    if (torqueOut < 0)
    {
        torqueOut = 0;
    }

    outputTorqueCommand = static_cast<int16_t>(torqueOut); // Post-clamping
    return outputTorqueCommand;  
}