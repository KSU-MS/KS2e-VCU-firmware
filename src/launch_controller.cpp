#include "launch_controller.hpp"


void launchController::initLaunchController(unsigned long sysTime)
{
    driverTorqueRequest = 0;
    lcTorqueRequest = 0;
    outputTorqueCommand = 0;
    this->launchElapsedTime = 0;
    this->setState(launchState::IDLE,sysTime);
}
launchState launchController::getState()
{
    return this->launchControlState;
}
launchState launchController::setState(const launchState nextState, unsigned long sysTime)
{
    launchState currentState = this->getState();
    if (currentState == nextState)
    {
        return currentState;
    }
    this->launchControlState = nextState;
    this->launchStartTime = sysTime;
    return this->getState();
}

void launchController::run(unsigned long sysTime, int16_t &torqueRequest, wheelSpeeds_s &wheelSpeedData)
{
    launchCurrentTime = sysTime;
    launchElapsedTime = launchCurrentTime - launchStartTime;
    driverTorqueRequest = torqueRequest;

    if (this->getState() == launchState::LAUNCHING)
    {
        if (launchElapsedTime > launchControlMaxDuration)
        {
            this->setState(launchState::FINISHED,sysTime);
        }
        outputTorqueCommand = this->calculateTorque(launchElapsedTime, driverTorqueRequest, wheelSpeedData);
        // Clamp output torque
        if (outputTorqueCommand > driverTorqueRequest)
        {
            outputTorqueCommand = driverTorqueRequest;
        }
        else if (outputTorqueCommand < 0)
        {
            outputTorqueCommand = 0;
        }
    }
    else if (this->getState() == launchState::FINISHED)
    {
        outputTorqueCommand = driverTorqueRequest;
    }
    else { outputTorqueCommand = 0; } // Set output to zero if not in LAUNCHING or FINISHED
}

diagData_s launchController::getDiagData()
{
    diagData_s diagData = diagData_s{this->launchElapsedTime,this->outputTorqueCommand,static_cast<uint8_t>(this->getState()),static_cast<uint8_t>(this->getType())};
    return diagData;
}


// Calculate the launch control system's ideal torque output
int16_t launchControllerLookup::calculateTorque(unsigned long elapsedTime, int16_t maxTorque, wheelSpeeds_s &wheelSpeedData) 
{
    float torqueOut = 0;
    // @jstri114 peep dis
    torqueOut = (cal5 * pow(elapsedTime, 5)) + (cal4 * pow(elapsedTime, 4)) + (cal3 * pow(elapsedTime, 3)) + (cal2 * pow(elapsedTime, 2)) + (cal1 * (elapsedTime)) + calIntercept;
    torqueOut *= 10; // Scale for inverter
    // this linear equation goes nowhere
    // torqueOut = (1.0/25.0) * (elapsedTime) + calIntercept; // Performs the calibration curve math, shrimple linear for now
    return static_cast<int>(torqueOut);
}

// PID to get optimal slip
int16_t launchControllerPID::calculateTorque(unsigned long elapsedTime, int16_t maxTorque, wheelSpeeds_s &wheelSpeedData)
{
    float torqueOut = 0;
    // Calculate front and rear wheel speeds - take average of left and right
    float frontRpmAvg = ((wheelSpeedData.fl+wheelSpeedData.fr)/2);
    float rearRpmAvg =  ((wheelSpeedData.rl+wheelSpeedData.rr)/2);

    printf("LC Front avg: %f Rear avg: %f\n",frontRpmAvg,rearRpmAvg);
    printf("LC WHEELSPEEDS \nFL: %f FR: %f\nRL: %f RR: %f\n",wheelSpeedData.fl,wheelSpeedData.fr,wheelSpeedData.rl,wheelSpeedData.rr);

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
        this->input = (rearRpmAvg / frontRpmAvg) - 1;
    }

    pid.run(elapsedTime);
    torqueOut = maxTorque + (output * maxTorque);

    return static_cast<int>(torqueOut);  
}
