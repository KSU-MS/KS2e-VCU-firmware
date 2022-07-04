#include "state_machine.hpp"

// initializes the mcu status and pedal handler
void StateMachine::init_state_machine(MCU_status &mcu_status)
{
    Serial.println("setting state");
    delay(1000);
    dash_->init_dashboard();                                      // do this before setting state
    set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE); // this is where it is failing rn
    Serial.println("initting pedals");
    delay(1000);
    pedals->init_pedal_handler();
    Serial.println("initting dash");
}

/* Handle changes in state */
void StateMachine::set_state(MCU_status &mcu_status, MCU_STATE new_state)
{
    Serial.println("getting state");
    if (mcu_status.get_state() == new_state)
    {
        return;
    }

    // exit logic
    switch (mcu_status.get_state())
    {
    case MCU_STATE::STARTUP:
#if DEBUG
        delay(100);
        Serial.println("in startup, breaking");
        delay(100);
#endif
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
        pm100->tryToClearMcFault();
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
        break;
    case MCU_STATE::ENABLING_INVERTER:
        break;
    case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
        // make dashboard stop buzzer
        mcu_status.set_activate_buzzer(false);
        digitalWrite(BUZZER, LOW);
        break;
    case MCU_STATE::READY_TO_DRIVE:
        break;
    }
#if DEBUG
    delay(200);
    Serial.println("actually setting state");
    delay(200);
#endif
    mcu_status.set_state(new_state);

    // entry logic
    // TODO unfuck the other pixel setting
    switch (new_state)
    {
    case MCU_STATE::STARTUP:
        break;
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
    {
#if DEBUG
        Serial.println("attempting to set dash shit");
        delay(100);
#endif
        dash_->set_dashboard_led_color(GREEN); // this is causing problems rn
        dash_->DashLedscolorWipe();
#if DEBUG
        Serial.println("set dash shit");
        delay(100);
#endif
        break;
    }
    case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
        dash_->set_dashboard_led_color(RED);
        dash_->DashLedscolorWipe();
        break;
    case MCU_STATE::ENABLING_INVERTER:
    {
        dash_->set_dashboard_led_color(YELLOW);
        dash_->DashLedscolorWipe();
        pm100->tryToClearMcFault();
        pm100->doStartup();
#if DEBUG
        Serial.println("MCU Sent enable command to inverter");
#endif
        break;
    }
    case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
        // make dashboard sound buzzer
        dash_->set_dashboard_led_color(ORANGE);
        dash_->DashLedscolorWipe();
        mcu_status.set_activate_buzzer(true);
        digitalWrite(BUZZER, HIGH);
        timer_ready_sound->reset();
#if DEBUG
        Serial.println("RTDS enabled");
#endif
        break;
    case MCU_STATE::READY_TO_DRIVE:
        dash_->set_dashboard_led_color(PINK);
        dash_->DashLedscolorWipe();
#if DEBUG
        Serial.println("Ready to drive");
#endif
        break;
    }
}

void StateMachine::handle_state_machine(MCU_status &mcu_status)
{
    switch (mcu_status.get_state())
    {
    case MCU_STATE::STARTUP:
    {
        break;
    }
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
    {
        if (!accumulator->GetIfPrechargeAttempted())
        {
            accumulator->sendPrechargeStartMsg();
        }

        bool accumulator_ready;
        if (accumulator->check_precharge_success() && (!accumulator->check_precharge_timeout()))
        {
            accumulator_ready = true;
        }
        else if ((!accumulator->check_precharge_timeout()) && (!accumulator->check_precharge_success()))
        {
            // if the accumulator hasnt finished precharge and it hasnt timed out yet, break
            break;
        }
        else if (accumulator->check_precharge_timeout() && (!accumulator->check_precharge_success()))
        {
            // attempt pre-charge again if the pre-charge has timed out
            // TODO add in re-try limitation number
            accumulator->sendPrechargeStartMsg();
            break;
        }
        else
        {
            // accumulator has timed out but it also pre-charged, continue
            accumulator_ready = true;
        }
        // if TS is above HV threshold, move to Tractive System Active
        if (pm100->check_TS_active() && accumulator_ready)
        {
#if DEBUG
            Serial.println("Setting state to TS Active from TS Not Active");
#endif
            set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
        }
        break;
    }
    case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
    {

#if DEBUG

        // Serial.println("TRACTIVE_SYSTEM_ACTIVE");
        // Serial.printf("button state: %i, pedal active %i\n", digitalRead(RTDbutton), mcu_status.get_brake_pedal_active());
#endif
        if (!pm100->check_TS_active())
        {
            set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
        }
        pm100->inverter_kick(0);

        // if start button has been pressed and brake pedal is held down, transition to the next state
        if (digitalRead(RTDbutton) == 0 && mcu_status.get_brake_pedal_active())
        {
#if DEBUG
            Serial.println("Setting state to Enabling Inverter");
#endif
            set_state(mcu_status, MCU_STATE::ENABLING_INVERTER);
        }
        break;
    }
    case MCU_STATE::ENABLING_INVERTER:
    {
        Serial.println("ENABLING INVERTER");
        if (!pm100->check_TS_active())
        {
            set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
            break;
        }

        // inverter enabling timed out
        if (pm100->check_inverter_enable_timeout())
        {
#if DEBUG
            Serial.println("Setting state to TS Active from Enabling Inverter");
#endif
            set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
            break;
        }

        // motor controller indicates that inverter has enabled within timeout period
        if (pm100->check_inverter_ready())
        {
#if DEBUG
            Serial.println("Setting state to Waiting Ready to Drive Sound");
#endif
            set_state(mcu_status, MCU_STATE::WAITING_READY_TO_DRIVE_SOUND);
        }
        break;

    case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
        Serial.println("WAITING READY TO DRIVER");
        if (!pm100->check_TS_active())
        {
            set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
            break;
        }
        if (!pm100->check_inverter_disabled())
        {
            set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
            break;
        }
        pm100->inverter_kick(1);

        // if the ready to drive sound has been playing for long enough, move to ready to drive mode
        if (timer_ready_sound->check())
        {
#if DEBUG
            Serial.println("Setting state to Ready to Drive");
#endif
            set_state(mcu_status, MCU_STATE::READY_TO_DRIVE);
        }
        break;
    }
    case MCU_STATE::READY_TO_DRIVE:
    {
        Serial.println("READY_TO_DRIVE");
        if (!pm100->check_TS_active())
        {
            set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
            break;
        }

        if (!pm100->check_inverter_disabled())
        {
            set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
            break; // TODO idk if we should break here or not but it sure seems like it
        }

        int calculated_torque = 0;
        bool accel_is_plausible = false;
        bool brake_is_plausible = false;
        bool accel_and_brake_plausible = false;

        // FSAE EV.5.5
        // FSAE T.4.2.10
        bool brake_is_active;
        pedals->verify_pedals(brake_is_active, accel_is_plausible, brake_is_plausible, accel_and_brake_plausible);

        mcu_status.set_no_accel_implausability(accel_is_plausible);
        mcu_status.set_no_brake_implausability(brake_is_plausible);
        mcu_status.set_no_accel_brake_implausability(accel_and_brake_plausible);

        if (
            accel_is_plausible &&
            brake_is_plausible &&
            accel_and_brake_plausible)
        {
            uint8_t max_t = mcu_status.get_max_torque();
            int16_t motor_speed = pm100->getmcMotorRPM();
            calculated_torque = pedals->calculate_torque(motor_speed, max_t);
        }
        else
        {
            Serial.println("not calculating torque");
            Serial.printf("no brake implausibility: %d\n", mcu_status.get_no_brake_implausability());
            Serial.printf("no accel implausibility: %d\n", mcu_status.get_no_accel_implausability());
            Serial.printf("no accel brake implausibility: %d\n", mcu_status.get_no_accel_brake_implausability());
        }
#if DEBUG
        // if (timer_debug_torque.check()) {
        /* Serial.print("MCU REQUESTED TORQUE: ");
         Serial.println(calculated_torque);
         Serial.print("MCU NO IMPLAUS ACCEL: ");
         Serial.println(mcu_status.get_no_accel_implausability());
         Serial.print("MCU NO IMPLAUS BRAKE: ");
         Serial.println(mcu_status.get_no_brake_implausability());
         Serial.print("MCU NO IMPLAUS ACCEL BRAKE: ");
         Serial.println(mcu_status.get_no_accel_brake_implausability());*/
        /* Serial.printf("ssok: %d\n", mcu_status.get_software_is_ok());
         Serial.printf("bms: %d\n", mcu_status.get_bms_ok_high());
         Serial.printf("imd: %d\n", mcu_status.get_imd_ok_high());*/
//}
#endif
        uint8_t torquePart1 = calculated_torque % 256;
        uint8_t torquePart2 = calculated_torque / 256;
        uint8_t angularVelocity1 = 0, angularVelocity2 = 0;
        bool emraxDirection = true; // forward
        bool inverterEnable = true; // go brrr
        uint8_t torqueCommand[] = {torquePart1, torquePart2, angularVelocity1, angularVelocity2, emraxDirection, inverterEnable, 0, 0};

        pm100->command_torque(torqueCommand);

        break;
    }
    }

    // things that are done every loop go here:
    pm100->updateInverterCAN();
    accumulator->updateAccumulatorCAN();
    if (pedal_check_->check())
    {
        mcu_status.set_brake_pedal_active(pedals->read_pedal_values());
    }
#if DEBUG
    if (debug_->check())
    {
        Serial.printf("button state: %i, pedal active %i\n", digitalRead(RTDbutton), mcu_status.get_brake_pedal_active());
        dash_->refresh_dash(pm100->getmcBusVoltage());
        pm100->debug_print();
    }

#endif

    // TODO update the dash here properly
}