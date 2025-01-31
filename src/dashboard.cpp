
#include "FlexCAN_util.hpp"
#include "dashboard.hpp"

unsigned long Dashboard::get_button_last_pressed_time(uint8_t buttonNumber)
{
    return button_last_pressed_time[buttonNumber - 1];
}
void Dashboard::set_button_last_pressed_time(unsigned long setpoint, uint8_t buttonNumber)
{
    button_last_pressed_time[buttonNumber - 1] = setpoint;
}

// Reset all button timers
void Dashboard::reset_all_button_timers()
{
    for (uint8_t i = 0; i < sizeof(button_last_pressed_time) / sizeof(button_last_pressed_time[0]); i++)
    {
        set_button_last_pressed_time(0, i+1);
    }
}


void Dashboard::update_dash(uint8_t input)
{
    float timestamp = millis() / float(1000);
#if DEBUG
    Serial.printf("Dash last received interval: %f\n", (timestamp - (this->last_received_timestamp)));
#endif
    this->last_received_timestamp = timestamp;
    for (int i = 0; i < 6; i++)
    {
        uint8_t bit = (0x1 << i);
        bool new_val = input & bit;
        bool old_val = (this->get_buttons() & bit);
        if (new_val != old_val)
        {
#if DEBUG
            Serial.printf("Button number %d changed from %d to %d\n", i + 1, old_val, new_val);
            Serial.printf("%d\n",static_cast<int>(this->get_button_last_pressed_time(i+1)));
#endif
            this->set_button_last_pressed_time(0, i+1);
        }
    }
    this->set_buttons(input);
}
// true if time since button pressed > duration_ms
bool Dashboard::get_button_held_duration(uint8_t button, unsigned long duration_ms)
{
    bool wasHeld = this->get_button(button) && (this->get_button_last_pressed_time(button) >= duration_ms);
    if (wasHeld) {
        #if DEBUG
        Serial.printf("last pressed time: %d\n",static_cast<int>(button_last_pressed_time[button-1]));
        #endif
        set_button_last_pressed_time(0,button);
        }
    return wasHeld;
}
// true if time since a button was released > duration_ms (if you want to delay an action for example)
bool Dashboard::get_button_released_duration(uint8_t button, unsigned long duration_ms)
{
    bool wasReleased = !(this->get_button(button)) && (this->get_button_last_pressed_time(button) >= duration_ms);
    if (wasReleased) {set_button_last_pressed_time(0,button);}
    return wasReleased;
}

void Dashboard::updateDashCAN()
{
    CAN_message_t rxMsg;

    if (ReadInverterCAN(rxMsg))
    {
        if (rxMsg.id == ID_DASH_BUTTONS)
        {
            float timestamp = millis() / float(1000);
#if DEBUG
            Serial.printf("Dash last received interval: %f\n", (timestamp - last_received_timestamp));
#endif
            last_received_timestamp = timestamp;
            // WriteToDaqCAN(rxMsg);
            button_states = rxMsg.buf[0];
        }
    }
}
