#pragma once

#include <string.h>
#include <stdint.h>

#pragma pack(push,1)
const int LAUNCH_CONTROL_BUTTON = 5;
class Dashboard 
{

public:
    Dashboard() = default;
    inline uint8_t get_buttons()  const {return button_states;}
    // Not Connected
    inline bool get_button1()  const {return (button_states&0x01);} 
    // Green button
    inline bool get_button2()  const {return (button_states&0x02);}
    // Blue button
    inline bool get_button3()  const {return (button_states&0x04);} 
    // Ready to drive
    inline bool get_button4()  const {return (button_states&0x08);}
    // Red button
    inline bool get_button5()  const {return (button_states&0x10);}
    // Yellow button
    inline bool get_button6()  const {return (button_states&0x20);}

    inline void set_buttons(uint8_t inputs)        { button_states = inputs; }
    /**
     * @brief Get a buttons value (high or low)
     * high is pressed!
     * 
     * @param button this should be the button number you want to check the status of
     *  as it is named on the dash (1-6)
     * @return true 
     * @return false 
     */
    bool get_button(uint8_t button) {return (button_states & (0x1 << (button - 1)));}
    /**
     * @brief Get the button held duration object
     * 
     * @param button 
     * @param duration_ms 
     * @return true 
     * @return false 
     */
    bool get_button_held_duration(uint8_t button, unsigned long duration_ms);
    bool get_button_released_duration(uint8_t button, unsigned long duration_ms);

    void updateDashCAN();
    /**
     * @brief Get the button last pressed time object
     * 
     * @param buttonNumber 
     * @return unsigned long 
     */
    unsigned long get_button_last_pressed_time(uint8_t buttonNumber);
    void set_button_last_pressed_time(unsigned long setpoint,uint8_t buttonNumber);
    void update_dash(uint8_t input);
    bool new_dash_msg_received;
    float last_received_timestamp = 0;
private:
    uint8_t button_states;
    elapsedMillis button_last_pressed_time[6] = {0,0,0,0,0,0}; // Maybe this should be a map :skullemoji:
    void reset_all_button_timers();
};

#pragma pack(pop)
