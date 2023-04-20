#pragma once

#include <string.h>
#include <stdint.h>
#ifdef HT_DEBUG_EN
    #include "Arduino.h"
#endif
#pragma pack(push,1)
class Dashboard 
{

public:
    Dashboard() = default;
    Dashboard(const uint8_t buf[8]) { load(buf); }

    inline void load(const uint8_t buf[])  { memcpy(this, buf, sizeof(*this)); }
    inline void write(uint8_t buf[]) const { memcpy(buf, this, sizeof(*this)); }
    inline bool get_buttons()  const {return button_states;}

    inline bool get_button1()  const {return !(button_states&0x01);}
    inline bool get_button2()  const {return button_states&0x02;}
    inline bool get_button3()  const {return button_states&0x04;}
    inline bool get_button4()  const {return button_states&0x08;}
    inline bool get_button5()  const {return button_states&0x10;}
    inline bool get_button6()  const {return button_states&0x20;}

    inline void set_buttons(const uint8_t inputs)        { button_states = inputs; }

    void updateDashCAN();

    uint8_t *ByteEachDigit(int num);

    
private:
    uint8_t button_states;

    uint8_t BusVolt_ByteEachDigit[8] = { 0 };

    int counter = 0; // This kinda cringe ngl but cannot think of alt way.
};

#pragma pack(pop)
