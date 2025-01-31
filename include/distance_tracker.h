#ifndef DISTANCE_TRACKER_H
#define DISTANCE_TRACKER_H
#include <stdint.h>
#include <deque>
#ifdef ARDUINO
#include <Arduino.h>
#endif
struct old_new_t
{
    float newval = 0;
    float oldval = 0;
};

struct energy_data_t
{
    uint16_t energy_wh;
    uint16_t eff_inst;
    uint16_t distance_m;
    uint16_t efficiency_kmkwh;
    energy_data_t(uint16_t wh, uint16_t eff, uint16_t m, uint16_t kmkwh) : energy_wh(wh), eff_inst(eff), distance_m(m), efficiency_kmkwh(kmkwh) {}
};

struct RollingAverage {
    std::deque<float> buffer;
    float sum = 0;
    int maxSize;

    RollingAverage(int maxBufferSize) : maxSize(maxBufferSize) {}

    void addValue(float value) {
        buffer.push_back(value);
        sum = sum + value;
        // printf("Sum: %f\n",sum);
        if (buffer.size() > maxSize) {
            sum -= buffer.front();
            buffer.pop_front();
        }
    }

    float getAverage() const {
        // printf("%d, %f\n",buffer.empty(),buffer.front());
        return buffer.empty() ? 0.0 : sum / buffer.size();
    }
};


class distance_tracker_s
{
public:
    void tick(unsigned long newtime)
    {
        time = newtime;
    }
    void update(float amps, float volts, float rpm, float circumference, unsigned long newtime)
    {
        // calc elapsed time
        unsigned long elapsed_time_ms = newtime - this->time;
        float elapsed_time_seconds = static_cast<float>(elapsed_time_ms) / 1000;
        // update time
        this->time = newtime;
        // calculate distance travelled during the last update time
        distance_m += elapsed_time_seconds * velocity_ms.oldval;
        // calculate power used during the last update time
        float kwh = (elapsed_time_seconds / 3600) * power_kw.oldval;
        energy_wh += kwh;

        // calculate amp hrs used during the last update time
        capacity_ah += (elapsed_time_seconds / 3600) * current_amps.oldval;

        // update efficiencies
        efficiency_kmkwh = distance_m/energy_wh;
        efficiency_instantaneous = (elapsed_time_seconds * velocity_ms.oldval) / (elapsed_time_seconds / 3600 * power_kw.oldval);
        if (power_kw.oldval <= 0.01)
        {
            efficiency_instantaneous = 0;
        }
        avgEff.addValue(efficiency_instantaneous);
        // set old vals to the previous new one
        power_kw.oldval = power_kw.newval;
        current_amps.oldval = current_amps.newval;
        velocity_ms.oldval = velocity_ms.newval;

        // set new vals to the newest
        current_amps.newval = amps;
        // Calculate power
        power_kw.newval = amps * volts;
        // Calculate speed
        velocity_ms.newval = rpm/60 * circumference;
    }

    // This is to calculate just using an inputted velocity
    void update(float amps, float volts, float velocity_m_s, unsigned long newtime)
    {
        // calc elapsed time
        unsigned long elapsed_time_ms = newtime - this->time;
        float elapsed_time_seconds = static_cast<float>(elapsed_time_ms) / 1000;
        // update time
        this->time = newtime;
        // calculate distance travelled during the last update time
        distance_m += elapsed_time_seconds * velocity_ms.oldval;
        // calculate power used during the last update time
        float kwh = (elapsed_time_seconds / 3600) * power_kw.oldval;
        energy_wh += kwh;

        // calculate amp hrs used during the last update time
        capacity_ah += (elapsed_time_seconds / 3600) * current_amps.oldval;

        // update efficiencies
        efficiency_kmkwh = distance_m/energy_wh;
        efficiency_instantaneous = (elapsed_time_seconds * velocity_ms.oldval) / (elapsed_time_seconds / 3600 * power_kw.oldval);
        if (power_kw.oldval <= 0.01)
        {
            efficiency_instantaneous = 0;
        }
        avgEff.addValue(efficiency_instantaneous);
        // set old vals to the previous new one
        power_kw.oldval = power_kw.newval;
        current_amps.oldval = current_amps.newval;
        velocity_ms.oldval = velocity_ms.newval;

        // set new vals to the newest
        current_amps.newval = amps;
        // Calculate power
        power_kw.newval = amps * volts;
        // Calculate speed
        velocity_ms.newval = velocity_m_s;
    }
    energy_data_t get_data()
    {
        energy_data_t data = energy_data_t(static_cast<uint16_t>(energy_wh*10), static_cast<uint16_t>(avgEff.getAverage()*1000), static_cast<uint16_t>(distance_m), static_cast<uint16_t>(efficiency_kmkwh*1000));
        return data;
    }
    float capacity_ah=0;
    float energy_wh = 0;
    float distance_m = 0;
    float efficiency_kmkwh = 0;
    float efficiency_instantaneous = 0;
private:
    old_new_t power_kw;
    old_new_t current_amps;
    old_new_t velocity_ms;
    unsigned long time;
    RollingAverage avgEff = RollingAverage(100);
};

#endif