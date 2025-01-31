#ifndef DISTANCE_TRACKER_TEST
#define DISTANCE_TRACKER_TEST
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "distance_tracker.h"
#include "parameters.hpp"
/*
distance_tracker works like this:
    void update(float amps, float volts, float rpm, float circumference, unsigned long newtime);


*/
/*
get_data() returns this:
struct energy_data_t
{
    int16_t energy_wh;
    int16_t eff_inst;
    int16_t distance_m;
    int16_t efficiency_kmkwh;
    energy_data_t(int16_t wh, int16_t eff, int16_t m, int16_t kmkwh) : energy_wh(wh), eff_inst(eff), distance_m(m), efficiency_kmkwh(kmkwh) {}
};
*/
TEST(distance_tracker_tests,init_zero)
{
    distance_tracker_s distance_tracker;
    EXPECT_EQ(distance_tracker.get_data().energy_wh,0);
    EXPECT_EQ(distance_tracker.get_data().eff_inst,0);
    EXPECT_EQ(distance_tracker.get_data().distance_m,0);
    EXPECT_EQ(distance_tracker.get_data().efficiency_kmkwh,0);
}
TEST(distance_tracker_tests,drive_onekm)
{
    distance_tracker_s distance_tracker;
    float amps = 10;
    float volts = 100;
    const float rpm = 11.58;

    distance_tracker.tick(0);
    for (unsigned long i = 0; i < 1000*3600; i+= 100)
    {
        distance_tracker.update(amps,volts,rpm,WHEEL_CIRCUMFERENCE,i);
    }
    // EXPECT_EQ(distance_tracker.get_data().energy_wh,1000);
    // EXPECT_EQ(distance_tracker.get_data().eff_inst,0);
    // EXPECT_EQ(distance_tracker.get_data().distance_m,1000);
    // EXPECT_EQ(distance_tracker.get_data().efficiency_kmkwh,1);
    EXPECT_NEAR(distance_tracker.capacity_ah,10,0.001); 
    EXPECT_NEAR(distance_tracker.energy_wh,1000,0.5); // half a watt hour accuracy
    EXPECT_NEAR(distance_tracker.distance_m,1000,1); // One meter accuracy
    EXPECT_NEAR(distance_tracker.efficiency_kmkwh,1,0.001);
    EXPECT_NEAR(distance_tracker.efficiency_instantaneous,1,0.001);
    printf("%fmeters %fah %fwh %fkm/kwh %fkm/kwh\n",distance_tracker.distance_m,distance_tracker.capacity_ah,distance_tracker.energy_wh,distance_tracker.efficiency_kmkwh,distance_tracker.efficiency_instantaneous);
    energy_data_t joe = distance_tracker.get_data();
    printf("%dmeters %dwh %dkm/kwh %dkm/kwh\n",joe.distance_m,joe.energy_wh,joe.eff_inst,joe.efficiency_kmkwh);
}
/*
    float capacity_ah;
    float energy_wh = 0;
    float distance_km = 0;
    float efficiency_kmkwh = 0;
    float efficiency_instantaneous = 0;
struct energy_data_t
{
    uint16_t energy_wh;
    uint16_t eff_inst;
    uint16_t distance_m;
    uint16_t efficiency_kmkwh;
    energy_data_t(uint16_t wh, uint16_t eff, uint16_t m, uint16_t kmkwh) : energy_wh(wh), eff_inst(eff), distance_m(m), efficiency_kmkwh(kmkwh) {}
};
    */



#endif