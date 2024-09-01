#ifndef MCU_STATUS_TEST
#define MCU_STATUS_TEST
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MCU_status.hpp"

TEST(MCUstatusTesting,is_at_startup)
{
    MCU_status mcu_status;
    MCU_STATE init_state = mcu_status.get_state();
    EXPECT_EQ(init_state, MCU_STATE::STARTUP);
}

#endif