#include <gtest/gtest.h>
#include "test_launch_control.h"
#include "test_mcu_state.h"
#include "test_distance_tracker.h"
#include "test_tc.h"
int main(int argc, char **argv)
{


    testing::InitGoogleTest(&argc, argv);
	if (RUN_ALL_TESTS())
	;
	// Always return zero-code and allow PlatformIO to parse results
	return 0;
}
