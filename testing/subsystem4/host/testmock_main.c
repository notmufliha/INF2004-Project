/** @file testmock_main.c
 * @brief Buddy 4 testmock main module.
 */
#include "testmock_main.h"
#include "imu_test_support.h"
#include <stdio.h>
#include <assert.h>

#include "imu_mock.h"

/** @brief Main. */
int
main (void)
{
    imu_data_t data;

    imu_mock_init();

    /* Test 1: Stationary */
    imu_mock_set_scenario(IMU_MOCK_STATIONARY);
    assert(imu_mock_read(&data));
    assert(imu_test_support_near(data.accel_x, 0.0f));
    printf("Test 1 PASSED: Stationary\n");

    /* Test 2: Accelerating */
    imu_mock_set_scenario(IMU_MOCK_ACCELERATING);
    assert(imu_mock_read(&data));
    assert(imu_test_support_near(data.accel_x, 2.0f));
    printf("Test 2 PASSED: Accelerating\n");

    /* Test 3: Turning */
    imu_mock_set_scenario(IMU_MOCK_TURNING);
    assert(imu_mock_read(&data));
    assert(imu_test_support_near(data.turn_rate_dps, 45.0f));
    printf("Test 3 PASSED: Turning\n");

    /* Test 4: Climbing hump */
    imu_mock_set_scenario(IMU_MOCK_CLIMBING_HUMP);
    assert(imu_mock_read(&data));
    assert((data.roll_deg >= 4.0f) && (data.roll_deg <= 12.0f));
    assert(imu_test_support_near(data.pitch_deg, 0.0f));
    printf("Test 4 PASSED: Climbing hump\n");

    /* Test 5: Hump peak */
    imu_mock_set_scenario(IMU_MOCK_HUMP_PEAK);
    assert(imu_mock_read(&data));
    assert(imu_test_support_near(data.roll_deg, 12.0f));
    printf("Test 5 PASSED: Hump peak\n");

    /* Test 6: Descending hump */
    imu_mock_set_scenario(IMU_MOCK_DESCENDING_HUMP);
    assert(imu_mock_read(&data));
    assert((data.roll_deg >= 0.0f) && (data.roll_deg <= 12.0f));
    printf("Test 6 PASSED: Descending hump\n");

    /* Test 7: Sudden impact */
    imu_mock_set_scenario(IMU_MOCK_SUDDEN_IMPACT);
    assert(imu_mock_read(&data));
    assert(imu_test_support_near(data.accel_x, 15.0f));
    printf("Test 7 PASSED: Sudden impact\n");

    /* Test 8: Null pointer handling */
    assert(!imu_mock_read(NULL));
    printf("Test 8 PASSED: Null pointer handling\n");

    printf("\nAll Mock IMU tests passed!\n");

    return 0;
}

/*** end of file ***/

