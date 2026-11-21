/* Compile for Cortex-M0+ and read the symbol sizes with arm-none-eabi-nm -S. */
#include "imu_analysis.h"
#include "imu_notice.h"
#include "imu_processing.h"

unsigned char sample_bytes[sizeof(imu_data_t)];
unsigned char motion_bytes[sizeof(imu_motion_t)];
unsigned char event_bytes[sizeof(imu_event_t)];
unsigned char notice_bytes[sizeof(imu_notice_t)];
unsigned char tracker_bytes[sizeof(imu_notice_tracker_t)];
unsigned char analysis_bytes[sizeof(imu_analysis_t)];
unsigned char processing_bytes[sizeof(imu_processing_t)];
