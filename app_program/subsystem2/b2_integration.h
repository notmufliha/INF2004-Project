/** @file b2_integration.h
 * @brief Buddy 2 telemetry and B2-owned shared faults.
 */
#ifndef B2_INTEGRATION_H
#define B2_INTEGRATION_H
#include "motion_command.h"
ER b2_integration_init(void);
void b2_integration_fault(motion_fault_t fault);
void b2_integration_publish(const motion_status_snapshot_t *snapshot);
#endif
