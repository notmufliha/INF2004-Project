/** @file task_producer.h
 * @brief Subsystem 2 sample producer and deterministic sample generator.
 */
#ifndef TASK_PRODUCER_H
#define TASK_PRODUCER_H

#include "../app.h"

void subsystem2_producer_task(INT stacd, void *p_exinf);
INT subsystem2_make_value(UW seq);

#endif /* TASK_PRODUCER_H */
