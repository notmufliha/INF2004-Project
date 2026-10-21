/*
 * subsystem3.h
 * Subsystem 3: consumer and shared sample statistics.
 *
 * This checkout retains the integrated producer/consumer example described
 * in TEAM.md. The attached Week 6 review assigns the real robot's B3 role to
 * IR line following and barcode decoding; those queues and hardware drivers
 * are not present in this codebase yet.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_CONSUMER        16
#define S3_BATCH_SIZE          5

typedef struct {
	UW received;
	INT min;
	INT max;
	W sum;
} APP_STATS;

/* IF-01: S2 sends SAMPLE_MSG pointers to the S3-owned mailbox. */
IMPORT ID mbx_samples;

/* IF-04: consumers must hold mtx_stats while reading or updating app_stats. */
IMPORT ID mtx_stats;
IMPORT APP_STATS app_stats;

IMPORT ER subsystem3_init(void);
IMPORT ER subsystem3_start(void);
IMPORT void consumer_task(INT stacd, void *exinf);

/* Pure helper used by the consumer and unit tests. */
IMPORT void s3_update_stats(APP_STATS *stats, INT value);

#endif /* SUBSYSTEM3_H */
