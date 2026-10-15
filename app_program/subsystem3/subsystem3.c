/* Subsystem 3 object ownership and statistics implementation. */

#include "subsystem3.h"

EXPORT ID mbx_samples;
EXPORT ID mtx_stats;
EXPORT APP_STATS app_stats;

EXPORT void s3_update_stats(APP_STATS *stats, INT value)
{
	if (stats->received == 0 || value < stats->min) {
		stats->min = value;
	}
	if (stats->received == 0 || value > stats->max) {
		stats->max = value;
	}
	stats->sum += value;
	stats->received++;
}

EXPORT ER subsystem3_init(void)
{
	T_CMBX cmbx = {0};
	T_CMTX cmtx = {0};
	ER er;

	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	if (mtx_stats < E_OK) {
		er = (ER)mtx_stats;
		(void)tk_del_mbx(mbx_samples);
		mbx_samples = 0;
		mtx_stats = 0;
		return er;
	}

	app_stats.received = 0;
	app_stats.min = 0;
	app_stats.max = 0;
	app_stats.sum = 0;
	return E_OK;
}

EXPORT ER subsystem3_start(void)
{
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	return (tid < E_OK) ? (ER)tid : E_OK;
}
