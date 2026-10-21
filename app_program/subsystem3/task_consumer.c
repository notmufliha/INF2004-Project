/* Subsystem 3 consumer task: receive, aggregate, report, and release samples. */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"

EXPORT void consumer_task(INT stacd, void *exinf)
{
	SAMPLE_MSG *msg;
	UB text[S4_SUMMARY_MAX];
	INT len;
	UW count;
	ER er;

	(void)stacd;
	(void)exinf;

	for (;;) {
		er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "tk_rcv_mbx");
			continue;
		}

		/* Do not strand the producer's bounded-pool slot on lock errors. */
		er = tk_loc_mtx(mtx_stats, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "tk_loc_mtx");
			(void)tk_rel_mpf(mpf_samples, msg);
			(void)tk_sig_sem(sem_slots, 1);
			continue;
		}
		s3_update_stats(&app_stats, msg->value);
		count = app_stats.received;
		er = tk_unl_mtx(mtx_stats);
		if (er < E_OK) {
			CHECK(er, "tk_unl_mtx");
		}

		if (count % S3_BATCH_SIZE == 0) {
			len = tm_sprintf(text, (const UB*)"n=%u last=%d from core %d",
					 count, msg->value, msg->prc - 1);
			if (len > 0 && len <= S4_SUMMARY_MAX) {
				er = tk_snd_mbf(mbf_summary, text, len, TMO_POL);
				if (er == E_OK) {
					er = tk_set_flg(flg_events, FLG_BATCH);
					if (er < E_OK) CHECK(er, "tk_set_flg");
				}
			}
		}

		/* Always return both resources so S2 cannot deadlock at pool capacity. */
		er = tk_rel_mpf(mpf_samples, msg);
		if (er < E_OK) CHECK(er, "tk_rel_mpf");
		er = tk_sig_sem(sem_slots, 1);
		if (er < E_OK) CHECK(er, "tk_sig_sem");
	}
}
