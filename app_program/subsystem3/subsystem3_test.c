/* On-target Subsystem 3 tests. Compiled and run with make TEST=1. */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test(void)
{
	APP_STATS stats = {0};
	ER er;

	TEST_INIT();
	s3_update_stats(&stats, 42);
	s3_update_stats(&stats, 7);
	s3_update_stats(&stats, 99);
	TEST_CHECK("S3-U01", stats.received == 3 && stats.min == 7 &&
		   stats.max == 99 && stats.sum == 148,
		   "statistics aggregate count, min, max, and sum");

	er = tk_loc_mtx(mtx_stats, TMO_FEVR);
	if (er == E_OK) {
		TEST_CHECK("S3-U02", app_stats.received == 0 ||
			   (app_stats.min <= app_stats.max && app_stats.min >= 0 &&
			    app_stats.max <= 100),
			   "live sample statistics remain in the documented range");
		er = tk_unl_mtx(mtx_stats);
	}
	if (er < E_OK) {
		TEST_CHECK("S3-U02", FALSE, "statistics mutex is usable");
	}

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
