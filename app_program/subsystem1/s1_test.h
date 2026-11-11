/** @file s1_test.h
 * @brief Subsystem 1 on-target tests (make TEST=1).
 */

#ifndef S1_TEST_H
#define S1_TEST_H

/* Runs S1-U01..U08, U20, U23 and, with NET=lwip MQTT=1, U09/U10.
 * Prints one "[TEST] id PASS|FAIL description" line per check (same
 * format as test_report.h) and a SUMMARY line.  Does nothing in a
 * normal build. */
void s1_test_run (void);

#endif /* S1_TEST_H */

/*** end of file ***/

