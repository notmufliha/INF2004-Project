/*
 *	subsystem2.c
 *	Subsystem 2 -- Producer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf
 */

#include "subsystem2.h"
#include "../robot_config.h"
#include "b2_motor.h"

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

EXPORT ER subsystem2_init(void)
{
#if ROBOT_MODE
    /* Robot mode uses only the B2 motor task; no demo sample pool. */
    return E_OK;
#else
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : E_OK;
#endif
}

EXPORT ER subsystem2_start(void)
{
	/* Pinned to core 1 (TP_PRC2): samples cross to subsystem 3 on core 0 */
	#if ROBOT_MODE
    ID tid = app_start_task((FP)b2_motor_task, 8, 0, "b2_motor_task");
#else
    ID tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
                           "producer_task");
#endif
	return (tid < E_OK) ? tid : E_OK;
}
