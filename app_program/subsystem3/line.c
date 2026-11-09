/**
 * @file line.c
 * @brief Two-channel IR line sampling integrated with B1 and the VC.
 */
#include "line.h"

#include <stdbool.h>

#include "../subsystem1/subsystem1.h"
#include "barcode.h"
#include "b3_line_barcode.h"
#include "drv_ir_adc.h"

#define B3_ADC_LEFT_CHANNEL (0)
#define B3_ADC_CENTRE_CHANNEL (1)
#define B3_STATE_LOCK_TIMEOUT_MS (5)
#define B3_MBF_MSG_OVERHEAD (8U)
#define B3_LINE_DIAGNOSTIC_PERIOD_MS (500U)

static B3_STATE g_b3_state;
static bool g_line_sample_valid;
static uint32_t g_line_sample_ms;
static uint8_t g_line_started;

EXPORT ID mbf_nav_cmd;
EXPORT ID mtx_b3_state;

static void line_task(INT stacd, void *p_exinf);
static ER b3_set_line_state(uint16_t left, uint16_t centre, uint8_t right,
			    uint8_t flags, uint32_t sample_ms);

ER line_init(void)
{
	T_CMBF cmbf = {0};
	T_CMTX cmtx = {0};
	ER error;

	if (mbf_nav_cmd > 0 || mtx_b3_state > 0) {
		return E_OBJ;
	}

	error = drv_ir_adc_init();
	if (error != E_OK) {
		return error;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_b3_state = tk_cre_mtx(&cmtx);
	if (mtx_b3_state < E_OK) {
		ER cleanup_error;

		error = (ER)mtx_b3_state;
		cleanup_error = drv_ir_adc_close();
		if (cleanup_error != E_OK) {
			CHECK(cleanup_error, "B3 ADC cleanup");
		}
		return error;
	}

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = (SZ)(B3_NAV_QUEUE_LENGTH *
			  (sizeof(uint8_t) + B3_MBF_MSG_OVERHEAD));
	cmbf.maxmsz = (INT)sizeof(uint8_t);
	mbf_nav_cmd = tk_cre_mbf(&cmbf);
	if (mbf_nav_cmd < E_OK) {
		ER cleanup_error;

		error = (ER)mbf_nav_cmd;
		cleanup_error = tk_del_mtx(mtx_b3_state);
		if (cleanup_error != E_OK) {
			CHECK(cleanup_error, "B3 mutex cleanup");
		}
		mtx_b3_state = 0;
		cleanup_error = drv_ir_adc_close();
		if (cleanup_error != E_OK) {
			CHECK(cleanup_error, "B3 ADC cleanup");
		}
		return error;
	}

	g_b3_state.ir_left = 0U;
	g_b3_state.ir_centre = 0U;
	g_b3_state.ir_right = 0U;
	g_b3_state.line_flags = B3_LINE_F_LOST;
	g_b3_state.last_barcode = 0U;
	g_b3_state.last_nav_command = NAV_NONE;
	g_line_sample_valid = false;
	g_line_sample_ms = 0U;
	g_line_started = 0U;
	return E_OK;
}

ER line_start(void)
{
	ID task_id;

	if (mtx_b3_state <= 0 || mbf_nav_cmd <= 0 || g_line_started != 0U) {
		return E_OBJ;
	}
#if TK_SUPPORT_SMP
	if (tk_get_prc() != 1) {
		return E_CTX;
	}
#endif

	task_id = app_start_task((FP)line_task, 16, TP_PRC1, "b3_line_task");
	if (task_id < E_OK) {
		return (ER)task_id;
	}
	g_line_started = 1U;
	return E_OK;
}

ER b3_get_state(B3_STATE *state)
{
	ER error;

	if (state == NULL) {
		return E_PAR;
	}
	error = tk_loc_mtx(mtx_b3_state, B3_STATE_LOCK_TIMEOUT_MS);
	if (error != E_OK) {
		return error;
	}
	*state = g_b3_state;
	error = tk_unl_mtx(mtx_b3_state);
	return error;
}

ER b3_get_timed_state(B3_STATE *state, uint32_t *sample_ms)
{
	ER error;
	ER unlock_error;

	if (state == NULL || sample_ms == NULL) {
		return E_PAR;
	}
	error = tk_loc_mtx(mtx_b3_state, B3_STATE_LOCK_TIMEOUT_MS);
	if (error != E_OK) {
		return error;
	}
	*state = g_b3_state;
	*sample_ms = g_line_sample_ms;
	error = g_line_sample_valid ? E_OK : E_OBJ;
	unlock_error = tk_unl_mtx(mtx_b3_state);
	if (unlock_error != E_OK) {
		CHECK(unlock_error, "B3 state unlock");
		return unlock_error;
	}
	return error;
}

void b3_publish_barcode(uint8_t letter)
{
	uint8_t code;
	ER lock_error;
	ER unlock_error;
	ER send_error;
	ER report_error;

	if (letter < (uint8_t)'A' || letter > (uint8_t)'Z') {
		CHECK(E_PAR, "B3 barcode letter");
		return;
	}

	code = (uint8_t)(letter - (uint8_t)'A' + 1U);
	lock_error = tk_loc_mtx(mtx_b3_state, B3_STATE_LOCK_TIMEOUT_MS);
	if (lock_error == E_OK) {
		g_b3_state.last_barcode = code;
		g_b3_state.last_nav_command = NAV_NONE;
		unlock_error = tk_unl_mtx(mtx_b3_state);
		if (unlock_error != E_OK) {
			CHECK(unlock_error, "B3 barcode state unlock");
		}
	} else {
		CHECK(lock_error, "B3 barcode state lock");
	}

	send_error = tk_snd_mbf(mbf_nav_cmd, &code, (INT)sizeof(code), TMO_POL);
	if (send_error != E_OK) {
		CHECK(send_error, "B3 navigation queue send");
	}

	report_error = comm_report_barcode(letter, NAV_NONE);
	if (report_error != E_OK) {
		CHECK(report_error, "B3 barcode telemetry event");
	}
}

static void line_task(INT stacd, void *p_exinf)
{
	uint32_t diagnostic_ms = 0U;

	(void)stacd;
	(void)p_exinf;

	for (;;) {
		uint32_t left_raw = 0U;
		uint32_t centre_raw = 0U;
		uint8_t flags = 0U;
		uint8_t barcode_level;
		SYSTIM now;
		ER error;

		error = drv_ir_adc_read(&left_raw, &centre_raw);
		if (error != E_OK) {
			CHECK(error, "B3 ADC read");
			tk_dly_tsk(B3_LINE_UPDATE_MS);
			continue;
		}
		error = tk_get_tim(&now);
		if (error != E_OK) {
			CHECK(error, "B3 sample timestamp");
			tk_dly_tsk(B3_LINE_UPDATE_MS);
			continue;
		}

		if (B3_LINE_LEFT_ACTIVE(left_raw) ||
		    B3_LINE_CENTRE_ACTIVE(centre_raw)) {
			flags |= B3_LINE_F_DETECTED;
		} else {
			flags |= B3_LINE_F_LOST;
		}
		if (B3_LINE_LEFT_ACTIVE(left_raw) &&
		    B3_LINE_CENTRE_ACTIVE(centre_raw)) {
			flags |= B3_LINE_F_JUNCTION;
		}
		barcode_level = (uint8_t)barcode_do_level();

		error = b3_set_line_state((uint16_t)left_raw,
					  (uint16_t)centre_raw, barcode_level,
					  flags, now.lo);
		if (error != E_OK) {
			CHECK(error, "B3 line state update");
		}
		comm_set_line((uint16_t)left_raw, (uint16_t)centre_raw,
			      barcode_level, flags);

		diagnostic_ms += B3_LINE_UPDATE_MS;
		if (diagnostic_ms >= B3_LINE_DIAGNOSTIC_PERIOD_MS) {
			diagnostic_ms = 0U;
			tm_printf((UB *)"[B3] line left=%d centre=%d flags=0x%02x\n",
				  (INT)left_raw, (INT)centre_raw, (INT)flags);
		}
		tk_dly_tsk(B3_LINE_UPDATE_MS);
	}
}

static ER b3_set_line_state(uint16_t left, uint16_t centre, uint8_t right,
			    uint8_t flags, uint32_t sample_ms)
{
	ER error = tk_loc_mtx(mtx_b3_state, B3_STATE_LOCK_TIMEOUT_MS);

	if (error != E_OK) {
		return error;
	}
	g_b3_state.ir_left = left & 0x0FFFU;
	g_b3_state.ir_centre = centre & 0x0FFFU;
	g_b3_state.ir_right = (right != 0U) ? 1U : 0U;
	g_b3_state.line_flags = flags & 0x07U;
	g_line_sample_ms = sample_ms;
	g_line_sample_valid = true;
	error = tk_unl_mtx(mtx_b3_state);
	return error;
}
