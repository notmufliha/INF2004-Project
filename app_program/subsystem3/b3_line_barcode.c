/**
 * @file b3_line_barcode.c
 * @brief Buddy 3 line telemetry, edge capture, and navigation queue.
 */
#include "b3_line_barcode.h"
#include "b3_code39.h"
#include "b3_line_cal.h"

#include <stdbool.h>

#include "../gpio_irq_dispatch.h"
#include "../subsystem1/subsystem1.h"

#define B3_BARCODE_PIN (28U)
#define B3_ADC_LEFT_CHANNEL (0)
#define B3_ADC_CENTRE_CHANNEL (1)
#define B3_EDGE_QUEUE_LENGTH (64U)
#define B3_MBF_MSG_OVERHEAD (8U)
#define B3_STATE_LOCK_TIMEOUT_MS (5)
#define B3_BARCODE_TASK_POLL_MS (1U)
#ifndef B3_SIO_BASE
#define B3_SIO_BASE (0xD0000000UL)
#endif
#define B3_SIO_GPIO_IN (*(volatile uint32_t *)(B3_SIO_BASE + 0x004U))
#ifndef B3_TIMER_BASE
#define B3_TIMER_BASE (0x40054000UL)
#endif
#define B3_TIMER_RAWL (*(volatile uint32_t *)(B3_TIMER_BASE + 0x028U))

#ifndef B3_IO_BANK_BASE
#define B3_IO_BANK_BASE (0x40014000UL)
#endif
#define B3_IO_INTR3 (*(volatile uint32_t *)(B3_IO_BANK_BASE + 0x0FCU))
#define B3_IO_PROC0_INTE3 (*(volatile uint32_t *)(B3_IO_BANK_BASE + 0x10CU))
#define B3_IO_PROC0_INTS3 (*(volatile uint32_t *)(B3_IO_BANK_BASE + 0x12CU))
#define B3_BARCODE_EDGE_BITS (0x0CU << ((B3_BARCODE_PIN % 8U) * 4U))

typedef struct {
	uint32_t timestamp_us;
	uint8_t level;
} B3_EDGE;

static B3_STATE g_b3_state;
static uint32_t g_line_sample_ms;
static bool g_line_sample_valid;
static volatile B3_EDGE g_edge_queue[B3_EDGE_QUEUE_LENGTH];
static volatile uint8_t g_edge_head;
static volatile uint8_t g_edge_tail;
static volatile uint8_t g_edge_pending;
static volatile uint8_t g_edge_overflow;
/* Time of the latest GP28 edge, for "barcode in progress". */
static volatile uint32_t g_last_edge_us;
static volatile uint8_t g_edge_seen;

/* Route table: which decoded letter queues which junction action.
 * Every letter is decoded and logged on the serial console; only letters
 * listed here act.  The dashboard can display A, B, C, D and Z. */
static const struct {
	char letter;
	uint8_t barcode;
	uint8_t nav;
} b3_routes[] = {
	{ 'A', B3_BARCODE_A, B3_NAV_STRAIGHT },
	{ 'Z', B3_BARCODE_Z, B3_NAV_RIGHT },
};
static ID g_adc_device;
static uint8_t g_b3_started;

EXPORT ID mbf_nav_cmd;
EXPORT ID mtx_b3_state;

static void b3_gpio_irq_handler(UINT intno);
static ER b3_read_adc(uint8_t channel, uint16_t *value);
static void b3_set_line_state(uint16_t left, uint16_t centre, uint8_t right,
			      uint8_t flags);
static void b3_set_barcode_state(uint8_t barcode, uint8_t nav);
static bool b3_pop_edge(B3_EDGE *edge);
static uint8_t b3_take_overflow(void);
static void b3_process_edge(const B3_EDGE *edge,
			    B3_CODE39_DECODER *decoder);
static void b3_publish_barcode(uint8_t barcode, uint8_t nav);
static void b3_handle_symbol(char symbol, const B3_CODE39_DECODER *decoder);
static void b3_log_reject(const B3_CODE39_DECODER *decoder);
static void b3_delay(uint32_t delay_ms);

/**
 * @brief Create the ADC handle, navigation queue, and state mutex.
 * @return E_OK on success or the first μT-Kernel error.
 */
EXPORT ER b3_init(void)
{
	T_CMBF cmbf = {0};
	T_CMTX cmtx = {0};
	ER error;

	if (mbf_nav_cmd > 0 || mtx_b3_state > 0) {
		return E_OBJ;
	}

	g_adc_device = tk_opn_dev((CONST UB *)"adca", TD_READ);
	if (g_adc_device < E_OK) {
		return (ER)g_adc_device;
	}

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = (SZ)(B3_NAV_QUEUE_LENGTH *
			  (sizeof(uint8_t) + B3_MBF_MSG_OVERHEAD));
	cmbf.maxmsz = (INT)sizeof(uint8_t);
	mbf_nav_cmd = tk_cre_mbf(&cmbf);
	if (mbf_nav_cmd < E_OK) {
		error = tk_cls_dev(g_adc_device, 0);
		if (error < E_OK) {
			CHECK(error, "B3 ADC close after queue error");
		}
		g_adc_device = 0;
		return (ER)mbf_nav_cmd;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_b3_state = tk_cre_mtx(&cmtx);
	if (mtx_b3_state < E_OK) {
		error = (ER)mtx_b3_state;
		{
			ER cleanup_error = tk_del_mbf(mbf_nav_cmd);

			if (cleanup_error < E_OK) {
				CHECK(cleanup_error, "B3 queue cleanup");
			}
			cleanup_error = tk_cls_dev(g_adc_device, 0);
			if (cleanup_error < E_OK) {
				CHECK(cleanup_error, "B3 ADC cleanup");
			}
		}
		mbf_nav_cmd = 0;
		g_adc_device = 0;
		return error;
	}

	g_b3_state.ir_left = 0U;
	g_b3_state.ir_centre = 0U;
	g_b3_state.ir_right = 0U;
	g_b3_state.line_flags = B3_LINE_F_LOST;
	g_b3_state.last_barcode = B3_BARCODE_NONE;
	g_b3_state.last_nav_command = B3_NAV_NONE;
	g_edge_head = 0U;
	g_edge_tail = 0U;
	g_edge_pending = 0U;
	g_edge_overflow = 0U;
	g_b3_started = 0U;
	g_line_sample_valid = false;

	return E_OK;
}

/**
 * @brief Configure GP28 and start the line-sampling and barcode tasks.
 * @return E_OK on success or an initialization/task-start error.
 */
EXPORT ER b3_start(void)
{
	ER error;
	ID task_id;
	const uint32_t pin_mask = (uint32_t)1U << B3_BARCODE_PIN;

	if ((mbf_nav_cmd <= 0) || (mtx_b3_state <= 0) ||
	    (g_adc_device < E_OK)) {
		return E_OBJ;
	}
	if (g_b3_started != 0U) {
		return E_OBJ;
	}
#if TK_SUPPORT_SMP
	/* Single-core builds (SMP=0) have no tk_get_prc(): only core 0 runs. */
	if (tk_get_prc() != 1) {
		return E_CTX;
	}
#endif

	error = gpio_set_pin(B3_BARCODE_PIN, GPIO_MODE_IN);
	if (error != E_OK) {
		return error;
	}

	B3_IO_PROC0_INTE3 &= ~B3_BARCODE_EDGE_BITS;
	B3_IO_INTR3 = B3_BARCODE_EDGE_BITS;
	error = app_gpio_irq_register((UW)pin_mask, b3_gpio_irq_handler);
	if (error != E_OK) {
		return error;
	}
	B3_IO_PROC0_INTE3 |= B3_BARCODE_EDGE_BITS;

	task_id = app_start_task(b3_barcode_task, 17, TP_PRC1,
				 "b3_barcode_task");
	if (task_id < E_OK) {
		return (ER)task_id;
	}

	task_id = app_start_task(b3_line_task, 16, TP_PRC1, "b3_line_task");
	if (task_id < E_OK) {
		return (ER)task_id;
	}

	g_b3_started = 1U;
	return E_OK;
}

/**
 * @brief Copy the packed B3 state while holding its mutex.
 * @param state Destination for the snapshot.
 * @return E_OK on success, E_PAR for a null pointer, otherwise a kernel error.
 */
EXPORT ER b3_get_state(B3_STATE *state)
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

EXPORT ER b3_get_timed_state(B3_STATE *state, uint32_t *sample_ms)
{
    ER error;
    if (state == NULL || sample_ms == NULL) return E_PAR;
    if (mtx_b3_state <= 0) return E_OBJ;
    error = tk_loc_mtx(mtx_b3_state, B3_STATE_LOCK_TIMEOUT_MS);
    if (error != E_OK) return error;
    *state = g_b3_state;
    *sample_ms = g_line_sample_ms;
    error = g_line_sample_valid ? E_OK : E_OBJ;
    {
        ER unlock_error = tk_unl_mtx(mtx_b3_state);
        if (unlock_error != E_OK) return unlock_error;
    }
    return error;
}

static void b3_invalidate_line(void)
{
    if (tk_loc_mtx(mtx_b3_state, B3_STATE_LOCK_TIMEOUT_MS) == E_OK) {
        g_line_sample_valid = false;
        (void)tk_unl_mtx(mtx_b3_state);
    }
}

/**
 * @brief Sample the GP26/GP27 ADC channels and publish line telemetry.
 * @param stacd μT-Kernel task start code (unused).
 * @param exinf μT-Kernel extended information (unused).
 */
static volatile bool g_b3_recal_request;

EXPORT void b3_request_recalibration(void)
{
	g_b3_recal_request = true;
}

EXPORT void b3_line_task(INT stacd, void *exinf)
{
	uint16_t left;
	uint16_t centre;
	uint16_t left_norm;
	uint16_t centre_norm;
	uint8_t right;
	uint8_t flags;
	ER error;
	B3_LINE_CAL left_cal;
	B3_LINE_CAL centre_cal;
	bool calibration_reported = false;

	(void)stacd;
	(void)exinf;
	b3_line_cal_init(&left_cal);
	b3_line_cal_init(&centre_cal);

	for (;;) {
		error = b3_read_adc(B3_ADC_LEFT_CHANNEL, &left);
		if (error != E_OK) {
			CHECK(error, "B3 GP26 ADC read");
            b3_invalidate_line();
			b3_delay(B3_LINE_UPDATE_MS);
			continue;
		}

		error = b3_read_adc(B3_ADC_CENTRE_CHANNEL, &centre);
		if (error != E_OK) {
			CHECK(error, "B3 GP27 ADC read");
            b3_invalidate_line();
			b3_delay(B3_LINE_UPDATE_MS);
			continue;
		}

		right = (uint8_t)((B3_SIO_GPIO_IN >> B3_BARCODE_PIN) & 1U);
		if (g_b3_recal_request) {
			g_b3_recal_request = false;
			b3_line_cal_init(&left_cal);
			b3_line_cal_init(&centre_cal);
			calibration_reported = false;
			tm_printf((UB *)"[B3-CAL] restarted on START\n");
		}
		left_norm = b3_line_cal_update(&left_cal, left);
		centre_norm = b3_line_cal_update(&centre_cal, centre);
		flags = 0U;
		if (B3_LINE_LEFT_ACTIVE(left_norm) ||
		    B3_LINE_CENTRE_ACTIVE(centre_norm)) {
			flags |= B3_LINE_F_DETECTED;
		} else {
			flags |= B3_LINE_F_LOST;
		}
		if (B3_LINE_IS_JUNCTION(left_norm, centre_norm)) {
			flags |= B3_LINE_F_JUNCTION;
		}

		/* Controller gets normalised values; dashboard gets raw ADC. */
		b3_set_line_state(left_norm, centre_norm, right, flags);
		comm_set_line((UH)left, (UH)centre, (UB)right, (UB)flags);
		if (!calibration_reported && b3_line_cal_ready(&left_cal) &&
		    b3_line_cal_ready(&centre_cal)) {
			calibration_reported = true;
			tm_printf((UB *)"[B3-CAL] learned GP26 white=%d black=%d; GP27 white=%d black=%d\n",
				  (int)left_cal.white, (int)left_cal.black,
				  (int)centre_cal.white, (int)centre_cal.black);
		}
		b3_delay(B3_LINE_UPDATE_MS);
	}
}

/**
 * @brief Drain GP28 edges and decode Code 39 symbols outside the ISR.
 * @param stacd μT-Kernel task start code (unused).
 * @param exinf μT-Kernel extended information (unused).
 */
EXPORT void b3_barcode_task(INT stacd, void *exinf)
{
	B3_EDGE edge;
	B3_CODE39_DECODER decoder = {0};

	(void)stacd;
	(void)exinf;

	for (;;) {
		char symbol;

		if (g_edge_pending != 0U) {
			if (b3_take_overflow() != 0U) {
				b3_code39_reset(&decoder);
				tm_printf((UB *)"[B3] GP28 edge queue overflow; barcode discarded\n");
			}
			while (b3_pop_edge(&edge)) {
				b3_process_edge(&edge, &decoder);
			}
		}
		/* Confirm the final element once it has outlasted the chatter
		 * filter, so the last bar decodes without another edge. */
		if (b3_code39_poll(&decoder, B3_TIMER_RAWL, &symbol)) {
			b3_handle_symbol(symbol, &decoder);
		}
		b3_log_reject(&decoder);
		b3_delay(B3_BARCODE_TASK_POLL_MS);
	}
}

static ER b3_read_adc(uint8_t channel, uint16_t *value)
{
	uint32_t adc_word = 0U;
	SZ actual_size = 0;
	ER result;

	if (value == NULL) {
		return E_PAR;
	}

	result = tk_srea_dev(g_adc_device, (W)channel, &adc_word, 1U,
			     &actual_size);
	if (result != E_OK) {
		return result;
	}
	if (actual_size != 1 || adc_word > 4095U) {
		return E_IO;
	}

	*value = (uint16_t)(adc_word & 0x0FFFU);
	return E_OK;
}

static void b3_set_line_state(uint16_t left, uint16_t centre, uint8_t right,
			      uint8_t flags)
{
	ER error = tk_loc_mtx(mtx_b3_state, B3_STATE_LOCK_TIMEOUT_MS);

	if (error != E_OK) {
		CHECK(error, "B3 state lock");
		return;
	}

	g_b3_state.ir_left = left & 0x0FFFU;
	g_b3_state.ir_centre = centre & 0x0FFFU;
	g_b3_state.ir_right = (right != 0U) ? 1U : 0U;
	g_b3_state.line_flags = flags & 0x07U;
    {
        SYSTIM now;
        g_line_sample_valid = tk_get_tim(&now) == E_OK;
        if (g_line_sample_valid) g_line_sample_ms = now.lo;
    }
	error = tk_unl_mtx(mtx_b3_state);
	if (error != E_OK) {
		CHECK(error, "B3 state unlock");
	}
}

static void b3_set_barcode_state(uint8_t barcode, uint8_t nav)
{
	ER error = tk_loc_mtx(mtx_b3_state, B3_STATE_LOCK_TIMEOUT_MS);

	if (error != E_OK) {
		CHECK(error, "B3 state lock");
		return;
	}

	g_b3_state.last_barcode = barcode & 0x07U;
	g_b3_state.last_nav_command = nav & 0x07U;
	error = tk_unl_mtx(mtx_b3_state);
	if (error != E_OK) {
		CHECK(error, "B3 state unlock");
	}
}

static void b3_gpio_irq_handler(UINT intno)
{
	uint32_t pending_edges = B3_IO_PROC0_INTS3 & B3_BARCODE_EDGE_BITS;
	uint8_t next_head;

	(void)intno;
	if (pending_edges == 0U) {
		return;
	}

	/* Both rise and fall pending means the pin bounced since the last
	 * interrupt.  Record only the level it settled at: the decoder ignores
	 * a repeated level, so the bounce merges into the surrounding bar or
	 * space instead of discarding the whole barcode. */
	next_head = (uint8_t)((g_edge_head + 1U) &
			      (B3_EDGE_QUEUE_LENGTH - 1U));
	if (next_head == g_edge_tail) {
		g_edge_overflow = 1U;
	} else {
		g_edge_queue[g_edge_head].timestamp_us = B3_TIMER_RAWL;
		g_last_edge_us = g_edge_queue[g_edge_head].timestamp_us;
		g_edge_seen = 1U;
		g_edge_queue[g_edge_head].level = (uint8_t)
			((B3_SIO_GPIO_IN >> B3_BARCODE_PIN) & 1U);
		g_edge_head = next_head;
	}

	g_edge_pending = 1U;
	B3_IO_INTR3 = pending_edges;
}

static bool b3_pop_edge(B3_EDGE *edge)
{
	UINT interrupt_status;
	bool available = false;

	DI(interrupt_status);
	if (g_edge_tail != g_edge_head) {
		edge->timestamp_us =
			g_edge_queue[g_edge_tail].timestamp_us;
		edge->level = g_edge_queue[g_edge_tail].level;
		g_edge_tail = (uint8_t)((g_edge_tail + 1U) &
					(B3_EDGE_QUEUE_LENGTH - 1U));
		available = true;
	} else {
		g_edge_pending = 0U;
	}
	EI(interrupt_status);
	return available;
}

static uint8_t b3_take_overflow(void)
{
	UINT interrupt_status;
	uint8_t overflow;

	DI(interrupt_status);
	overflow = g_edge_overflow;
	g_edge_overflow = 0U;
	EI(interrupt_status);
	return overflow;
}

static void b3_process_edge(const B3_EDGE *edge,
			    B3_CODE39_DECODER *decoder)
{
	char symbol;

	if (b3_code39_feed(decoder, edge->level, edge->timestamp_us, &symbol)) {
		b3_handle_symbol(symbol, decoder);
	}
}

static void b3_handle_symbol(char symbol, const B3_CODE39_DECODER *decoder)
{
	static const char *const nav_names[] = {
		"NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"
	};
	size_t index;

	for (index = 0U; index < sizeof(b3_routes) / sizeof(b3_routes[0]);
	     ++index) {
		if (b3_routes[index].letter == symbol) {
			tm_printf((UB *)"[B3] barcode *%c* read %s -> %s at next junction\n",
				  symbol, decoder->last_reversed != 0U ?
				  "(reversed)" : "(forward)",
				  nav_names[b3_routes[index].nav]);
			b3_publish_barcode(b3_routes[index].barcode,
					   b3_routes[index].nav);
			return;
		}
	}
	tm_printf((UB *)"[B3] barcode *%c* read %s; not in route table, ignored\n",
		  symbol, decoder->last_reversed != 0U ? "(reversed)" : "(forward)");
}

static void b3_log_reject(const B3_CODE39_DECODER *decoder)
{
	static const char *const reasons[] = {
		"none",
		"bars not clearly wide/narrow (dirty strip, speed change or sensor threshold)",
		"not a Code 39 character",
		"gap between characters wrong",
		"stop character missing"
	};
	static uint16_t logged = 0U;

	if (decoder->reject_count != logged) {
		logged = decoder->reject_count;
		tm_printf((UB *)"[B3] barcode after start '*' rejected: %s (payload '%c')\n",
			  decoder->last_reject < 5U ?
			  reasons[decoder->last_reject] : "?",
			  decoder->last_reject_char != '\0' ?
			  decoder->last_reject_char : '?');
	}
}

EXPORT bool b3_barcode_active(void)
{
	return g_edge_seen != 0U &&
		(uint32_t)(B3_TIMER_RAWL - g_last_edge_us) < B3_BARCODE_ACTIVE_US;
}

static void b3_publish_barcode(uint8_t barcode, uint8_t nav)
{
	uint8_t command = nav;
	UB barcode_character = barcode == B3_BARCODE_Z ?
		(UB)'Z' : (UB)('A' + barcode - 1U);
	ER error;

	b3_set_barcode_state(barcode, nav);
	error = tk_snd_mbf(mbf_nav_cmd, &command, (INT)sizeof(command), TMO_POL);
	if (error != E_OK) {
		CHECK(error, "B3 navigation queue send");
	}

	error = comm_report_barcode(barcode_character, (NAV_CMD)nav);
	if (error != E_OK) {
		CHECK(error, "B3 barcode telemetry report");
	}
}

static void b3_delay(uint32_t delay_ms)
{
	ER error = tk_dly_tsk((RELTIM)delay_ms);

	if (error < E_OK) {
		CHECK(error, "B3 task delay");
	}
}
