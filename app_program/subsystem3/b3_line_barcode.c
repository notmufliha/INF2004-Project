/**
 * @file b3_line_barcode.c
 * @brief Buddy 3 line telemetry, edge capture, and navigation queue.
 */
#include "b3_line_barcode.h"

#include <stdbool.h>

#include "../gpio_irq_dispatch.h"
#include "../subsystem1/subsystem1.h"

#define B3_BARCODE_PIN (28U)
#define B3_ADC_LEFT_CHANNEL (0)
#define B3_ADC_CENTRE_CHANNEL (1)
/* WARNING: Calibrate this ADC threshold against the installed sensors/track. */
#define B3_LINE_THRESHOLD (2048U)
#define B3_EDGE_QUEUE_LENGTH (16U)
#define B3_MBF_MSG_OVERHEAD (8U)
#define B3_STATE_LOCK_TIMEOUT_MS (5)
#define B3_BARCODE_TASK_POLL_MS (1U)
#ifndef B3_BARCODE_PROTOCOL_CONFIGURED
#define B3_BARCODE_PROTOCOL_CONFIGURED (0)
#endif

/*
 * The barcode note in app_program/subsystem3/README.md does not specify a
 * pulse-width protocol. Set B3_BARCODE_PROTOCOL_CONFIGURED to 1 and define
 * the four upper bounds only after measuring the real track's A-D symbols.
 */
#if B3_BARCODE_PROTOCOL_CONFIGURED
#if !defined(B3_BARCODE_A_MAX_US) || !defined(B3_BARCODE_B_MAX_US) || \
	!defined(B3_BARCODE_C_MAX_US) || !defined(B3_BARCODE_D_MAX_US)
#error "Define measured A-D pulse-width upper bounds before enabling decoding"
#endif
#if (B3_BARCODE_A_MAX_US == 0U) || \
	(B3_BARCODE_A_MAX_US >= B3_BARCODE_B_MAX_US) || \
	(B3_BARCODE_B_MAX_US >= B3_BARCODE_C_MAX_US) || \
	(B3_BARCODE_C_MAX_US >= B3_BARCODE_D_MAX_US)
#error "Barcode pulse-width bounds must be positive and strictly increasing"
#endif
#endif

#define B3_SIO_BASE (0xD0000000UL)
#define B3_SIO_GPIO_IN (*(volatile uint32_t *)(B3_SIO_BASE + 0x004U))
#define B3_TIMER_BASE (0x40054000UL)
#define B3_TIMER_RAWL (*(volatile uint32_t *)(B3_TIMER_BASE + 0x028U))

#define B3_IO_BANK_BASE (0x40014000UL)
#define B3_IO_INTR3 (*(volatile uint32_t *)(B3_IO_BANK_BASE + 0x0FCU))
#define B3_IO_PROC0_INTE3 (*(volatile uint32_t *)(B3_IO_BANK_BASE + 0x10CU))
#define B3_IO_PROC0_INTS3 (*(volatile uint32_t *)(B3_IO_BANK_BASE + 0x12CU))
#define B3_BARCODE_EDGE_BITS (0x0CU << ((B3_BARCODE_PIN % 8U) * 4U))

typedef struct {
	uint32_t timestamp_us;
	uint8_t level;
} B3_EDGE;

static B3_STATE g_b3_state;
static volatile B3_EDGE g_edge_queue[B3_EDGE_QUEUE_LENGTH];
static volatile uint8_t g_edge_head;
static volatile uint8_t g_edge_tail;
static volatile uint8_t g_edge_pending;
static volatile uint8_t g_edge_overflow;
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
static bool b3_decode_pulse_width(uint32_t width_us, uint8_t *barcode,
				  uint8_t *nav);
static void b3_process_edge(const B3_EDGE *edge, uint8_t *pulse_active,
			    uint32_t *pulse_start_us);
static void b3_publish_barcode(uint8_t barcode, uint8_t nav);
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
	if (tk_get_prc() != 1) {
		return E_CTX;
	}

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

/**
 * @brief Sample the GP26/GP27 ADC channels and publish line telemetry.
 * @param stacd μT-Kernel task start code (unused).
 * @param exinf μT-Kernel extended information (unused).
 */
EXPORT void b3_line_task(INT stacd, void *exinf)
{
	uint16_t left;
	uint16_t centre;
	uint8_t right;
	uint8_t flags;
	ER error;

	(void)stacd;
	(void)exinf;

	for (;;) {
		error = b3_read_adc(B3_ADC_LEFT_CHANNEL, &left);
		if (error != E_OK) {
			CHECK(error, "B3 GP26 ADC read");
			b3_delay(B3_LINE_UPDATE_MS);
			continue;
		}

		error = b3_read_adc(B3_ADC_CENTRE_CHANNEL, &centre);
		if (error != E_OK) {
			CHECK(error, "B3 GP27 ADC read");
			b3_delay(B3_LINE_UPDATE_MS);
			continue;
		}

		right = (uint8_t)((B3_SIO_GPIO_IN >> B3_BARCODE_PIN) & 1U);
		flags = 0U;
		if ((left <= B3_LINE_THRESHOLD) ||
		    (centre <= B3_LINE_THRESHOLD)) {
			flags |= B3_LINE_F_DETECTED;
		} else {
			flags |= B3_LINE_F_LOST;
		}
		if ((left <= B3_LINE_THRESHOLD) &&
		    (centre <= B3_LINE_THRESHOLD)) {
			flags |= B3_LINE_F_JUNCTION;
		}

		b3_set_line_state(left, centre, right, flags);
		comm_set_line((UH)left, (UH)centre, (UB)right, (UB)flags);
		b3_delay(B3_LINE_UPDATE_MS);
	}
}

/**
 * @brief Drain timestamped GP28 edges and decode pulse widths outside the ISR.
 * @param stacd μT-Kernel task start code (unused).
 * @param exinf μT-Kernel extended information (unused).
 */
EXPORT void b3_barcode_task(INT stacd, void *exinf)
{
	B3_EDGE edge;
	uint8_t pulse_active = 0U;
	uint32_t pulse_start_us = 0U;

	(void)stacd;
	(void)exinf;

	for (;;) {
		if (g_edge_pending == 0U) {
			b3_delay(B3_BARCODE_TASK_POLL_MS);
			continue;
		}
		if (b3_take_overflow() != 0U) {
			pulse_active = 0U;
			tm_printf((UB *)"[B3] GP28 edge queue overflow; pulse discarded\n");
		}
		while (b3_pop_edge(&edge)) {
			b3_process_edge(&edge, &pulse_active, &pulse_start_us);
		}
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
	if (actual_size != 1) {
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

	if ((pending_edges & (pending_edges - 1U)) != 0U) {
		g_edge_overflow = 1U;
	} else {
		next_head = (uint8_t)((g_edge_head + 1U) &
				      (B3_EDGE_QUEUE_LENGTH - 1U));
		if (next_head == g_edge_tail) {
			g_edge_overflow = 1U;
		} else {
			g_edge_queue[g_edge_head].timestamp_us = B3_TIMER_RAWL;
			g_edge_queue[g_edge_head].level = (uint8_t)
				((B3_SIO_GPIO_IN >> B3_BARCODE_PIN) & 1U);
			g_edge_head = next_head;
		}
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

static bool b3_decode_pulse_width(uint32_t width_us, uint8_t *barcode,
				  uint8_t *nav)
{
	if ((barcode == NULL) || (nav == NULL) || (width_us == 0U)) {
		return false;
	}

#if B3_BARCODE_PROTOCOL_CONFIGURED
	if (width_us <= B3_BARCODE_A_MAX_US) {
		*barcode = B3_BARCODE_A;
		*nav = B3_NAV_LEFT;
	} else if (width_us <= B3_BARCODE_B_MAX_US) {
		*barcode = B3_BARCODE_B;
		*nav = B3_NAV_RIGHT;
	} else if (width_us <= B3_BARCODE_C_MAX_US) {
		*barcode = B3_BARCODE_C;
		*nav = B3_NAV_STRAIGHT;
	} else if (width_us <= B3_BARCODE_D_MAX_US) {
		*barcode = B3_BARCODE_D;
		*nav = B3_NAV_UTURN;
	} else {
		return false;
	}
	return true;
#else
	(void)width_us;
	(void)barcode;
	(void)nav;
	return false;
#endif
}

static void b3_process_edge(const B3_EDGE *edge, uint8_t *pulse_active,
			    uint32_t *pulse_start_us)
{
	uint32_t pulse_width_us;
	uint8_t barcode;
	uint8_t nav;

	/* NOTE: This assumes an active-low barcode pulse on GP28. */
	if (edge->level == 0U) {
		if (*pulse_active == 0U) {
			*pulse_start_us = edge->timestamp_us;
			*pulse_active = 1U;
		}
		return;
	}

	if (*pulse_active == 0U) {
		return;
	}

	pulse_width_us = edge->timestamp_us - *pulse_start_us;
	*pulse_active = 0U;
	if (b3_decode_pulse_width(pulse_width_us, &barcode, &nav)) {
		b3_publish_barcode(barcode, nav);
	}
}

static void b3_publish_barcode(uint8_t barcode, uint8_t nav)
{
	uint8_t command = nav;
	ER error;

	b3_set_barcode_state(barcode, nav);
	error = tk_snd_mbf(mbf_nav_cmd, &command, (INT)sizeof(command), TMO_POL);
	if (error != E_OK) {
		CHECK(error, "B3 navigation queue send");
	}

	error = comm_report_barcode((UB)('A' + barcode - 1U), (NAV_CMD)nav);
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
