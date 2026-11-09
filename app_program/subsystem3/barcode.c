/**
 * @file barcode.c
 * @brief GP28 edge capture and Code 39 barcode delivery to the Vehicle Controller.
 */
#include "barcode.h"

#include <stdbool.h>

#include "../gpio_irq_dispatch.h"
#include "../subsystem1/subsystem1.h"
#include "b3_line_barcode.h"
#include "cfg_barcode.h"
#include "code39.h"

#define BARCODE_PIN (28U)
#define BARCODE_EDGE_QUEUE_SIZE (64U)
#define BARCODE_EDGE_QUEUE_MASK (BARCODE_EDGE_QUEUE_SIZE - 1U)
#define BARCODE_QUIET_TIMEOUT_US (250000U)
#define BARCODE_TASK_PERIOD_MS (1U)

#define BARCODE_SIO_BASE (0xD0000000UL)
#define BARCODE_SIO_GPIO_IN (*(volatile uint32_t *)(BARCODE_SIO_BASE + 0x004U))
#define BARCODE_TIMER_BASE (0x40054000UL)
#define BARCODE_TIMER_RAWL (*(volatile uint32_t *)(BARCODE_TIMER_BASE + 0x028U))
#define BARCODE_IO_BANK_BASE (0x40014000UL)
#define BARCODE_IO_INTR3 (*(volatile uint32_t *)(BARCODE_IO_BANK_BASE + 0x0FCU))
#define BARCODE_IO_PROC0_INTE3 (*(volatile uint32_t *)(BARCODE_IO_BANK_BASE + 0x10CU))
#define BARCODE_IO_PROC0_INTS3 (*(volatile uint32_t *)(BARCODE_IO_BANK_BASE + 0x12CU))
#define BARCODE_EDGE_BITS (0x0CU << ((BARCODE_PIN % 8U) * 4U))

typedef struct {
	uint32_t timestamp_us;
	uint8_t level;
} barcode_edge_t;

static barcode_edge_t g_edge_queue[BARCODE_EDGE_QUEUE_SIZE];
static volatile uint8_t g_edge_head;
static volatile uint8_t g_edge_tail;
static volatile uint8_t g_edge_overflow;
static volatile uint32_t g_edge_count;
static uint32_t g_last_irq_us;
static bool g_last_irq_valid;
static uint8_t g_barcode_started;
static uint8_t g_barcode_irq_registered;

static void barcode_task(INT stacd, void *p_exinf);
static void barcode_gpio_irq_handler(UINT intno);
static bool barcode_pop_edge(barcode_edge_t *edge);
static void barcode_process_edge(const barcode_edge_t *edge,
				 bool *active, uint32_t *last_us,
				 uint8_t *count, uint32_t *widths);
static void barcode_decode_capture(const uint32_t *widths, uint8_t count);
static void barcode_start_capture(const barcode_edge_t *edge,
				  bool *active, uint32_t *last_us,
				  uint8_t *count);

ER barcode_init(void)
{
	g_edge_head = 0U;
	g_edge_tail = 0U;
	g_edge_overflow = 0U;
	g_edge_count = 0U;
	g_last_irq_us = 0U;
	g_last_irq_valid = false;
	g_barcode_started = 0U;
	g_barcode_irq_registered = 0U;
	return E_OK;
}

ER barcode_start(void)
{
	ER error;
	ID task_id;
	const UW pin_mask = (UW)1U << BARCODE_PIN;

	if (mbf_nav_cmd <= 0 || g_barcode_started != 0U) {
		return E_OBJ;
	}
#if TK_SUPPORT_SMP
	if (tk_get_prc() != 1) {
		return E_CTX;
	}
#endif

	error = gpio_set_pin(BARCODE_PIN, GPIO_MODE_IN);
	if (error != E_OK) {
		return error;
	}
	BARCODE_IO_PROC0_INTE3 &= ~BARCODE_EDGE_BITS;
	BARCODE_IO_INTR3 = BARCODE_EDGE_BITS;
	if (g_barcode_irq_registered == 0U) {
		error = app_gpio_irq_register(pin_mask, barcode_gpio_irq_handler);
		if (error != E_OK) {
			return error;
		}
		g_barcode_irq_registered = 1U;
	}

	task_id = app_start_task((FP)barcode_task, 17, TP_PRC1,
				 "b3_barcode_task");
	if (task_id < E_OK) {
		return (ER)task_id;
	}
	BARCODE_IO_PROC0_INTE3 |= BARCODE_EDGE_BITS;
	g_barcode_started = 1U;
	return E_OK;
}

uint32_t barcode_edge_count(void)
{
	return g_edge_count;
}

uint32_t barcode_do_level(void)
{
	bool pin_high = ((BARCODE_SIO_GPIO_IN >> BARCODE_PIN) & 1U) != 0U;
	return (BAR_DO_BLACK_IS_HIGH == pin_high) ? 1U : 0U;
}

static void barcode_task(INT stacd, void *p_exinf)
{
	bool active = false;
	uint32_t last_us = 0U;
	uint32_t widths[BAR_ELEMENTS] = {0U};
	uint8_t count = 0U;

	(void)stacd;
	(void)p_exinf;
	for (;;) {
		barcode_edge_t edge;

		if (g_edge_overflow != 0U) {
			g_edge_overflow = 0U;
			active = false;
			count = 0U;
			tm_printf((UB *)"[B3] barcode edge queue overflow; capture discarded\n");
		}
		while (barcode_pop_edge(&edge)) {
			barcode_process_edge(&edge, &active, &last_us, &count,
					     widths);
		}
		if (active &&
		    ((uint32_t)BARCODE_TIMER_RAWL - last_us) >
			BARCODE_QUIET_TIMEOUT_US) {
			active = false;
			count = 0U;
		}
		tk_dly_tsk(BARCODE_TASK_PERIOD_MS);
	}
}

static void barcode_gpio_irq_handler(UINT intno)
{
	uint32_t pending = BARCODE_IO_PROC0_INTS3 & BARCODE_EDGE_BITS;
	uint32_t now_us;
	bool pin_high;
	uint8_t head;
	uint8_t next;

	(void)intno;
	if (pending == 0U) {
		return;
	}
	now_us = (uint32_t)BARCODE_TIMER_RAWL;
	pin_high = ((BARCODE_SIO_GPIO_IN >> BARCODE_PIN) & 1U) != 0U;
	BARCODE_IO_INTR3 = pending;

	if (g_last_irq_valid &&
	    ((uint32_t)(now_us - g_last_irq_us) < BAR_EDGE_REJECT_US)) {
		return;
	}
	g_last_irq_us = now_us;
	g_last_irq_valid = true;
	g_edge_count++;

	head = g_edge_head;
	next = (uint8_t)(((uint32_t)head + 1U) & BARCODE_EDGE_QUEUE_MASK);
	if (next == g_edge_tail) {
		g_edge_overflow = 1U;
		return;
	}
	g_edge_queue[head].timestamp_us = now_us;
	g_edge_queue[head].level =
		(uint8_t)((BAR_DO_BLACK_IS_HIGH == pin_high) ? 1U : 0U);
	g_edge_head = next;
}

static bool barcode_pop_edge(barcode_edge_t *edge)
{
	uint8_t tail;

	if (edge == NULL || g_edge_tail == g_edge_head) {
		return false;
	}
	tail = g_edge_tail;
	edge->timestamp_us = g_edge_queue[tail].timestamp_us;
	edge->level = g_edge_queue[tail].level;
	g_edge_tail =
		(uint8_t)(((uint32_t)tail + 1U) & BARCODE_EDGE_QUEUE_MASK);
	return true;
}

static void barcode_process_edge(const barcode_edge_t *edge, bool *active,
				 uint32_t *last_us, uint8_t *count,
				 uint32_t *widths)
{
	uint32_t duration;

	if (!*active) {
		if (edge->level == 1U) {
			barcode_start_capture(edge, active, last_us, count);
		}
		return;
	}

	duration = edge->timestamp_us - *last_us;
	if (duration > BARCODE_QUIET_TIMEOUT_US) {
		*active = false;
		*count = 0U;
		if (edge->level == 1U) {
			barcode_start_capture(edge, active, last_us, count);
		}
		return;
	}

	if (*count >= BAR_ELEMENTS) {
		*active = false;
		*count = 0U;
		return;
	}
	widths[*count] = duration;
	(*count)++;
	*last_us = edge->timestamp_us;
	if (*count == BAR_ELEMENTS) {
		barcode_decode_capture(widths, *count);
		*active = false;
		*count = 0U;
	}
}

static void barcode_decode_capture(const uint32_t *widths, uint8_t count)
{
	uint8_t scaled[BAR_ELEMENTS];
	uint32_t minimum = UINT32_MAX;
	uint32_t char_index = 0U;
	uint32_t index;
	uint32_t result;

	if (count != BAR_ELEMENTS) {
		return;
	}
	for (index = 0U; index < count; index++) {
		if (widths[index] < minimum) {
			minimum = widths[index];
		}
	}
	if (minimum == 0U || minimum == UINT32_MAX) {
		return;
	}
	for (index = 0U; index < count; index++) {
		uint64_t scaled_width = ((uint64_t)widths[index] * 100U) / minimum;
		if (scaled_width > UINT8_MAX) {
			scaled_width = UINT8_MAX;
		}
		scaled[index] = (uint8_t)scaled_width;
	}

	result = code39_decode(scaled, BAR_ELEMENTS, &char_index);
	if (result == CODE39_OK) {
		uint32_t ascii = code39_char_ascii(char_index);
		if (ascii >= (uint32_t)'A' && ascii <= (uint32_t)'Z') {
			b3_publish_barcode((uint8_t)ascii);
		}
	}
}

static void barcode_start_capture(const barcode_edge_t *edge, bool *active,
				  uint32_t *last_us, uint8_t *count)
{
	*active = true;
	*last_us = edge->timestamp_us;
	*count = 0U;
}
