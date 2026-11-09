/**
 * @file drv_ir_adc.c
 * @brief RP2040 ADC device adapter for the GP26/GP27 line sensors.
 */
#include "drv_ir_adc.h"

#define DRV_IR_ADC_SAMPLES (4U)

static ID g_adc_device;

ER drv_ir_adc_init(void)
{
	ID device;

	if (g_adc_device > 0) {
		return E_OBJ;
	}
	device = tk_opn_dev((CONST UB *)"adca", TD_READ);
	if (device < E_OK) {
		return (ER)device;
	}
	g_adc_device = device;
	return E_OK;
}

ER drv_ir_adc_close(void)
{
	ER error;

	if (g_adc_device <= 0) {
		return E_OBJ;
	}
	error = tk_cls_dev(g_adc_device, 0);
	if (error == E_OK) {
		g_adc_device = 0;
	}
	return error;
}

ER drv_ir_adc_read(uint32_t *p_left, uint32_t *p_right)
{
	uint32_t *outputs[2] = {p_left, p_right};
	uint32_t channel;

	if (p_left == NULL || p_right == NULL) {
		return E_PAR;
	}
	if (g_adc_device <= 0) {
		return E_OBJ;
	}

	for (channel = 0U; channel < 2U; channel++) {
		uint32_t sum = 0U;
		uint32_t sample;

		for (sample = 0U; sample < DRV_IR_ADC_SAMPLES; sample++) {
			uint32_t word = 0U;
			SZ actual_size = 0;
			ER error = tk_srea_dev(g_adc_device, (W)channel, &word, 1U,
					       &actual_size);
			if (error != E_OK) {
				return error;
			}
			if (actual_size != 1) {
				return E_IO;
			}
			sum += word & 0x0FFFU;
		}
		*outputs[channel] = sum / DRV_IR_ADC_SAMPLES;
	}
	return E_OK;
}
