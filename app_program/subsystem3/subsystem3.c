#include "subsystem3.h"

#include "barcode.h"
#include "line.h"
#include "test_barcode.h"
#include "test_line.h"

ER b3_init(void)
{
	ER error = line_init();

	if (error == E_OK) {
		error = barcode_init();
	}
	return error;
}

ER b3_start(void)
{
	ER error = line_start();

	if (error == E_OK) {
		error = barcode_start();
	}
	return error;
}

ER subsystem3_init(void)
{
	return b3_init();
}

ER subsystem3_start(void)
{
	return b3_start();
}

void subsystem3_test(void)
{
	test_line_run();
	test_barcode_run();
}
