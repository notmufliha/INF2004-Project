#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

ER subsystem3_init(void);
ER subsystem3_start(void);
#if APP_TEST
void subsystem3_test(void);
#endif

#endif
