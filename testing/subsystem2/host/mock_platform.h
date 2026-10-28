#ifndef S2_MOCK_PLATFORM_H
#define S2_MOCK_PLATFORM_H
/* Host stand-ins for RTOS services; production C files are compiled unchanged. */
#define APP_H
#define SUBSYSTEM2_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
typedef int32_t INT;
typedef int32_t ER;
typedef int32_t ID;
typedef uint32_t UINT;
typedef uint32_t UW;
typedef uint8_t UB;
typedef void (*FP)(void);
#define TP_PRC2 2u
ID app_start_task(FP entry, INT priority, UW core, const char *name);
typedef struct { UINT hi; UINT lo; } SYSTIM;
typedef struct { UINT sematr; INT isemcnt; INT maxsem; } T_CSEM;
#define TK_SUPPORT_SMP 0
#define E_OK 0
#define E_SYS (-5)
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define TA_TFIFO 0
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TSK_SELF 0
#define DI(status) ((status) = 0u)
#define EI(status) ((void)(status))
ER tk_get_tim(SYSTIM *time);
ID tk_cre_sem(const T_CSEM *sem);
ER tk_wai_sem(ID id, INT count, INT timeout);
ER tk_sig_sem(ID id, INT count);
ER tk_wup_tsk(ID id);
ER tk_can_wup(ID id);
ER tk_slp_tsk(INT timeout);
ER tk_dly_tsk(INT delay);
INT tm_printf(UB *format, ...);
#endif
