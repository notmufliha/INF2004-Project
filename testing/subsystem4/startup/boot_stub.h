/** @file boot_stub.h
 * @brief Minimal native kernel ABI for executing the real B4 startup module.
 */
#ifndef BOOT_STUB_H
#define BOOT_STUB_H
#define APP_H

#include <stdint.h>
#include <stddef.h>

typedef int32_t  ID;
typedef int32_t  ER;
typedef int32_t  SZ;
typedef int32_t  INT;
typedef int32_t  TMO;
typedef uint32_t UW;
typedef uint32_t UINT;

typedef struct
{
    uint32_t hi;
    uint32_t lo;
} SYSTIM;

ER tk_get_tim (SYSTIM * p_time);

typedef struct
{
    uint32_t mbfatr;
    SZ       bufsz;
    SZ       maxmsz;
} T_CMBF;

typedef struct
{
    uint32_t flgatr;
    uint32_t iflgptn;
} T_CFLG;

#define E_OK 0
#define E_SYS (-5)
#define TMO_POL 0
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define TA_TFIFO 0U
#define TA_WMUL 8U
#define TWF_ORW 1U

ID tk_cre_mbf (const T_CMBF * p_config);
ER tk_snd_mbf (ID h_buffer, const void * p_data, INT length, TMO timeout);
INT tk_rcv_mbf (ID h_buffer, void * p_data, TMO timeout);
ID tk_cre_flg (const T_CFLG * p_config);
ER tk_wai_flg (ID h_flag, UINT mask, UINT mode, UINT * p_flags, TMO timeout);
ER tk_set_flg (ID h_flag, UINT mask);
ER tk_clr_flg (ID h_flag, UINT mask);
ID app_start_task (void (*p_entry)(INT, void *), INT priority, UW core,
                   const char * p_name);

#endif

/*** end of file ***/

