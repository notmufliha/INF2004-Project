#ifndef VC_MOCK_PLATFORM_H
#define VC_MOCK_PLATFORM_H
#include "../subsystem2/host/mock_platform.h"
typedef uint16_t UH;
typedef int BOOL;
typedef int TMO;
#define EXPORT
#define TRUE 1
#define FALSE 0
#define TP_PRC1 1u
#define CHECK(error, name) ((void)(error))
INT tk_rcv_mbf(ID id, void *data, TMO timeout);
#endif
