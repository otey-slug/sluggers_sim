#ifndef HOST_STUB_BOARD_H
#define HOST_STUB_BOARD_H

#include <stdint.h>

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef SUCCESS
#define SUCCESS 1
#endif

#ifndef ERROR
#define ERROR 0
#endif

void BOARD_Init(void);
void BOARD_End(void);
uint32_t BOARD_GetMilliSeconds(void);
void BOARD_SetMilliSeconds(uint32_t now_ms);

#endif
