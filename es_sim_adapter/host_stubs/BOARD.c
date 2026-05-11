#include "BOARD.h"

static uint32_t HostNowMs;

void BOARD_Init(void)
{
    HostNowMs = 0;
}

void BOARD_End(void)
{
}

uint32_t BOARD_GetMilliSeconds(void)
{
    return HostNowMs;
}

void BOARD_SetMilliSeconds(uint32_t now_ms)
{
    HostNowMs = now_ms;
}
