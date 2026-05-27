#include "ES_Configure.h"
#include "ES_Events.h"
#include "ES_TattleTale.h"

/*
 * Host stub for ES_TattleTale.
 *
 * The PIC implementation uses hardware timer registers to timestamp traces.
 * For the simulator build we only need to satisfy the linker.
 */

void ES_AddTattlePoint(const char *FunctionName, const char *StateName, ES_Event ThisEvent)
{
    (void)FunctionName;
    (void)StateName;
    (void)ThisEvent;
}

void ES_CheckTail(const char *FunctionName)
{
    (void)FunctionName;
}
