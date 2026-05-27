#include <stdint.h>

#include "LineSensors.h"

/*
 * Host stub for the PIC line sensor module.
 *
 * The simulator adapter provides the 10-bit line mask each tick via
 * LineSensors_SimSetMask().
 */

static uint16_t sSimLineMask;

void LineSensors_SimSetMask(uint16_t mask)
{
    sSimLineMask = mask;
}

void LineSensors_Init(void)
{
    sSimLineMask = 0U;
}

static uint8_t ReadMask(uint16_t mask)
{
    return (sSimLineMask & mask) ? 1U : 0U;
}

uint8_t LineSensors_ReadFrontLeftOut(void)
{
    return ReadMask(LINE_FRONT_LEFT_OUT_MASK);
}

uint8_t LineSensors_ReadFrontLeftIn(void)
{
    return ReadMask(LINE_FRONT_LEFT_IN_MASK);
}

uint8_t LineSensors_ReadFrontRightIn(void)
{
    return ReadMask(LINE_FRONT_RIGHT_IN_MASK);
}

uint8_t LineSensors_ReadFrontRightOut(void)
{
    return ReadMask(LINE_FRONT_RIGHT_OUT_MASK);
}

uint8_t LineSensors_ReadMiddleLeft(void)
{
    return ReadMask(LINE_MIDDLE_LEFT_MASK);
}

uint8_t LineSensors_ReadMiddleRight(void)
{
    return ReadMask(LINE_MIDDLE_RIGHT_MASK);
}

uint8_t LineSensors_ReadRearLeftOut(void)
{
    return ReadMask(LINE_REAR_LEFT_OUT_MASK);
}

uint8_t LineSensors_ReadRearLeftIn(void)
{
    return ReadMask(LINE_REAR_LEFT_IN_MASK);
}

uint8_t LineSensors_ReadRearRightOut(void)
{
    return ReadMask(LINE_REAR_RIGHT_OUT_MASK);
}

uint8_t LineSensors_ReadRearRightIn(void)
{
    return ReadMask(LINE_REAR_RIGHT_IN_MASK);
}

uint16_t LineSensors_ReadAll(void)
{
    return sSimLineMask;
}
