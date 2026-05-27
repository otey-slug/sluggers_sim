#include <stdint.h>

#include "Buttons.h"

/*
 * Host stub for the PIC Buttons / bump switch module.
 *
 * In the simulator we don't have real IO pins; instead the simulator adapter
 * feeds a raw 7-bit pressed mask each tick via Buttons_SimSetRaw().
 */

static uint8_t sSimPressedMask;

void Buttons_SimSetRaw(uint8_t pressedMask)
{
    sSimPressedMask = pressedMask;
}

void Buttons_Init(void)
{
    sSimPressedMask = 0;
}

void Buttons_ReassertBumpPins(void)
{
    /* No-op on host. */
}

static uint8_t ReadMask(uint8_t mask)
{
    return (sSimPressedMask & mask) ? BUTTON_PRESSED : BUTTON_NOT_PRESSED;
}

uint8_t Buttons_ReadFrontRight(void)
{
    return ReadMask(BUTTON_FRONT_RIGHT_MASK);
}

uint8_t Buttons_ReadRightMiddle(void)
{
    return ReadMask(BUTTON_RIGHT_MIDDLE_MASK);
}

uint8_t Buttons_ReadRearRight(void)
{
    return ReadMask(BUTTON_REAR_RIGHT_MASK);
}

uint8_t Buttons_ReadRearLeft(void)
{
    return ReadMask(BUTTON_REAR_LEFT_MASK);
}

uint8_t Buttons_ReadMiddleLeft(void)
{
    return ReadMask(BUTTON_MIDDLE_LEFT_MASK);
}

uint8_t Buttons_ReadFrontLeft(void)
{
    return ReadMask(BUTTON_FRONT_LEFT_MASK);
}

uint8_t Buttons_ReadMiddleFront(void)
{
    return ReadMask(BUTTON_MIDDLE_FRONT_MASK);
}

uint8_t Buttons_ReadAll(void)
{
    return sSimPressedMask;
}
