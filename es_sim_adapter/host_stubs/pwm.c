#include "pwm.h"
#include "BOARD.h"

static unsigned int ActivePins;
static unsigned int Frequency = PWM_DEFAULT_FREQUENCY;
static unsigned int DutyCycles[5];

static int PinToIndex(unsigned int Pin)
{
    unsigned int bit;

    if (Pin == 0U || (Pin & (Pin - 1U)) != 0U) {
        return -1;
    }
    for (bit = 0U; bit < 5U; bit++) {
        if (Pin == (1U << bit)) {
            return (int)bit;
        }
    }
    return -1;
}

char PWM_Init(void)
{
    Frequency = PWM_DEFAULT_FREQUENCY;
    return SUCCESS;
}

char PWM_SetFrequency(unsigned int NewFrequency)
{
    if (NewFrequency < MIN_PWM_FREQ || NewFrequency > MAX_PWM_FREQ) {
        return ERROR;
    }
    Frequency = NewFrequency;
    return SUCCESS;
}

unsigned int PWM_GetFrequency(void)
{
    return Frequency;
}

char PWM_AddPins(unsigned short int PWMpins)
{
    ActivePins |= PWMpins;
    return SUCCESS;
}

char PWM_RemovePins(unsigned int PWMPins)
{
    ActivePins &= ~PWMPins;
    return SUCCESS;
}

unsigned int PWM_ListPins(void)
{
    return ActivePins;
}

char PWM_SetDutyCycle(unsigned char Channel, unsigned int Duty)
{
    int index = PinToIndex(Channel);

    if (index < 0) {
        return ERROR;
    }
    DutyCycles[index] = Duty > MAX_PWM ? MAX_PWM : Duty;
    return SUCCESS;
}

unsigned int PWM_GetDutyCycle(char Channel)
{
    int index = PinToIndex((unsigned int)Channel);

    if (index < 0) {
        return ERROR;
    }
    return DutyCycles[index];
}

char PWM_End(void)
{
    PWM_SimReset();
    return SUCCESS;
}

void PWM_SimReset(void)
{
    unsigned int i;

    ActivePins = 0U;
    Frequency = PWM_DEFAULT_FREQUENCY;
    for (i = 0U; i < 5U; i++) {
        DutyCycles[i] = 0U;
    }
}
