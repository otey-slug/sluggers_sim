#include "AD.h"
#include "BOARD.h"
#include "IO_Ports.h"
#include "RobotServiceConfig.h"
#include <stddef.h>

#define AD_MAX_CHANNELS 16
#define AD_MAX_VALUE 1023U

typedef struct {
    unsigned int ledOffValue;
    unsigned int ledOnValue;
    unsigned char usesTapeLed;
} SimAdChannel;

static unsigned int ActivePins;
static SimAdChannel Channels[AD_MAX_CHANNELS];
static unsigned char NewDataReady;

static int PinToIndex(unsigned int Pin)
{
    unsigned int bit;

    if (Pin == 0U || (Pin & (Pin - 1U)) != 0U) {
        return -1;
    }
    for (bit = 0U; bit < AD_MAX_CHANNELS; bit++) {
        if (Pin == (1U << bit)) {
            return (int)bit;
        }
    }
    return -1;
}

static unsigned int ClampAdValue(unsigned int Value)
{
    return Value > AD_MAX_VALUE ? AD_MAX_VALUE : Value;
}

char AD_Init(void)
{
    AD_SimReset();
    return SUCCESS;
}

char AD_AddPins(unsigned int AddPins)
{
    ActivePins |= AddPins;
    return SUCCESS;
}

char AD_RemovePins(unsigned int RemovePins)
{
    ActivePins &= ~RemovePins;
    return SUCCESS;
}

unsigned int AD_ActivePins(void)
{
    return ActivePins;
}

char AD_IsNewDataReady(void)
{
    unsigned char wasReady = NewDataReady;
    NewDataReady = FALSE;
    return wasReady;
}

unsigned int AD_ReadADPin(unsigned int Pin)
{
    int index = PinToIndex(Pin);

    if (index < 0) {
        return ERROR;
    }
    if (Channels[index].usesTapeLed &&
            (IO_PortsSimGetLatch(TAPE_LED_PORT) & TAPE_LED_PIN) != 0U) {
        return Channels[index].ledOnValue;
    }
    return Channels[index].ledOffValue;
}

void AD_End(void)
{
    AD_SimReset();
}

void AD_SimReset(void)
{
    unsigned int i;

    ActivePins = 0U;
    NewDataReady = FALSE;
    for (i = 0U; i < AD_MAX_CHANNELS; i++) {
        Channels[i].ledOffValue = 0U;
        Channels[i].ledOnValue = 0U;
        Channels[i].usesTapeLed = FALSE;
    }
}

void AD_SimSetPin(unsigned int Pin, unsigned int Value)
{
    int index = PinToIndex(Pin);

    if (index >= 0) {
        Channels[index].ledOffValue = ClampAdValue(Value);
        Channels[index].ledOnValue = Channels[index].ledOffValue;
        Channels[index].usesTapeLed = FALSE;
        NewDataReady = TRUE;
    }
}

void AD_SimSetTapePin(unsigned int Pin, unsigned int LedOffValue, unsigned int LedOnValue)
{
    int index = PinToIndex(Pin);

    if (index >= 0) {
        Channels[index].ledOffValue = ClampAdValue(LedOffValue);
        Channels[index].ledOnValue = ClampAdValue(LedOnValue);
        Channels[index].usesTapeLed = TRUE;
        NewDataReady = TRUE;
    }
}
