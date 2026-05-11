#ifndef HOST_STUB_AD_H
#define HOST_STUB_AD_H

#define AD_PORTV3 (1U << 0)
#define AD_PORTV4 (1U << 1)
#define AD_PORTV5 (1U << 2)
#define AD_PORTV6 (1U << 3)
#define AD_PORTV7 (1U << 4)
#define AD_PORTV8 (1U << 5)
#define AD_PORTW3 (1U << 6)
#define AD_PORTW4 (1U << 7)
#define AD_PORTW5 (1U << 8)
#define AD_PORTW6 (1U << 9)
#define AD_PORTW7 (1U << 10)
#define AD_PORTW8 (1U << 11)
#define BAT_VOLTAGE (1U << 12)
#define ROACH_LIGHT_SENSOR (1U << 13)

char AD_Init(void);
char AD_AddPins(unsigned int AddPins);
char AD_RemovePins(unsigned int RemovePins);
unsigned int AD_ActivePins(void);
char AD_IsNewDataReady(void);
unsigned int AD_ReadADPin(unsigned int Pin);
void AD_End(void);

void AD_SimReset(void);
void AD_SimSetPin(unsigned int Pin, unsigned int Value);
void AD_SimSetTapePin(unsigned int Pin, unsigned int LedOffValue, unsigned int LedOnValue);

#endif
