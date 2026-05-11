#ifndef HOST_STUB_PWM_H
#define HOST_STUB_PWM_H

#define MIN_PWM_FREQ 100U
#define PWM_500HZ 500U
#define PWM_1KHZ 1000U
#define PWM_2KHZ 2000U
#define PWM_5KHZ 5000U
#define PWM_10KHZ 10000U
#define PWM_20KHZ 20000U
#define PWM_30KHZ 30000U
#define PWM_40KHZ 40000U
#define MAX_PWM_FREQ 100000U
#define PWM_DEFAULT_FREQUENCY PWM_1KHZ

#define PWM_PORTZ06 (1U << 0)
#define PWM_PORTY12 (1U << 1)
#define PWM_PORTY10 (1U << 2)
#define PWM_PORTY04 (1U << 3)
#define PWM_PORTX11 (1U << 4)

#define MIN_PWM 0U
#define MAX_PWM 1000U

char PWM_Init(void);
char PWM_SetFrequency(unsigned int NewFrequency);
unsigned int PWM_GetFrequency(void);
char PWM_AddPins(unsigned short int PWMpins);
char PWM_RemovePins(unsigned int PWMPins);
unsigned int PWM_ListPins(void);
char PWM_SetDutyCycle(unsigned char Channel, unsigned int Duty);
unsigned int PWM_GetDutyCycle(char Channel);
char PWM_End(void);

void PWM_SimReset(void);

#endif
