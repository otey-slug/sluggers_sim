#ifndef HOST_STUB_IO_PORTS_H
#define HOST_STUB_IO_PORTS_H

#include "BOARD.h"
#include <stdint.h>

#define PORTV 0
#define PORTW 1
#define PORTX 2
#define PORTY 3
#define PORTZ 4

#define PIN3  0x0008U
#define PIN4  0x0010U
#define PIN5  0x0020U
#define PIN6  0x0040U
#define PIN7  0x0080U
#define PIN8  0x0100U
#define PIN9  0x0200U
#define PIN10 0x0400U
#define PIN11 0x0800U
#define PIN12 0x1000U

int8_t IO_PortsSetPortDirection(int8_t port, uint16_t pattern);
int8_t IO_PortsSetPortInputs(int8_t port, uint16_t pattern);
int8_t IO_PortsSetPortOutputs(int8_t port, uint16_t pattern);
int16_t IO_PortsReadPort(int8_t port);
int8_t IO_PortsWritePort(int8_t port, uint16_t pattern);
int8_t IO_PortsSetPortBits(int8_t port, uint16_t pattern);
int8_t IO_PortsClearPortBits(int8_t port, uint16_t pattern);
int8_t IO_PortsTogglePortBits(int8_t port, uint16_t pattern);

void IO_PortsSimReset(void);
void IO_PortsSimSetInputBits(int8_t port, uint16_t mask, uint16_t values);
void IO_PortsSimSetPingDistanceInches(double distanceInches);
uint16_t IO_PortsSimGetLatch(int8_t port);

#endif
