#include "IO_Ports.h"
#include "RobotServiceConfig.h"

#define NUM_PORTS 5

static uint16_t PortDirection[NUM_PORTS];
static uint16_t PortLatch[NUM_PORTS];
static uint16_t PortInputs[NUM_PORTS];
static double PingDistanceInches;
static uint32_t PingTriggerRiseMs;
static uint8_t PingTriggerArmed;

static uint8_t IsValidPort(int8_t port)
{
    return port >= 0 && port < NUM_PORTS;
}

static uint16_t PingEchoMask(void)
{
    uint32_t now;
    uint32_t echoWidthMs;

    if (!PingTriggerArmed || PingDistanceInches <= 0.0) {
        return 0U;
    }

    now = BOARD_GetMilliSeconds();
    if (now <= PingTriggerRiseMs) {
        return 0U;
    }

    echoWidthMs = (uint32_t)((PingDistanceInches * 148.0) / 1000.0 + 0.5);
    if (echoWidthMs == 0U) {
        echoWidthMs = 1U;
    }

    return (now - PingTriggerRiseMs) <= echoWidthMs ? PING_ECHO_PIN : 0U;
}

static void UpdatePingTrigger(uint16_t previousLatch, uint16_t nextLatch)
{
    if ((previousLatch & PING_TRIGGER_PIN) == 0U && (nextLatch & PING_TRIGGER_PIN) != 0U) {
        PingTriggerRiseMs = BOARD_GetMilliSeconds();
        PingTriggerArmed = TRUE;
    }
}

int8_t IO_PortsSetPortDirection(int8_t port, uint16_t pattern)
{
    if (!IsValidPort(port)) {
        return ERROR;
    }
    PortDirection[port] = pattern;
    return SUCCESS;
}

int8_t IO_PortsSetPortInputs(int8_t port, uint16_t pattern)
{
    if (!IsValidPort(port)) {
        return ERROR;
    }
    PortDirection[port] |= pattern;
    return SUCCESS;
}

int8_t IO_PortsSetPortOutputs(int8_t port, uint16_t pattern)
{
    if (!IsValidPort(port)) {
        return ERROR;
    }
    PortDirection[port] &= (uint16_t)~pattern;
    return SUCCESS;
}

int16_t IO_PortsReadPort(int8_t port)
{
    uint16_t inputMask;
    uint16_t inputValues;

    if (!IsValidPort(port)) {
        return ERROR;
    }

    inputMask = PortDirection[port];
    inputValues = PortInputs[port];
    if (port == PING_PORT) {
        inputValues = (uint16_t)((inputValues & (uint16_t)~PING_ECHO_PIN) | PingEchoMask());
    }

    return (int16_t)((PortLatch[port] & (uint16_t)~inputMask) | (inputValues & inputMask));
}

int8_t IO_PortsWritePort(int8_t port, uint16_t pattern)
{
    uint16_t previousLatch;

    if (!IsValidPort(port)) {
        return ERROR;
    }
    previousLatch = PortLatch[port];
    PortLatch[port] = pattern;
    if (port == PING_PORT) {
        UpdatePingTrigger(previousLatch, PortLatch[port]);
    }
    return SUCCESS;
}

int8_t IO_PortsSetPortBits(int8_t port, uint16_t pattern)
{
    uint16_t previousLatch;

    if (!IsValidPort(port)) {
        return ERROR;
    }
    previousLatch = PortLatch[port];
    PortLatch[port] |= pattern;
    if (port == PING_PORT) {
        UpdatePingTrigger(previousLatch, PortLatch[port]);
    }
    return SUCCESS;
}

int8_t IO_PortsClearPortBits(int8_t port, uint16_t pattern)
{
    if (!IsValidPort(port)) {
        return ERROR;
    }
    PortLatch[port] &= (uint16_t)~pattern;
    return SUCCESS;
}

int8_t IO_PortsTogglePortBits(int8_t port, uint16_t pattern)
{
    uint16_t previousLatch;

    if (!IsValidPort(port)) {
        return ERROR;
    }
    previousLatch = PortLatch[port];
    PortLatch[port] ^= pattern;
    if (port == PING_PORT) {
        UpdatePingTrigger(previousLatch, PortLatch[port]);
    }
    return SUCCESS;
}

void IO_PortsSimReset(void)
{
    int i;

    for (i = 0; i < NUM_PORTS; i++) {
        PortDirection[i] = 0U;
        PortLatch[i] = 0U;
        PortInputs[i] = 0U;
    }
    PingDistanceInches = 0.0;
    PingTriggerRiseMs = 0U;
    PingTriggerArmed = FALSE;
}

void IO_PortsSimSetInputBits(int8_t port, uint16_t mask, uint16_t values)
{
    if (IsValidPort(port)) {
        PortInputs[port] = (uint16_t)((PortInputs[port] & (uint16_t)~mask) | (values & mask));
    }
}

void IO_PortsSimSetPingDistanceInches(double distanceInches)
{
    PingDistanceInches = distanceInches;
}

uint16_t IO_PortsSimGetLatch(int8_t port)
{
    if (!IsValidPort(port)) {
        return 0U;
    }
    return PortLatch[port];
}
