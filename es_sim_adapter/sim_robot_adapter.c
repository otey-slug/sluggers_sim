#include "sim_robot_adapter.h"
#include "host_stubs/AD.h"
#include "host_stubs/BOARD.h"
#include "host_stubs/IO_Ports.h"
#include "host_stubs/pwm.h"
#include "BumpDetectorService.h"
#include "DriveMotorService.h"
#include "ES_Framework.h"
#include "ES_Timers.h"
#include "FiringMotorService.h"
#include "RobotServiceConfig.h"
#include <math.h>
#include <string.h>

#define ES_STEP_GUARD 64U
#define TAPE_AMBIENT_AD 50U
#define TAPE_ACTIVE_AD 260U
#define DRIVE_MAX_IN_PER_SEC 18.0
#define DRIVE_MAX_RAD_PER_SEC 4.0

static int Initialized;

static uint16_t BoolMask(uint8_t enabled, uint16_t mask)
{
    return enabled ? mask : 0U;
}

static uint32_t SecondsToMilliseconds(double seconds)
{
    if (seconds <= 0.0) {
        return 0U;
    }
    return (uint32_t)(seconds * 1000.0 + 0.5);
}

static uint16_t AnalogFromStrength(double strength)
{
    if (strength <= 0.0) {
        return 0U;
    }
    if (strength >= 1.0) {
        return 1023U;
    }
    return (uint16_t)(strength * 1023.0 + 0.5);
}

static void SetTapeAd(unsigned int pin, uint8_t onTape)
{
    unsigned int active = onTape ? TAPE_ACTIVE_AD : TAPE_AMBIENT_AD;
    AD_SimSetTapePin(pin, TAPE_AMBIENT_AD, active);
}

static void PushTapeSensors(const SimTapeSensors *tape)
{
    SetTapeAd(TAPE_FRONT_LEFT_AD, tape->front_left || tape->front || tape->left);
    SetTapeAd(TAPE_FRONT_RIGHT_AD, tape->front_right || tape->front || tape->right);
    SetTapeAd(TAPE_REAR_LEFT_AD, tape->rear_left || tape->rear || tape->left);
    SetTapeAd(TAPE_REAR_RIGHT_AD, tape->rear_right || tape->rear || tape->right);
}

static void PushBumpSensors(const SimBumpSensors *bump)
{
    uint16_t values = 0U;

    values |= BoolMask(bump->front || bump->any, BUMP_FRONT_PIN);
    values |= BoolMask(bump->rear || bump->any, BUMP_REAR_PIN);
    values |= BoolMask(bump->left || bump->any, BUMP_LEFT_PIN);
    values |= BoolMask(bump->right || bump->any, BUMP_RIGHT_PIN);
    IO_PortsSimSetInputBits(BUMP_PORT, BUMP_PINS, values);
}

static void PushTrackWireSensors(const SimIrBeacon *target)
{
    double strength = target->strength;
    uint16_t left = 0U;
    uint16_t right = 0U;

    if (strength <= 0.0 && (target->front || target->rear)) {
        strength = 1.0;
    }

    if (target->front || target->rear || strength > 0.0) {
        uint16_t adValue = AnalogFromStrength(strength);
        if (target->bearing_deg <= 5.0) {
            left = adValue;
        }
        if (target->bearing_deg >= -5.0) {
            right = adValue;
        }
    }

    AD_SimSetPin(TRACKWIRE_LEFT_AD, left);
    AD_SimSetPin(TRACKWIRE_RIGHT_AD, right);
}

static void PushSensorsToStubs(const SimSensors *sensors)
{
    PushTapeSensors(&sensors->tape);
    PushBumpSensors(&sensors->bump);
    PushTrackWireSensors(&sensors->target_ir);
    IO_PortsSimSetPingDistanceInches(sensors->ping.front);
}

static void ReadServiceOutputs(SimControls *controls)
{
    DriveMotorCommand_t drive = DriveMotorService_GetLastCommand();
    unsigned int firingDuty = PWM_GetDutyCycle(FIRING_PWM_PIN);

    controls->vx = ((double)drive.forward / 100.0) * DRIVE_MAX_IN_PER_SEC;
    controls->vy = ((double)drive.strafe / 100.0) * DRIVE_MAX_IN_PER_SEC;
    controls->omega = ((double)drive.rotate / 100.0) * DRIVE_MAX_RAD_PER_SEC;
    controls->shoot = (FiringMotorService_IsBusy() || firingDuty > 0U) ? 1U : 0U;
}

static void ResetHostHardware(void)
{
    BOARD_Init();
    AD_SimReset();
    IO_PortsSimReset();
    PWM_SimReset();
}

SIM_EXPORT int SimRobot_Init(void)
{
    ResetHostHardware();
    Initialized = (ES_Initialize() == Success);
    return Initialized;
}

SIM_EXPORT void SimRobot_Reset(void)
{
    Initialized = 0;
    (void)SimRobot_Init();
}

SIM_EXPORT int SimRobot_Step(const SimSensors *sensors, double dt, SimControls *controls)
{
    if (sensors == 0 || controls == 0) {
        return 0;
    }
    if (!Initialized) {
        if (!SimRobot_Init()) {
            return 0;
        }
    }

    SimSensors stepped = *sensors;
    stepped.dt_s = dt;
    BOARD_SetMilliSeconds(SecondsToMilliseconds(stepped.time_s));
    PushSensorsToStubs(&stepped);

    ES_Timer_SimTick(SecondsToMilliseconds(dt));
    if (ES_RunForTicks(ES_STEP_GUARD) != Success) {
        memset(controls, 0, sizeof(*controls));
        return 0;
    }

    ReadServiceOutputs(controls);
    return 1;
}
