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
#ifdef ES_SIM_HSM
#include "Buttons.h"
#include "LineSensors.h"
#endif
#include <math.h>
#include <string.h>

#define ES_STEP_GUARD 64U
#define TAPE_AMBIENT_AD 50U
#define TAPE_ACTIVE_AD 260U
#define DRIVE_MAX_IN_PER_SEC 18.0
#define DRIVE_MAX_RAD_PER_SEC 4.0

static int Initialized;

#ifdef ES_SIM_HSM
/* Host-stub setters (not part of the PIC headers). */
void Buttons_SimSetRaw(uint8_t pressedMask);
void LineSensors_SimSetMask(uint16_t mask);

static uint16_t LineMaskFromTape(const SimTapeSensors *tape)
{
    uint16_t mask = 0U;

    if (tape == 0) {
        return 0U;
    }

    /*
     * The sim provides 8-ish logical regions, while the project expects 10
     * discrete sensors. We map each sim region to a reasonable subset.
     */
    if (tape->front_left || tape->front || tape->left) {
        mask |= (LINE_FRONT_LEFT_OUT_MASK | LINE_FRONT_LEFT_IN_MASK);
    }
    if (tape->front_right || tape->front || tape->right) {
        mask |= (LINE_FRONT_RIGHT_IN_MASK | LINE_FRONT_RIGHT_OUT_MASK);
    }

    if (tape->mid_left || tape->left) {
        mask |= LINE_MIDDLE_LEFT_MASK;
    }
    if (tape->mid_right || tape->right) {
        mask |= LINE_MIDDLE_RIGHT_MASK;
    }
    if (tape->front_center) {
        mask |= (LINE_MIDDLE_LEFT_MASK | LINE_MIDDLE_RIGHT_MASK);
    }

    if (tape->rear_left || tape->rear || tape->left) {
        mask |= (LINE_REAR_LEFT_OUT_MASK | LINE_REAR_LEFT_IN_MASK);
    }
    if (tape->rear_right || tape->rear || tape->right) {
        mask |= (LINE_REAR_RIGHT_OUT_MASK | LINE_REAR_RIGHT_IN_MASK);
    }
    if (tape->rear_center) {
        mask |= (LINE_REAR_LEFT_IN_MASK | LINE_REAR_RIGHT_IN_MASK);
    }

    return mask;
}

static uint8_t RawButtonsFromBump(const SimBumpSensors *bump)
{
    uint8_t raw = 0U;

    if (bump == 0) {
        return 0U;
    }

    if (bump->front || bump->any) {
        raw |= (BUTTON_FRONT_LEFT_MASK | BUTTON_FRONT_RIGHT_MASK | BUTTON_MIDDLE_FRONT_MASK);
    }
    if (bump->rear || bump->any) {
        raw |= (BUTTON_REAR_LEFT_MASK | BUTTON_REAR_RIGHT_MASK);
    }
    if (bump->left || bump->any) {
        raw |= (BUTTON_MIDDLE_LEFT_MASK | BUTTON_FRONT_LEFT_MASK | BUTTON_REAR_LEFT_MASK);
    }
    if (bump->right || bump->any) {
        raw |= (BUTTON_RIGHT_MIDDLE_MASK | BUTTON_FRONT_RIGHT_MASK | BUTTON_REAR_RIGHT_MASK);
    }

    return raw;
}
#endif

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

#ifdef ES_SIM_HSM
    LineSensors_SimSetMask(LineMaskFromTape(tape));
#endif
}

static void PushBumpSensors(const SimBumpSensors *bump)
{
#ifdef ES_SIM_HSM
    Buttons_SimSetRaw(RawButtonsFromBump(bump));
#else
    uint16_t values = 0U;

    values |= BoolMask(bump->front || bump->any, BUMP_FRONT_PIN);
    values |= BoolMask(bump->rear || bump->any, BUMP_REAR_PIN);
    values |= BoolMask(bump->left || bump->any, BUMP_LEFT_PIN);
    values |= BoolMask(bump->right || bump->any, BUMP_RIGHT_PIN);
    IO_PortsSimSetInputBits(BUMP_PORT, BUMP_PINS, values);
#endif
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
#ifdef ES_SIM_HSM
    /*
     * HSM project: DriveMotorService uses mecanum primitives directly.
     * Infer body-frame commands from wheel PWM + direction pins.
     */

    /* Must match the mapping in ECE118FinalProject/src/Mecanum.c */
    const unsigned char FL_PWM = PWM_PORTY12;
    const unsigned char FR_PWM = PWM_PORTY10;
    const unsigned char RL_PWM = PWM_PORTX11;
    const unsigned char RR_PWM = PWM_PORTZ06;

    const uint16_t FL_IN1 = PIN6; /* PORTY06 */
    const uint16_t FL_IN2 = PIN7; /* PORTY07 */
    const uint16_t FR_IN1 = PIN3; /* PORTY03 */
    const uint16_t FR_IN2 = PIN5; /* PORTY05 */

    const uint16_t RL_IN1 = PIN3; /* PORTV03 */
    const uint16_t RL_IN2 = PIN4; /* PORTV04 */
    const uint16_t RR_IN1 = PIN5; /* PORTV05 */
    const uint16_t RR_IN2 = PIN6; /* PORTV06 */

    uint16_t portY = (uint16_t)IO_PortsReadPort(PORTY);
    uint16_t portV = (uint16_t)IO_PortsReadPort(PORTV);

    unsigned int dutyFl = PWM_GetDutyCycle(FL_PWM);
    unsigned int dutyFr = PWM_GetDutyCycle(FR_PWM);
    unsigned int dutyRl = PWM_GetDutyCycle(RL_PWM);
    unsigned int dutyRr = PWM_GetDutyCycle(RR_PWM);

    double fl = 0.0;
    double fr = 0.0;
    double rl = 0.0;
    double rr = 0.0;

    /* Direction decoding based on SetFrontWheelDir / SetRearWheelDir. */
    if (((portY & FL_IN1) == 0U) && ((portY & FL_IN2) != 0U)) {
        fl = (double)dutyFl / 1000.0;
    } else if (((portY & FL_IN1) != 0U) && ((portY & FL_IN2) == 0U)) {
        fl = -((double)dutyFl / 1000.0);
    }

    if (((portY & FR_IN1) == 0U) && ((portY & FR_IN2) != 0U)) {
        fr = (double)dutyFr / 1000.0;
    } else if (((portY & FR_IN1) != 0U) && ((portY & FR_IN2) == 0U)) {
        fr = -((double)dutyFr / 1000.0);
    }

    if (((portV & RL_IN1) != 0U) && ((portV & RL_IN2) == 0U)) {
        rl = (double)dutyRl / 1000.0;
    } else if (((portV & RL_IN1) == 0U) && ((portV & RL_IN2) != 0U)) {
        rl = -((double)dutyRl / 1000.0);
    }

    if (((portV & RR_IN1) != 0U) && ((portV & RR_IN2) == 0U)) {
        rr = (double)dutyRr / 1000.0;
    } else if (((portV & RR_IN1) == 0U) && ((portV & RR_IN2) != 0U)) {
        rr = -((double)dutyRr / 1000.0);
    }

    /* Invert the wheel decomposition used by Mecanum_DriveVector(). */
    {
        double vy = (fl + fr + rl + rr) / 4.0; /* forward */
        double vx = (fl - fr - rl + rr) / 4.0; /* strafe-right */
        double w = (fl - fr + rl - rr) / 4.0;  /* yaw CW */

        controls->vx = vy * DRIVE_MAX_IN_PER_SEC;
        controls->vy = vx * DRIVE_MAX_IN_PER_SEC;
        controls->omega = w * DRIVE_MAX_RAD_PER_SEC;
    }
    controls->shoot = FiringMotorService_IsBusy() ? 1U : 0U;
#else
    DriveMotorCommand_t drive = DriveMotorService_GetLastCommand();
    unsigned int firingDuty = PWM_GetDutyCycle(FIRING_PWM_PIN);

    controls->vx = ((double)drive.forward / 100.0) * DRIVE_MAX_IN_PER_SEC;
    controls->vy = ((double)drive.strafe / 100.0) * DRIVE_MAX_IN_PER_SEC;
    controls->omega = ((double)drive.rotate / 100.0) * DRIVE_MAX_RAD_PER_SEC;
    controls->shoot = (FiringMotorService_IsBusy() || firingDuty > 0U) ? 1U : 0U;
#endif
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
