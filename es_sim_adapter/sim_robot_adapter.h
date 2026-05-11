#ifndef SIM_ROBOT_ADAPTER_H
#define SIM_ROBOT_ADAPTER_H

#include <stdint.h>

#ifdef _WIN32
#define SIM_EXPORT __declspec(dllexport)
#else
#define SIM_EXPORT
#endif

typedef enum {
    SIM_EVENT_NONE = 0,
    SIM_EVENT_INIT,
    SIM_EVENT_TICK,
    SIM_EVENT_TAPE_CHANGED,
    SIM_EVENT_BUMP_CHANGED,
    SIM_EVENT_IR_CHANGED,
    SIM_EVENT_PING_CHANGED,
    SIM_EVENT_GAME_CHANGED,
    SIM_EVENT_TIMEOUT,
} SimEventType;

typedef struct {
    SimEventType EventType;
    uint16_t EventParam;
} SimEvent;

typedef struct {
    uint8_t front_left;
    uint8_t front_center;
    uint8_t front_right;
    uint8_t mid_left;
    uint8_t mid_right;
    uint8_t rear_left;
    uint8_t rear_center;
    uint8_t rear_right;
    uint8_t front;
    uint8_t rear;
    uint8_t left;
    uint8_t right;
} SimTapeSensors;

typedef struct {
    uint8_t any;
    uint8_t front;
    uint8_t rear;
    uint8_t left;
    uint8_t right;
} SimBumpSensors;

typedef struct {
    double front;
    double front_left;
    double front_right;
    double left;
    double right;
} SimPingSensors;

typedef struct {
    double range_in;
    double bearing_deg;
    double strength;
    uint8_t front;
    uint8_t rear;
} SimIrBeacon;

typedef struct {
    double x;
    double y;
    double heading_deg;
} SimPose;

typedef struct {
    uint8_t ammo;
    uint16_t score;
    uint8_t hits;
    uint8_t goal;
    uint8_t legal_zone;
    uint8_t isz_reached;
    uint8_t disqualified;
} SimGameState;

typedef struct {
    double time_s;
    double dt_s;
    SimPose pose;
    SimTapeSensors tape;
    SimBumpSensors bump;
    SimPingSensors ping;
    SimIrBeacon target_ir;
    SimIrBeacon obstacle_ir;
    SimGameState game;
} SimSensors;

typedef struct {
    double vx;
    double vy;
    double omega;
    uint8_t shoot;
} SimControls;

SIM_EXPORT int SimRobot_Init(void);
SIM_EXPORT int SimRobot_Step(const SimSensors *sensors, double dt, SimControls *controls);
SIM_EXPORT void SimRobot_Reset(void);

#endif
