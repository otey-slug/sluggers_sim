#include "sim_robot_service.h"
#include <math.h>
#include <string.h>

typedef enum {
    ROBOT_FACE_ISZ = 0,
    ROBOT_DRIVE_TO_ISZ,
    ROBOT_BACK_UP,
    ROBOT_STRAFE_AROUND,
    ROBOT_SEARCH_BEACON,
    ROBOT_AIM,
    ROBOT_FIRE,
    ROBOT_SETTLE,
    ROBOT_DONE,
} RobotState;

static uint8_t MyPriority;
static RobotState State;
static double StateStartTime;
static SimSensors CurrentSensors;
static SimControls CurrentControls;
static int AvoidDir;
static uint8_t ShotsRequested;

static double Clamp(double value, double lo, double hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static double HeadingErrorDeg(double target_deg, double current_deg)
{
    double error = fmod(target_deg - current_deg + 540.0, 360.0) - 180.0;
    return error;
}

static void SetState(RobotState next)
{
    if (State != next) {
        State = next;
        StateStartTime = CurrentSensors.time_s;
    }
}

static double StateAge(void)
{
    return CurrentSensors.time_s - StateStartTime;
}

static void Drive(double vx, double vy, double omega, uint8_t shoot)
{
    CurrentControls.vx = Clamp(vx, -18.0, 18.0);
    CurrentControls.vy = Clamp(vy, -18.0, 18.0);
    CurrentControls.omega = Clamp(omega, -4.0, 4.0);
    CurrentControls.shoot = shoot;
}

static int ChooseAvoidDir(void)
{
    double left_room = CurrentSensors.ping.front_left;
    if (CurrentSensors.ping.left < left_room) {
        left_room = CurrentSensors.ping.left;
    }

    double right_room = CurrentSensors.ping.front_right;
    if (CurrentSensors.ping.right < right_room) {
        right_room = CurrentSensors.ping.right;
    }

    return (left_room >= right_room) ? 1 : -1;
}

static double ObstacleAvoidTurn(void)
{
    const double slow_range = 18.0;
    const double side_clearance = 9.0;
    double turn = 0.0;

    if (CurrentSensors.ping.front < slow_range) {
        turn += (CurrentSensors.ping.front_left > CurrentSensors.ping.front_right) ? 1.0 : -1.0;
        turn *= (slow_range - CurrentSensors.ping.front) / slow_range;
    }
    if (CurrentSensors.ping.front_left < slow_range) {
        turn -= (slow_range - CurrentSensors.ping.front_left) / slow_range;
    }
    if (CurrentSensors.ping.front_right < slow_range) {
        turn += (slow_range - CurrentSensors.ping.front_right) / slow_range;
    }
    if (CurrentSensors.ping.left < side_clearance) {
        turn -= 0.7 * (side_clearance - CurrentSensors.ping.left) / side_clearance;
    }
    if (CurrentSensors.ping.right < side_clearance) {
        turn += 0.7 * (side_clearance - CurrentSensors.ping.right) / side_clearance;
    }
    return Clamp(turn, -1.0, 1.0);
}

static double TapeGuardTurn(void)
{
    double turn = 0.0;
    if (CurrentSensors.tape.left) {
        turn -= 0.7;
    }
    if (CurrentSensors.tape.right) {
        turn += 0.7;
    }
    return Clamp(turn, -1.0, 1.0);
}

static double TapeGuardStrafe(void)
{
    double strafe = 0.0;
    if (CurrentSensors.tape.left) {
        strafe -= 4.2;
    }
    if (CurrentSensors.tape.right) {
        strafe += 4.2;
    }
    return Clamp(strafe, -6.0, 6.0);
}

static void DriveToIsz(void)
{
    const double drive_speed = 8.5;
    const double turn_rate = 1.4;
    const double slow_range = 18.0;
    const double stop_range = 6.5;

    double nearest_front = CurrentSensors.ping.front;
    if (CurrentSensors.ping.front_left < nearest_front) {
        nearest_front = CurrentSensors.ping.front_left;
    }
    if (CurrentSensors.ping.front_right < nearest_front) {
        nearest_front = CurrentSensors.ping.front_right;
    }

    if (CurrentSensors.ping.front < stop_range) {
        AvoidDir = ChooseAvoidDir();
        SetState(ROBOT_BACK_UP);
        Drive(-7.0, 0.0, 0.0, 0);
        return;
    }

    double heading_turn = Clamp(HeadingErrorDeg(180.0, CurrentSensors.pose.heading_deg) / 90.0, -1.0, 1.0);
    double avoid_turn = ObstacleAvoidTurn();
    double avoid_weight = Clamp((slow_range - nearest_front) / slow_range, 0.0, 1.0);
    double turn = Clamp((heading_turn * 0.65 + TapeGuardTurn()) * (1.0 - avoid_weight) + avoid_turn * avoid_weight, -1.0, 1.0);
    double speed = drive_speed;
    if (nearest_front < slow_range) {
        speed = Clamp(drive_speed * nearest_front / slow_range, 3.0, drive_speed);
    }
    double strafe = Clamp(avoid_turn * 6.0 + TapeGuardStrafe(), -6.0, 6.0);
    Drive(speed, strafe, turn_rate * turn, 0);
}

int SimRobotService_Init(uint8_t priority)
{
    MyPriority = priority;
    (void)MyPriority;
    SimRobotService_Reset();
    return 1;
}

SimEvent SimRobotService_Run(SimEvent event)
{
    (void)event;
    Drive(0.0, 0.0, 0.0, 0);

    if (CurrentSensors.game.disqualified || CurrentSensors.game.goal || CurrentSensors.game.ammo == 0) {
        return (SimEvent){SIM_EVENT_NONE, 0};
    }

    if (CurrentSensors.bump.any && State != ROBOT_BACK_UP && State != ROBOT_STRAFE_AROUND) {
        AvoidDir = ChooseAvoidDir();
        SetState(ROBOT_BACK_UP);
    }

    switch (State) {
    case ROBOT_FACE_ISZ: {
        double error = HeadingErrorDeg(180.0, CurrentSensors.pose.heading_deg);
        if (fabs(error) < 10.0) {
            SetState(ROBOT_DRIVE_TO_ISZ);
        }
        Drive(0.0, 0.0, 1.4 * Clamp(error / 90.0, -1.0, 1.0), 0);
        break;
    }
    case ROBOT_DRIVE_TO_ISZ:
        if (CurrentSensors.game.isz_reached) {
            SetState(ROBOT_SEARCH_BEACON);
        } else {
            DriveToIsz();
        }
        break;
    case ROBOT_BACK_UP:
        if (StateAge() < 0.35) {
            Drive(-7.0, 0.0, 0.0, 0);
        } else {
            SetState(ROBOT_STRAFE_AROUND);
        }
        break;
    case ROBOT_STRAFE_AROUND:
        if (StateAge() < 1.15 || CurrentSensors.ping.front < 18.0) {
            Drive(4.5, 6.0 * AvoidDir, 0.35 * AvoidDir, 0);
        } else {
            SetState(CurrentSensors.game.isz_reached ? ROBOT_SEARCH_BEACON : ROBOT_FACE_ISZ);
        }
        break;
    case ROBOT_SEARCH_BEACON:
        if (CurrentSensors.target_ir.front) {
            SetState(ROBOT_AIM);
        }
        Drive(0.0, 0.0, 0.8, 0);
        break;
    case ROBOT_AIM: {
        double turn = Clamp(CurrentSensors.target_ir.bearing_deg / 45.0, -1.0, 1.0);
        if (CurrentSensors.target_ir.front && fabs(CurrentSensors.target_ir.bearing_deg) < 3.0) {
            SetState(ROBOT_FIRE);
            Drive(0.0, 0.0, 0.0, 0);
        } else {
            Drive(0.0, 0.0, 1.4 * turn, 0);
        }
        break;
    }
    case ROBOT_FIRE:
        if (StateAge() < 0.15) {
            Drive(0.0, 0.0, 0.0, 0);
        } else {
            ShotsRequested++;
            SetState(ROBOT_SETTLE);
            Drive(0.0, 0.0, 0.0, 1);
        }
        break;
    case ROBOT_SETTLE:
        if (StateAge() < 0.7) {
            Drive(0.0, 0.0, 0.0, 0);
        } else if (ShotsRequested >= 6) {
            SetState(ROBOT_DONE);
        } else {
            SetState(ROBOT_AIM);
        }
        break;
    case ROBOT_DONE:
    default:
        Drive(0.0, 0.0, 0.0, 0);
        break;
    }

    return (SimEvent){SIM_EVENT_NONE, 0};
}

int SimRobotService_Post(SimEvent event)
{
    (void)event;
    return 1;
}

void SimRobotService_Reset(void)
{
    memset(&CurrentSensors, 0, sizeof(CurrentSensors));
    memset(&CurrentControls, 0, sizeof(CurrentControls));
    State = ROBOT_FACE_ISZ;
    StateStartTime = 0.0;
    AvoidDir = 1;
    ShotsRequested = 0;
}

void SimRobotService_SetSensors(const SimSensors *sensors)
{
    if (sensors != 0) {
        CurrentSensors = *sensors;
    }
}

void SimRobotService_GetControls(SimControls *controls)
{
    if (controls != 0) {
        *controls = CurrentControls;
    }
}
