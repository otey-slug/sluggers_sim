#ifndef SIM_ROBOT_SERVICE_H
#define SIM_ROBOT_SERVICE_H

#include "sim_robot_adapter.h"

int SimRobotService_Init(uint8_t priority);
SimEvent SimRobotService_Run(SimEvent event);
int SimRobotService_Post(SimEvent event);
void SimRobotService_Reset(void);
void SimRobotService_SetSensors(const SimSensors *sensors);
void SimRobotService_GetControls(SimControls *controls);

#endif
