
#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include "ev_types.h"

void VehicleControl_Init(void);

void VehicleControl_UpdateFromCan(
    BmsStatus_t bmsStatus,
    MotorStatus_t motorStatus,
    ChargerStatus_t chargerStatus
);

VehicleState_t VehicleControl_GetState(void);
uint32_t VehicleControl_GetFaultFlags(void);

BmsCommand_t VehicleControl_GetBmsCommand(void);
MotorCommand_t VehicleControl_GetMotorCommand(void);
ChargerCommand_t VehicleControl_GetChargerCommand(void);

#endif /* VEHICLE_CONTROL_H */
