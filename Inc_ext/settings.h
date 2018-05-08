#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include "stm32f3xx_hal.h"

// Extern
extern uint8_t          uTimeStampState;
extern FunctionalState  StateTerminalResistor;
extern FunctionalState  StatePowerCANBusDevice;

// Public
void settings_save();
uint8_t settings_get_TimeStampState();
FunctionalState settings_get_StateTerminalResistor();
FunctionalState settings_get_StatePowerCANBusDevice();

#endif