#ifndef MAIN_H
#define MAIN_H

#include "Arduino.h"
#include "STM32_CAN.h"

#define AC_INPUT PA5
#define AC_PRESSURE_SWITCH_LOW PA6
#define AC_PRESSURE_SWITCH_HIGH PA7
#define AC_PRESSURE_SENSOR PB0

#define EXTERNAL_TEMPERATURE_SENSOR PB1

#define CAN_ICL2 0x613
#define CAN_ICL3 0x615

#define CAN_INTERVAL 200

#define MIN_FAN_STAGE 2
#define MAX_FAN_STAGE 15

STM32_CAN Can1(CAN1, DEF);

unsigned long previousCANMillis = 0;
CAN_message_t outCanMsg;

int previousACStatus = LOW;
int currentFanStage = MIN_FAN_STAGE;
unsigned long fanStageCount = 0;

void initialize();
void processCan(long currentMillis);

// Instrument cluster sends two Message over can.
// The 0x613 is not used on e36 swap but this is needed to avoid CAN error code.
// https://www.ms4x.net/index.php?title=CAN_Bus_ID_0x613_ICL2
void processCanICL2();
// https://www.ms4x.net/index.php?title=CAN_Bus_ID_0x615_ICL3
void processCanICL3();

/*
 * On E36 the pressure switch is a ON/OFF. On E46 a pressure sensor is used.
 * Is it possible to replace a pressure switch with a pressure sensor to achieve a more linear
 * activation of the fan stage.
 */
byte calculateFanStage(int acStatus);
byte calculateFanStageWithPressureSwitch(int acStatus);
byte readTemperatureSensor();

#endif
