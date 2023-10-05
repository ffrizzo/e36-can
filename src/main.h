#ifndef MAIN_H
#define MAIN_H

#include "Arduino.h"
#include "STM32_CAN.h"

#define AC_INPUT PA5

#define AC_PRESSURE_SENSOR_TYPE PA4
#define AC_PRESSURE_SWITCH_HIGH PA6
#define AC_PRESSURE_SENSOR PB0

#define EXTERNAL_TEMPERATURE_SENSOR PB1

#define DME1 316
#define DME2 329

#define CAN_ICL2 0x613
#define CAN_ICL3 0x615

#define CAN_INTERVAL 200  // milliseconds

#define MIN_FAN_STAGE 1
#define MAX_FAN_STAGE 15

#define FAN_STAGE_AT_LOW_PRESSURE 3
#define FAN_STAGE_AT_HIGH_PRESSURE 10

void initialize();
void processCan(long currentMillis);

// DME2 t/index.php?title=Siemens_MS43_CAN_Bus#DME2_0x329
void processCanReadDME2(CAN_message_t msg);

void processCanWriteICL(long currentMillis);
// Instrument cluster sends two Message over can.
// The 0x613 is not used on e36 swap but this is needed to avoid CAN error code.
// https://www.ms4x.net/index.php?title=CAN_Bus_ID_0x613_ICL2
void processCanWriteICL2();
// https://www.ms4x.net/index.php?title=CAN_Bus_ID_0x615_ICL3
void processCanWriteICL3();

/*
 * On E36 the pressure switch is a ON/OFF. On E46 a pressure sensor is used.
 * Is it possible to replace a pressure switch with a pressure sensor to achieve a more linear
 * activation of the fan stage.
 */
byte calculateFanStage(int acStatus);
byte calculateFanStateWithPressureSensor(int acStatus);
byte calculateFanStageWithPressureSwitch(int acStatus);
byte readTemperatureSensor();

#endif
