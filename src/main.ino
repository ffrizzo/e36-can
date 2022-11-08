#include "Arduino.h"
#include "STM32_CAN.h"

#define AC_INPUT PA6
#define AC_PRESSURE_SWITCH PA7
#define AC_PRESSURE_SENSOR PB0

#define EXTERNAL_TEMPERATURE_SENSOR PB1

#define CAN_ICL2 0x613
#define CAN_ICL3 0x615

#define CAN_INTERVAL 200

#define MIN_FAN_STAGE_PRESSURE_SWITCH 5
#define MAX_FAN_STAGE_PRESSURE_SWITCH 15

STM32_CAN Can1(CAN1, DEF);

unsigned long previousCANMillis = 0;
CAN_message_t outCanMsg;

int previousACStatus = LOW;
int currentFanStage = MIN_FAN_STAGE_PRESSURE_SWITCH;
unsigned long fanStageCount = 0;

void setup()
{
    initialize();
}

void loop()
{
    unsigned long currentMillis = millis();
    processCan(currentMillis);
}

void  initialize()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Initializing CAN module...");

    pinMode(AC_INPUT, INPUT_PULLDOWN);
    pinMode(AC_PRESSURE_SWITCH, INPUT_PULLDOWN);
    pinMode(AC_PRESSURE_SENSOR, INPUT_ANALOG);

    pinMode(EXTERNAL_TEMPERATURE_SENSOR, INPUT_ANALOG);

    Can1.begin();
    Can1.setBaudRate(500000);
}

void processCan(long currentMillis){
    if ((currentMillis - previousCANMillis) < CAN_INTERVAL)
    {
        return;
    }

    Serial.println("Processing CAN Messages...");
    processCanICL2();
    processCanICL3();
    previousCANMillis = currentMillis;
}

// Instrument cluster sends two Message over can.
// The 0x613 is not used on e36 swap but this is needed to avoid CAN error code.
// https://www.ms4x.net/index.php?title=CAN_Bus_ID_0x613_ICL2
void processCanICL2()
{
    outCanMsg.id = CAN_ICL2;
    outCanMsg.len = 8;
    outCanMsg.buf[0] = 0x00;
    outCanMsg.buf[1] = 0x00;
    outCanMsg.buf[2] = 0x00;
    outCanMsg.buf[3] = 0x00;
    outCanMsg.buf[4] = 0x00;
    outCanMsg.buf[5] = 0x00;
    outCanMsg.buf[6] = 0x00;
    outCanMsg.buf[7] = 0x00;
    Can1.write(outCanMsg);
}

// https://www.ms4x.net/index.php?title=CAN_Bus_ID_0x615_ICL3
void processCanICL3()
{
    int acStatus = digitalRead(AC_INPUT);
    if (previousACStatus != acStatus) {
        fanStageCount = 0;
    }

    outCanMsg.id = CAN_ICL3;
    outCanMsg.len = 8;

    switch (acStatus)
    {
    case HIGH:
        outCanMsg.buf[0] = 0xDF; // E36 AC Compressor does not have variable control. Set to max Torque 31nm
        outCanMsg.buf[4] = 0xC0;
        break;
    default: // OFF
        outCanMsg.buf[0] = 0x00;
        outCanMsg.buf[4] = 0x00;
        break;
    }

    outCanMsg.buf[1] = calculateFanStage(acStatus);
    outCanMsg.buf[2] = 0x00;
    outCanMsg.buf[3] = readTemperatureSensor();
    outCanMsg.buf[5] = 0x00;
    outCanMsg.buf[6] = 0x00;
    outCanMsg.buf[7] = 0x00;
    Can1.write(outCanMsg);

    previousACStatus = acStatus;
}

/*
* On E36 the pressure switch is a ON/OFF. On E46 a pressure sensor is used.
* Is it possible to replace a pressure switch with a pressure sensor to achieve a more linear
* activation of the fan stage.
*/
byte calculateFanStage(int acStatus)
{
    // TODO this is not functional. Need to properly interpolate the voltage vs pressure
    // If pressure sensor is connected interpolate this to calculate Fan Stage
    int pressureSensor = analogRead(AC_PRESSURE_SENSOR);
    if (pressureSensor > 0)
    {
        // If AC is turned off but pressure is to high keep the fan running
        if (acStatus == LOW && pressureSensor < 13) {
            return 0x00;
        }

        if (pressureSensor > 28) {
            return 0xF0;
        } else if (pressureSensor > 23) {
            return 0xD0;
        } else if (pressureSensor > 21){
            return 0xC0;
        } else if (pressureSensor > 17) {
            return 0xA0;
        } else if (pressureSensor > 13) {
            return 0x50;
        } else if (pressureSensor > 9) {
            return 0x20;
        } else if (pressureSensor < 10) {
            return 0x10;
        }
    }

    return calculateFanStageWithPressureSwitch(acStatus);
}

byte calculateFanStageWithPressureSwitch(int acStatus)
{
    int pressureSwitch = digitalRead(AC_PRESSURE_SWITCH);

    // Wait until low pressure or 20 cycles to turn off the fan when AC is turned off
    if (acStatus == LOW) {
        if (pressureSwitch == HIGH || fanStageCount%20 == 0) {
            currentFanStage = MIN_FAN_STAGE_PRESSURE_SWITCH;
            return 0x00;
        }

        fanStageCount++;
        return MIN_FAN_STAGE_PRESSURE_SWITCH;
    }

    // Will increase/decrease fan stage every 3 cycles of CAN_INTERVAL
    // until reach out the MAX_FAN_STAGE_PRESSURE_SWITCH or MIN_FAN_STAGE_PRESSURE_SWITCH
    switch (pressureSwitch) {
    case LOW:
        if (currentFanStage < MAX_FAN_STAGE_PRESSURE_SWITCH
                 && (fanStageCount%3 == 0)) {
            currentFanStage++;
        }
        break;
    default:
        if (currentFanStage > MIN_FAN_STAGE_PRESSURE_SWITCH
                && (fanStageCount%3 == 0)) {
            currentFanStage--;
        }
        break;
    }

    fanStageCount++;

    String result = String(currentFanStage, HEX) + '0';
    return result.toInt();
}

byte readTemperatureSensor() {
    long tempSensor = analogRead(EXTERNAL_TEMPERATURE_SENSOR);
    Serial.println("External temperature sensor " + tempSensor);

    return 0x1E; // Temperature Fixed in 30C
}
