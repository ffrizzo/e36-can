#include "Arduino.h"
#include "STM32_CAN.h"

#define AC_INPUT PA6
#define AC_PRESSURE_SWITCH PA7
#define AC_PRESSURE_SENSOR PB0

#define EXTERNAL_TEMPERATURE_SENSOR PB1

#define CAN_ICL2 0x613
#define CAN_ICL3 0x615

STM32_CAN Can1(CAN1, DEF);

const int canInterval = 200;
unsigned long previousCANMillis = 0;
CAN_message_t outCanMsg;

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
    if ((currentMillis - previousCANMillis) < canInterval)
    {
        return;
    }

    Serial.println("Processing CAN Messages...");
    processCanICL2();
    processCanICL3();
    previousCANMillis = currentMillis;
}

// Send 0x613 message to avoid can message error code.
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

    outCanMsg.id = CAN_ICL3;
    outCanMsg.len = 8;

    switch (acStatus)
    {
    case HIGH:
        outCanMsg.buf[0] = 0xFB; // E36 AC Compressor does not have variable control. Set to max Torque 31nm
        outCanMsg.buf[1] = calculateFanStage(acStatus);
        outCanMsg.buf[4] = 0xC0;
        break;
    default: // OFF
        outCanMsg.buf[0] = 0x00;
        outCanMsg.buf[1] = 0x00;
        outCanMsg.buf[4] = 0x00;
        break;
    }

    outCanMsg.buf[2] = 0x00;
    outCanMsg.buf[3] = readTemperatureSensor();
    outCanMsg.buf[5] = 0x00;
    outCanMsg.buf[6] = 0x00;
    outCanMsg.buf[7] = 0x00;
    Can1.write(outCanMsg);
}

/*
* On E36 the pressure switch is a ON/OFF. On E46 a pressure sensor is used.
* Is it possible to replace a pressure switch with a pressure sensor to achieve a more linear
* activation of th fan stage.
*/
byte calculateFanStage(int acStatus)
{
    if (acStatus != HIGH) { return 0x00; }

    // If pressure sensor connected interpolate this to calculate Fan Stage
    int pressureSensor = analogRead(AC_PRESSURE_SENSOR);
    if (pressureSensor > 0)
    {
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

    int pressureSwitch = digitalRead(AC_PRESSURE_SWITCH);
    // If pressure switch is on set the stage to max(15)
    if (pressureSwitch == HIGH)
    {
        return 0xF0;
    }

    // If pressure switch is off set the stage to intermediate level(8)
    return 0x80;
}

byte readTemperatureSensor() {
    long tempSensor = analogRead(EXTERNAL_TEMPERATURE_SENSOR);
    Serial.println("External temperature sensor " + tempSensor);

    return 0x1E; // Temperature Fixed in 30C
}
