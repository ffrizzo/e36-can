#include "Arduino.h"
#include "STM32_CAN.h"

#define AC_INPUT PA6
#define AC_PRESSURE_SWITCH PA7
#define TEMPERATURE_SENSOR PA5

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
    pinMode(TEMPERATURE_SENSOR, INPUT_ANALOG);

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
        outCanMsg.buf[1] = readPressureSwitch(acStatus);
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

byte readPressureSwitch(int acStatus)
{
    if (acStatus != HIGH) { return 0x0; }

    int pressureSwitch = digitalRead(AC_PRESSURE_SWITCH);
    if (pressureSwitch == HIGH)
    {
        return 0xF0;
    }

    return 0x80;
}

byte readTemperatureSensor() {
    long temperatureSensor = analogRead(TEMPERATURE_SENSOR);
    return 0x1E; // Temperature Fixed in 30c
}
