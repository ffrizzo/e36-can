#include "main.h"

void setup() {
    initialize();
}

void loop() {
    unsigned long currentMillis = millis();
    processCan(currentMillis);
}

void initialize() {
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("Initializing CAN module...");

    pinMode(AC_INPUT, INPUT_PULLDOWN);
    pinMode(AC_PRESSURE_SWITCH_LOW, INPUT_PULLDOWN);
    pinMode(AC_PRESSURE_SWITCH_HIGH, INPUT_PULLDOWN);

    pinMode(AC_PRESSURE_SENSOR, INPUT_ANALOG);

    pinMode(EXTERNAL_TEMPERATURE_SENSOR, INPUT_ANALOG);

    Can1.begin();
    Can1.setBaudRate(500000);
}

void processCan(long currentMillis) {
    if ((currentMillis - previousCANMillis) < CAN_INTERVAL) {
        return;
    }

    Serial.println("Processing CAN Messages...");
    processCanICL2();
    processCanICL3();
    previousCANMillis = currentMillis;
}

void processCanICL2() {
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

void processCanICL3() {
    int acStatus = digitalRead(AC_INPUT);
    if (previousACStatus != acStatus) {
        fanStageCount = 0;
    }

    outCanMsg.id = CAN_ICL3;
    outCanMsg.len = 8;

    switch (acStatus) {
        case HIGH:
            outCanMsg.buf[0] = 0xDF;  // E36 AC Compressor does not have variable control. Set to max Torque 31nm
            outCanMsg.buf[4] = 0xC0;
            break;
        default:  // OFF
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

byte calculateFanStage(int acStatus) {
    // TODO this is not functional. Need to properly interpolate the voltage vs pressure
    // If pressure sensor is connected interpolate this to calculate Fan Stage
    int pressureSensor = analogRead(AC_PRESSURE_SENSOR);
    if (pressureSensor > 0) {
        // If AC is turned off but pressure is to high keep the fan running
        if (acStatus == LOW && pressureSensor < 13) {
            return 0x00;
        }

        if (pressureSensor > 28) {
            return 0xF0;
        } else if (pressureSensor > 23) {
            return 0xD0;
        } else if (pressureSensor > 21) {
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

byte calculateFanStageWithPressureSwitch(int acStatus) {
    bool pressureLow = digitalRead(AC_PRESSURE_SWITCH_LOW) == HIGH;
    bool pressureHigh = digitalRead(AC_PRESSURE_SWITCH_HIGH) == HIGH;

    // Wait until low pressure or 20 cycles to turn off the fan when AC is turned off
    if (acStatus == LOW) {
        if ((pressureLow || pressureHigh) && fanStageCount % 20 == 0) {
            fanStageCount++;

            String result = String(currentFanStage, HEX) + '0';
            return result.toInt();
        }

        currentFanStage = MIN_FAN_STAGE;
        return 0x00;
    }

    // Will increase/decrease fan stage every 3 cycles of CAN_INTERVAL
    // until reach out the MAX_FAN_STAGE or MIN_FAN_STAGE
    if (fanStageCount % 3 == 0) {
        int switchState = 0;
        int maxStage = MAX_FAN_STAGE;
        int minStage = MIN_FAN_STAGE;

        if (pressureHigh || pressureLow) {
            switchState = 1;

            if (pressureHigh) {
                // If high pressure keep the min fan stage at minimum of ~50%
                maxStage = MAX_FAN_STAGE;
                minStage = MAX_FAN_STAGE / 2;
            } else if (pressureLow) {
                // If low pressure keep the max fan stage at maximum of ~50%
                maxStage = MAX_FAN_STAGE / 2;
                minStage = MIN_FAN_STAGE;
            }
        }

        switch (switchState) {
            case LOW:
                if (currentFanStage < maxStage) {
                    currentFanStage++;
                }
                break;
            default:
                if (currentFanStage > minStage) {
                    currentFanStage--;
                }
                break;
        }
    }

    fanStageCount++;

    String result = String(currentFanStage, HEX) + '0';
    return result.toInt();
}

byte readTemperatureSensor() {
    long tempSensor = analogRead(EXTERNAL_TEMPERATURE_SENSOR);
    Serial.println("External temperature sensor " + tempSensor);

    return 0x1E;  // Temperature Fixed in 30C
}
