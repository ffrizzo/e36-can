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
    Serial.println("Initializing module...");

    pinMode(AC_INPUT, INPUT_PULLDOWN);

    pinMode(AC_PRESSURE_TYPE, INPUT);
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
    Serial.printf("AC Status is %d\n", acStatus);

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
    if (digitalRead(AC_PRESSURE_TYPE) == HIGH) {
        return calculateFanStageWithPressureSwitch(acStatus);
    }

    return calculateFanStateWithPressureSensor(acStatus);
}

byte calculateFanStateWithPressureSensor(int accStatus) {
    Serial.println("calculate fan stage with pressure sensor");
    // TODO this is not functional. Need to properly interpolate the voltage vs pressure
    // If pressure sensor is connected interpolate this to calculate Fan Stage
    // int pressureSensor = analogRead(AC_PRESSURE_SENSOR);
    // if (pressureSensor > 0) {
    //     // If AC is turned off but pressure is to high keep the fan running
    //     if (acStatus == LOW && pressureSensor < 13) {
    //         return 0x00;
    //     }

    //     if (pressureSensor > 28) {
    //         return 0xF0;
    //     } else if (pressureSensor > 23) {
    //         return 0xD0;
    //     } else if (pressureSensor > 21) {
    //         return 0xC0;
    //     } else if (pressureSensor > 17) {
    //         return 0xA0;
    //     } else if (pressureSensor > 13) {
    //         return 0x50;
    //     } else if (pressureSensor > 9) {
    //         return 0x20;
    //     } else if (pressureSensor < 10) {
    //         return 0x10;
    //     }
    // }

    // To be on safe side
    String result = String(MAX_FAN_STAGE - 5, HEX) + '0';
    return result.toInt();
}

byte calculateFanStageWithPressureSwitch(int acStatus) {
    Serial.println("calculate fan stage with pressure switch");

    bool pressureIsLow = digitalRead(AC_PRESSURE_SWITCH_LOW) == HIGH;
    bool pressureIsHigh = digitalRead(AC_PRESSURE_SWITCH_HIGH) == HIGH;

    int stage = 0;
    if (acStatus == LOW) {
        if (pressureIsHigh) {
            stage = FAN_STAGE_HIGH_PRESSURE;
        } else if (pressureIsLow) {
            stage = FAN_STAGE_LOW_PRESSURE;
        } else {
            stage = 0;
        }

    } else {
        if (pressureIsHigh) {
            stage = FAN_STAGE_HIGH_PRESSURE;
        } else if (pressureIsLow) {
            stage = FAN_STAGE_LOW_PRESSURE;
        } else {
            // If AC is ON eFan should run at the MIN_FAN_STAGE
            stage = MIN_FAN_STAGE;
        }
    }

    String result = String(stage, HEX) + '0';
    return result.toInt();
}

byte readTemperatureSensor() {
    long tempSensor = analogRead(EXTERNAL_TEMPERATURE_SENSOR);
    Serial.println("External temperature sensor " + tempSensor);

    return 0x1E;  // Temperature Fixed in 30C
}
