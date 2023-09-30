#include "main.h"

const byte FAN_STAGE_TO_HEX[16] = {0x0, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};

STM32_CAN Can1(CAN1, DEF);

unsigned long previousCANMillis = 0;
CAN_message_t outCanMsg;

int previousACStatus = LOW;
unsigned long acStatusChangedMillis = 0;

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

    pinMode(AC_PRESSURE_SENSOR_TYPE, INPUT_PULLUP);
    pinMode(AC_PRESSURE_SWITCH_HIGH, INPUT_PULLUP);
    pinMode(AC_PRESSURE_SENSOR, INPUT_ANALOG);

    pinMode(EXTERNAL_TEMPERATURE_SENSOR, INPUT_ANALOG);

    previousACStatus = digitalRead(AC_INPUT);
    acStatusChangedMillis = millis() - (30 * 1000);

    Can1.begin();
    Can1.setBaudRate(500000);
}

void processCan(long currentMillis) {
    if ((currentMillis - previousCANMillis) < CAN_INTERVAL) {
        return;
    }

    Serial.println("Processing CAN Messages...");
    processCanICL2();
    processCanICL3(currentMillis);
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

void processCanICL3(long currentMillis) {
    int acStatus = digitalRead(AC_INPUT);
    Serial.printf("AC status is: %s\n", acStatus == 1 ? "on" : "off");
    if (acStatus != previousACStatus) {
        acStatusChangedMillis = currentMillis;
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

    outCanMsg.buf[1] = calculateFanStage(acStatus, currentMillis);
    outCanMsg.buf[2] = 0x00;
    outCanMsg.buf[3] = readTemperatureSensor();
    outCanMsg.buf[5] = 0x00;
    outCanMsg.buf[6] = 0x00;
    outCanMsg.buf[7] = 0x00;
    Can1.write(outCanMsg);

    previousACStatus = acStatus;
}

byte calculateFanStage(int acStatus, long currentMillis) {
    // eFan should change status after 30 secs of AC Status have beeing changed
    int diff = (currentMillis - acStatusChangedMillis) * 0.001;
    if (diff < 30) {
        if (acStatus == HIGH) {
            return FAN_STAGE_TO_HEX[0];
        }

        // if ac status was changed to off keep fun running for other 30secs
        // Set the ac status to high will guarantee the fan runs on right stage
        acStatus = HIGH;
    }

    if (digitalRead(AC_PRESSURE_SENSOR_TYPE) == HIGH) {
        return calculateFanStageWithPressureSwitch(acStatus);
    }

    return calculateFanStateWithPressureSensor(acStatus);
}

byte calculateFanStateWithPressureSensor(int acStatus) {
    // TODO this is not functional. Need to properly interpolate the voltage vs pressure
    // If pressure sensor is connected interpolate this to calculate Fan Stage
    // int pressureSensor = analogRead(AC_PRESSURE_SENSOR);
    // if (pressureSensor > 0) {

    int stage = MAX_FAN_STAGE - 5;
    // If AC is turned off but pressure is to high keep the fan running
    //     if (acStatus == LOW && pressureSensor < 13) {
    //        stage = 0;
    //     }

    //     if (pressureSensor > 28) {
    //         stage = 15;
    //     } else if (pressureSensor > 23) {
    //         stage = 12;
    //     } else if (pressureSensor > 21) {
    //         stage = 9;
    //     } else if (pressureSensor > 17) {
    //         stage = 7;
    //     } else if (pressureSensor > 13) {
    //         stage = 5;
    //     } else if (pressureSensor > 9) {
    //         stage = 3;
    //     } else if (pressureSensor < 10) {
    //         stage = 2;
    //     }
    // }

    Serial.printf("Fan stage %d calculate by pressure sensor\n", stage);
    return FAN_STAGE_TO_HEX[stage];
}

byte calculateFanStageWithPressureSwitch(int acStatus) {
    // Pressure switch works with ground to provide a signal for second stage relay
    bool pressureIsHigh = digitalRead(AC_PRESSURE_SWITCH_HIGH) == LOW;

    int stage = 0;
    if (pressureIsHigh) {
        stage = FAN_STAGE_AT_HIGH_PRESSURE;
    } else if (acStatus == HIGH) {
        // If AC is ON eFan should run at the FAN_STAGE_LOW_PRESSURE
        stage = FAN_STAGE_AT_LOW_PRESSURE;
    }

    Serial.printf("Fan stage %d\n", stage);
    return FAN_STAGE_TO_HEX[stage];
}

byte readTemperatureSensor() {
    long tempSensor = analogRead(EXTERNAL_TEMPERATURE_SENSOR);
    Serial.printf("External temperature sensor %d\n", tempSensor);

    return 0x1E;  // Temperature Fixed in 30C
}
