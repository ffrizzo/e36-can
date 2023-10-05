#include "main.h"

const byte FAN_STAGE_TO_HEX[16] = {0x0, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};

STM32_CAN Can1(CAN1, DEF);

CAN_message_t canMsgRx;
CAN_message_t canMsgTx;

unsigned long previousCanICLMillis = 0;

bool engineIsRunning = true;
int previousACStatus = LOW;

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

    Can1.begin();
    Can1.setBaudRate(500000);
}

void processCan(long currentMillis) {
    // if (Can1.read(canMsgRx)) {
    //     switch (canMsgRx.id) {
    //         case DME2:
    //             processCanReadDME2(canMsgRx);
    //             break;
    //     }
    // }

    processCanWriteICL(currentMillis);
}

void processCanReadDME2(CAN_message_t msg) {
    byte byte3 = msg.buf[3];
    engineIsRunning = bitRead(byte3, 3) == 1;
    Serial.printf("Engine is running %s\n", engineIsRunning);
}

void processCanWriteICL(long currentMillis) {
    if ((currentMillis - previousCanICLMillis) < CAN_INTERVAL) {
        return;
    }

    Serial.println("Processing ICL CAN Messages...");
    processCanWriteICL2();
    processCanWriteICL3();
    previousCanICLMillis = currentMillis;
}

void processCanWriteICL2() {
    canMsgTx.id = CAN_ICL2;
    canMsgTx.len = 8;
    canMsgTx.buf[0] = 0x00;
    canMsgTx.buf[1] = 0x00;
    canMsgTx.buf[2] = 0x00;
    canMsgTx.buf[3] = 0x00;
    canMsgTx.buf[4] = 0x00;
    canMsgTx.buf[5] = 0x00;
    canMsgTx.buf[6] = 0x00;
    canMsgTx.buf[7] = 0x00;
    Can1.write(canMsgTx);
}

void processCanWriteICL3() {
    int acStatus = digitalRead(AC_INPUT);
    Serial.printf("AC status is: %s\n", acStatus == 1 ? "on" : "off");

    canMsgTx.id = CAN_ICL3;
    canMsgTx.len = 8;

    if (engineIsRunning) {
        if (acStatus != previousACStatus) {
            // E46 sends a signal of ac requesting before sending the actual request for compressor activation
            // this is valid for both states on/off
            canMsgTx.buf[0] = 0x80;
            canMsgTx.buf[1] = calculateFanStage(previousACStatus);
        } else {
            switch (acStatus) {
                case HIGH:
                    canMsgTx.buf[0] = 0xD9;  // E36 AC Compressor does not have variable control
                    break;
                default:  // OFF
                    canMsgTx.buf[0] = 0x00;
                    break;
            }

            canMsgTx.buf[1] = calculateFanStage(acStatus);
        }

        previousACStatus = acStatus;
    } else {
        canMsgTx.buf[0] = 0x00;
        canMsgTx.buf[1] = 0x00;
    }

    canMsgTx.buf[2] = 0x00;
    canMsgTx.buf[3] = readTemperatureSensor();
    canMsgTx.buf[4] = 0x00;
    canMsgTx.buf[5] = 0x00;
    canMsgTx.buf[6] = 0x00;
    canMsgTx.buf[7] = 0x00;
    Can1.write(canMsgTx);
}

byte calculateFanStage(int acStatus) {
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
    int stage = 0;
    if (acStatus == HIGH) {
        // Pressure switch works with ground to provide a signal for second stage relay
        bool pressureIsHigh = digitalRead(AC_PRESSURE_SWITCH_HIGH) == LOW;
        if (pressureIsHigh) {
            stage = FAN_STAGE_AT_HIGH_PRESSURE;
        } else {
            // If AC is ON eFan should run at the FAN_STAGE_LOW_PRESSURE
            stage = FAN_STAGE_AT_LOW_PRESSURE;
        }
    }

    Serial.printf("Fan stage %d\n", stage);
    return FAN_STAGE_TO_HEX[stage];
}

byte readTemperatureSensor() {
    long tempSensor = analogRead(EXTERNAL_TEMPERATURE_SENSOR);
    Serial.printf("External temperature sensor %d\n", tempSensor);

    return 0x1E;  // Temperature Fixed in 30C
}
