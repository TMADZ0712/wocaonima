// ✅ Arduino.h MUST come first
#include "Arduino.h"
#include "MPU9250.h"
#include "Wire.h"


MPU9250 mpu;

void print_roll_pitch_yaw();  // ✅ forward declaration fixes the loop() error

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() - prev_ms >= 25) {  // ✅ rollover-safe
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}