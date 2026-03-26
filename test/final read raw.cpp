#include <math.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);



float dt  = 0.004; // 4 milliseconds in seconds
float convert = 57.29577951; // 180 / pi
//Calibration variables
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
float accel_x_offset = 0, accel_y_offset = 0;
//Struct gia tri
typedef struct{
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float roll_angle;
    float pitch_angle;
} IMU_values;

IMU_values drone_imu;
void calibration(){
    int sample = 2000;
        for (int i = 0; i < sample; i++){
            sensors_event_t a,g,temp;
            mpu.getEvent(&a,&g,&temp);
            gyro_x_offset += g.gyro.x;
            gyro_y_offset += g.gyro.y;
            gyro_z_offset += g.gyro.z;
            accel_x_offset += a.acceleration.x;
            accel_y_offset += a.acceleration.y;
            delay(2);
    }
        gyro_x_offset /= sample;
        gyro_y_offset /= sample;
        gyro_z_offset /= sample;

        accel_x_offset /= sample;
        accel_y_offset /= sample;
}
void read_sensor(){
    sensors_event_t a,g,temp;
    mpu.getEvent(&a, &g, &temp);
    //Rate
    drone_imu.roll_rate = (g.gyro.x - gyro_x_offset)*convert;
    drone_imu.pitch_rate = (g.gyro.y - gyro_y_offset)*convert;
    drone_imu.yaw_rate = (g.gyro.z - gyro_z_offset) * convert;
    //Accelerometer Angle
    float acc_x = a.acceleration.x - accel_x_offset;
    float acc_y = a.acceleration.y - accel_y_offset;
    float acc_z = a.acceleration.z;
    //Roll Angle
    float acc_roll_angle = atan2(acc_y,acc_z)*convert;
    float acc_pitch_angle = atan2(-acc_x,sqrt(acc_y*acc_y + acc_z*acc_z)) * convert;
    //Complementary Filter
    float alpha = 0.98;
    drone_imu.roll_angle = alpha * (drone_imu.roll_angle + drone_imu.roll_rate * dt) + (1 - alpha) * acc_roll_angle;
    drone_imu.pitch_angle = alpha * (drone_imu.pitch_angle + drone_imu.pitch_rate * dt) + (1 - alpha) * acc_pitch_angle;
}
void setup(){
    Serial.begin(115200);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED FAILED");
    while(true);
  }
// Try to initialize!
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
            }
    }
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    calibration();
}
void loop(){
    unsigned long long start = micros();
    read_sensor();
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    
    display.print("Roll: ");
    display.println(drone_imu.roll_angle);


    display.print("Pitch: ");
    display.println(drone_imu.pitch_angle);


    display.print("Yaw: ");
    display.println(a.acceleration.z);
    display.display();
    while(micros() - start < dt){
        // Do nothing, just wait for the next reading
    }
}