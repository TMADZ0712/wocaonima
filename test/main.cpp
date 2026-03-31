#include <math.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

unsigned long long start = micros();
float dt  = 0.004; // 4 milliseconds in seconds
float convert = 57.29577951; // 180 / pi

//Calibration variables
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
float accel_x_offset = 0, accel_y_offset = 0;
//Struct for MPU values
typedef struct{
    //Rate Loop
    float roll_rate;
    float pitch_rate;
    float yaw_rate;

    //Angle Loop
    float roll_angle;
    float pitch_angle;
    float yaw_angle;
} IMU_values;
IMU_values drone_imu;
//Struct for PID values
typedef struct{
    float KP_rate;
    float KI_rate;
    float KD_rate;
    float KP_angle;
    float Prev_error;
    float Integral;
    float Derivative;
} PID;
typedef struct{
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
} OUTPUT;
float PID_calculate(PID *a, float setpoint, float measured_value){
    float error = setpoint - measured_value;
    a->Integral += error * dt;
    a->Integral = constrain(a->Integral, -200, 200); // Anti-windup
    a->Derivative = (error - a->Prev_error) / dt;
    a->Prev_error = error;
    return (a->KP_rate * error) + (a->KI_rate * a->Integral) + (a->KD_rate * a->Derivative);
}
float PID_caculate_angle(PID *a, float setpoint, float measured_value){
    float error = setpoint - measured_value;
    return a->KP_angle * error;
}
PID roll_angle_pid = {1.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0};
PID roll_rate_pid = {1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
PID pitch_angle_pid = {1.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0};
PID pitch_rate_pid = {1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
PID yaw_angle_pid = {1.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0};
PID yaw_rate_pid = {1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
void pid(){
    // PID_caculate_angle(roll_angle_pid, 0,drone_imu.roll_angle );
    // PID_caculate_angle(pitch_angle_pid, 0,drone_imu.pitch_angle );
    // PID_caculate_angle(yaw_angle_pid, 0,drone_imu.yaw_angle );
    float OUTPUT.roll = PID_calculate(roll_rate_pid, 0,PID_caculate_angle(roll_angle_pid, 0,drone_imu.roll_angle ));
    float OUTPUT.pitch = PID_calculate(pitch_rate_pid, 0,PID_caculate_angle(pitch_angle_pid, 0,drone_imu.pitch_angle ));
    float OUTPUT.yaw = PID_calculate(yaw_rate_pid, 0,PID_caculate_angle(yaw_angle_pid, 0,drone_imu.yaw_angle ));
}

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
    //Rate (-Offset)
    drone_imu.roll_rate = (g.gyro.x - gyro_x_offset)*convert;
    drone_imu.pitch_rate = (g.gyro.y - gyro_y_offset)*convert;
    drone_imu.yaw_rate = (g.gyro.z - gyro_z_offset) * convert;
    //Accelerometer Angle (-Offset)
    float acc_x = a.acceleration.x - accel_x_offset;
    float acc_y = a.acceleration.y - accel_y_offset;
    float acc_z = a.acceleration.z;
    //Roll Angle    (-Offset)
    float acc_roll_angle = atan2(acc_y,acc_z)*convert;
    //Pitch Angle    (-Offset)
    float acc_pitch_angle = atan2(-acc_x,sqrt(acc_y*acc_y + acc_z*acc_z)) * convert;
    //Yaw Angle   
    drone_imu.yaw_angle += drone_imu.yaw_rate * dt;
    //Complementary Filter
    float alpha = 0.98;
    drone_imu.roll_angle = alpha * (drone_imu.roll_angle + drone_imu.roll_rate * dt) + (1 - alpha) * acc_roll_angle;
    drone_imu.pitch_angle = alpha * (drone_imu.pitch_angle + drone_imu.pitch_rate * dt) + (1 - alpha) * acc_pitch_angle;
}

void setup(){
    Serial.begin(115200);
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
    
    while(micros() - start < dt){
        // Do nothing, just wait for the next reading
    }
}