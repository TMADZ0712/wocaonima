#include <Arduino.h>
#include "Wire.h"
#include "MPU6050.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float AccX,AccY,AccZ;
float RateRoll,RatePitch,RateYaw;

int16_t ax,ay,az;
int16_t gx,gy,gz;

MPU6050 mpu;
void setup() {
   Wire.begin(21,22);
  
  //Low Pass Filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);

  //Accelerometer sensitivity
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  //Gyroscope sensitivity
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();


  Serial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED FAILED");
    while(true);
  }
  Serial.println("Bắt đầu mpu6050. ");
  mpu.initialize();
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  } 
  else{
    Serial.println("MPU6050 connection successful");
  }
}

void loop() {
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  RateRoll = gx/65.5;
  RatePitch = gy/65.5;
  RateYaw = gz/65.6;
  
  AccX = ax/4096;
  AccY = ay/4096;
  AccZ = az/4096;

  Serial.print(ax); 
  Serial.print(ay); 
  Serial.print(az); 
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  
  display.print("Roll: ");
  display.println(RateRoll);


  display.print("Pitch: ");
  display.println(RatePitch);


  display.print("Yaw: ");
  display.println(RateYaw);

  display.display();
}
