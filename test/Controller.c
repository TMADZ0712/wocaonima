#include <esp_now.h>
#include <WiFi.h>
#include "test\main.cpp"

//Input joystick
#define LX 
#define LY
#define RX
#define RY
#define Button1
#define Button2
//Map value

//INPUT values
typedef struct {
  float roll ;
  float pitch ;
  float yaw;
  float throttle ;   // 0–4095
  bool  armed;
} INPUT;
INPUT input;
void update_input(){
  input.roll = analogRead(RX);
  input.pitch = analogRead(RY);
  input.yaw = analogRead(LX);
  input.throttle = analogRead(LY); 
  input.armed = digitalRead(Button1);
}
//MAC Address of the drone
uint8_t droneMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

esp_now_peer_info_t peer;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peer.peer_addr, droneMAC, 6);
  peer.channel = 0;  
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}
void loop(){
  // Simulate input values (replace with actual input reading)
  input.roll = 10.0;  
  input.pitch = 5.0;  
  input.yaw = 15.0;   
  input.throttle = 500; 
  input.armed = true; 

  // Send the data to the drone
  esp_err_t result = esp_now_send(droneMAC, (uint8_t *)&input, sizeof(input));
  
  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data"); 
  }
  
  // delay(100); // Adjust the delay as needed
}

