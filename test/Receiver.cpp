#include <esp_now.h>
#include <WiFi.h>
#include "test\main.cpp"

//INPUT values
typedef struct {
  float roll ;
  float pitch ;
  float yaw;
  float throttle ;   // 0–4095
  bool  armed;
} INPUT;
INPUT input;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&input, incomingData, sizeof(input));  
  Serial.print(" Throttle: ");
  Serial.print(input.throttle);
  Serial.print("   |   ")
  Serial.print("Roll: ");
  Serial.print(input.roll);
  Serial.print("   |   ")
  Serial.print(" Pitch: ");
  Serial.print(input.pitch);
  Serial.print("   |    ")
  Serial.print(" Yaw: ");
  Serial.println(input.yaw);

}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
void loop(){
  
  
  }
  
  // delay(100); // Adjust the delay as needed


