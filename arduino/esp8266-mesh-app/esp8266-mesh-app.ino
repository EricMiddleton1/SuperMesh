#include <ESP8266WiFi.h>


#include "Mesh.h"

Mesh mesh;

const int BUTTON = 0;
const unsigned long DEBOUNCE_TIME = 500;

unsigned long buttonTime = 0;

const char* MESH_NAME = "SuperMesh";

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(BUTTON, INPUT);

  Serial.print("[Info] Starting mesh...");
  mesh.begin(MESH_NAME);
  Serial.println("done");
}

void loop() {
  auto curTime = millis();
  
  if(digitalRead(BUTTON) == LOW && curTime >= buttonTime) {
    //mesh.runNetworkTest();

    buttonTime = curTime + DEBOUNCE_TIME;
  }
}

void datagramHandler(std::vector<uint8_t> datagram) {
  Serial.print("[Info] Received datagram of length ");
  Serial.println(datagram.size());
}

