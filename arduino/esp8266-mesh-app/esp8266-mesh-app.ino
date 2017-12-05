#include <ESP8266WiFi.h>

#include "Mesh.h"

void datagramHandler(Mesh::Datagram datagram);

Mesh mesh{datagramHandler};

const int BUTTON = 0;
const unsigned long DEBOUNCE_TIME = 500;

unsigned long buttonTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(BUTTON, INPUT);

  Serial.print("[Info] Starting mesh...");
  mesh.begin();
  Serial.println("done");
}

void loop() {
  auto curTime = millis();
  
  if(digitalRead(BUTTON) == LOW && curTime >= buttonTime) {
    mesh.runNetworkTest();

    buttonTime = curTime + DEBOUNCE_TIME;
  }
  
  mesh.run();
}

void datagramHandler(std::vector<uint8_t> datagram) {
  Serial.print("[Info] Received datagram of length ");
  Serial.println(datagram.size());
}

