#include <ESP8266WiFi.h>

#include "Mesh.h"

void datagramHandler(Mesh::Datagram datagram);

Mesh mesh{datagramHandler};

void setup() {
  Serial.begin(115200);
  Serial.println();

  Serial.print("[Info] Starting mesh...");
  mesh.begin();
  Serial.println("done");
}

void loop() {
  mesh.run();
}

void datagramHandler(std::vector<uint8_t> datagram) {
  Serial.print("[Info] Received datagram of length ");
  Serial.println(datagram.size());
}

