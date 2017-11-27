#include <ESP8266WiFi.h>

#include "Router.h"

Router::ChipID myID;

Router router;

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  WiFi.macAddress(myID.data());
  router.begin(myID);

  Router::Route n1{{{0, 0, 0, 0, 0, 1}, 10}, IPAddress().fromString("192.168.1.1")};
  Router::Route n2{{{0, 0, 0, 0, 0, 2}, 15}, IPAddress().fromString("192.168.1.2")};
  Router::Route n3{{{0, 0, 0, 0, 0, 3}, 5}, IPAddress().fromString("192.168.1.3")};
  Router::Route n4{{{0, 0, 0, 0, 0, 4}, 10}, IPAddress().fromString("192.168.1.4")};

  router.updateNeighbors({n1, n2});

  Serial.println("[Info] Updating routing table...");
  auto startTime = micros();
  router.updateRoutingTable();
  auto duration = micros() - startTime;
  Serial.print("[Info] Completed in "); Serial.print(duration); Serial.println("us");
}

void loop() {
  // put your main code here, to run repeatedly:

}
