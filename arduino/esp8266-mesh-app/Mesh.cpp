#include "Mesh.h"

Mesh::Mesh(const DatagramHandler& _handler)
  : handler{_handler}
  , nextHelloTime{0}
  , nextRouteTime{0} {
}

void Mesh::begin() {
  WiFi.disconnect();
  
  Router::ChipID myID;
  
  WiFi.macAddress(myID.data());
  router.begin(myID);

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(String("esp8266-mesh ") + Router::chipIDToString(myID));
}

void Mesh::run() {
  auto curTime = millis();

  if(curTime >= nextHelloTime) {
    
  }
  
  if(curTime >= nextRouteTime) {
    if(WiFi.status() != WL_CONNECTING) {
      WiFi.scanNetworksAsync([this](int networkCount) {
          processWiFiScan(networkCount);
        });
    }
  }
}

