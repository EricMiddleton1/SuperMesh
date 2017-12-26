#pragma once

#include <vector>

#include <ESP8266WiFi.h>

#include "Types.h"

class Message {
public:
  using iterator = std::vector<uint8_t>::iterator;
  using const_iterator = std::vector<uint8_t>::const_iterator;

  Message();
  Message(const uint8_t* data, int length, const ChipID& sender = {}, const IPAddress& senderIP = {});
  ~Message();

  operator bool() const;

  int size() const;

  ChipID sender() const;
  IPAddress senderIP() const;

  void write(uint8_t);
  void write(const uint8_t*, int length);

  const uint8_t* data() const;
  uint8_t* data();

  const_iterator begin() const;
  iterator begin();

  const_iterator end() const;
  iterator end();

private:
  std::vector<uint8_t> data_;
  ChipID sender_;
  IPAddress senderIP_;
};
