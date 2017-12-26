#include "Message.h"


Message::Message() {
}

Message::Message(const uint8_t* data, int length, const ChipID& sender, const IPAddress& senderIP)
  : data_(data, data + length)
  , sender_(sender)
  , senderIP_(senderIP) {
}

Message::~Message() {
}

Message::operator bool() const {
  return !data_.empty();
}

int Message::size() const {
  return data_.size();
}

ChipID Message::sender() const {
  return sender_;
}

IPAddress Message::senderIP() const {
  return senderIP_;
}

void Message::write(uint8_t value) {
  data_.push_back(value);
}

void Message::write(const uint8_t* buffer, int length) {
  data_.insert(data_.end(), buffer, buffer + length);
}

const uint8_t* Message::data() const {
  return data_.data();
}

uint8_t* Message::data() {
  return data_.data();
}

Message::const_iterator Message::begin() const {
  return data_.begin();
}

Message::iterator Message::begin() {
  return data_.begin();
}

Message::const_iterator Message::end() const {
  return data_.end();
}

Message::iterator Message::end() {
  return data_.end();
}

