#include "Radio.h"

void Radio::sendID() {
  for (byte mask = 1; mask>0; mask<<= 1) {
    sendBit(id & mask);
  }
};

void Radio::startSend() {
  sendBit(LOW);
  sendBit(LOW);
  sendBit(LOW);
  sendID();
};

void Radio::endSend() {
  sendBit(LOW);
  sendBit(LOW);
  sendBit(LOW);
};

void Radio::sendBit(bool b) {
  startTime = micros();
  digitalWrite(sendPin, b);
  while (micros() - startTime <= bitDelay);
};

void Radio::Send(word data) {
  for (word mask = 1; mask > 0; mask <<=1) {
    sendBit(data & mask);
  }
};

int Radio::Listen() {
  startTime = micros();
  return 0;
};

void Radio::Test() {
  bool STATE = HIGH;
  for (int i=0; i<10; i++) {
    digitalWrite(sendPin, STATE);
    STATE = !STATE;
    delay(1000);
  }
};
