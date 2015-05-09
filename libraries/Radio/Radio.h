#include <Arduino.h>
class Radio {
  unsigned long bitDelay; // in us
  unsigned long startTime;
  int sendPin;
  int id;
  void sendID();
  void startSend();
  void endSend();
  void sendBit(bool b);
 public:
  int data;
  Radio(int sendPin, int id, unsigned long bitDelay) {
    this->bitDelay = bitDelay;
    pinMode(sendPin, OUTPUT);
    this->id = id;
    this->sendPin = sendPin;
  };
  void Send(word data);
  int Listen();
  void Test();
};
