#include <stdint.h>
typedef uint64_t u64;
typedef int8_t   i8;
#include "common/common.h"

// #include "ShiftRegisterPWM.h"

// ShiftRegisterPWM sr(1, 16);

// void setup()
// {
//   pinMode(2, OUTPUT); // sr data pin
//   pinMode(3, OUTPUT); // sr clock pin
//   pinMode(4, OUTPUT); // sr latch pin

//   sr.interrupt(ShiftRegisterPWM::UpdateFrequency::SuperFast);
// }

// // Motor on PIN 5
// // LED on PIN 2
// void loop()
// {
//   uint8_t j = 2;
//   sr.set(j, 165); //165 statt 255
//   delay(1000);

//   for (uint8_t i = 0; i < 8; i++) {
//     sr.set(i, 0); // all off
//   }
//   delay(1000);

//   j = 5;
//   sr.set(j, 195);
//   j = 2;
//   sr.set(j, 255); // Maximal
//   delay(1000); //

//   for (uint8_t i = 0; i < 8; i++) {
//     sr.set(i, 0);
//   }
//   delay(1000);
// }

void setup() {
  // initialize serial port
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
}

ClientMsg readMsg() {
  if (Serial.available()) {
    // Check first byte to see what kind of message this is
    int incomingByte = Serial.read();
    switch (incomingByte) {
      case 69: // Connection check
        Serial.write("Hello!\n");
        Serial.flush();
        break;
      case 1:  // Client sent message
        // TODO
        break;
      case 2:  // Client asks to receive some message
        // TODO
        break;
      default:
        return;
    }
  }
}

void loop() {
  // read the incoming byte:
  ClientMsg msg = readMsg();
  
}
