#include <Arduino.h>

#include <Decoder145027.h>
#include <Detect145027.h>

#include "MotorDataHandler.h"

// Globally create a detector.
Detect145027 detector(Detect145027::TURNOUT_NOMINAL_PULSE_WIDTH_MICROS);
Decoder145027 decoder;

void handler() {
  // Do read the pin that corresponds to the interrupt as fast as possible.
  // This is machine-dependent!
  int fastReadResult = PIND & (1 << PD2);

  // Call the appropriate Method on the detector.
  if (fastReadResult != 0) {
    // state is up, must have been leading edge..
    detector.pickUpLeadingEdge();
  } else {
    // must have been trailing edge then...
    detector.pickUpTrailingEdge();
  }
}

// constexpr uint8_t kNumDataHandlers = 2;

// addr, red, green
// ExampleDataHandler handlers[kNumDataHandlers] = {ExampleDataHandler(103, 5,
// 6),
//                                                 ExampleDataHandler(104, 7,
//                                                 8)};

MotorDataHandler dataHandler;

// The setup function is called once at startup of the sketch
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing!");

  pinMode(2, INPUT);

  attachInterrupt(0, &handler, CHANGE);
  // Do not use two interrupt handlers on the same pin!
  //(RISING, FALLING)... That didn't work for me.
  // Use CHANGE instead and do a fast read in the interrupt handler
  // to tell leading from trailing edges!

  // for (uint8_t i = 0; i < kNumDataHandlers; ++i) {
  //  handlers[i].init();
  //  decoder.addHandler(&handlers[i]);
  //}

  // motor 1, motor 2, relay high, relay low
  dataHandler.setPins(9, 10, 11, A3);
  dataHandler.setAddress(103, 104);
  decoder.addHandler(&dataHandler);
}

// The loop function is called in an endless loop
void loop() {
  if (detector.available()) {
    unsigned char* datagram = detector.getCurrentDatagram();

    /*
        Serial.print(F("Received Data: 0x"));
        for (int i = 0; i < 18; i++) {
          Serial.print((int) datagram[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
    */
    decoder.decodeDatagram(datagram);
  }

  dataHandler.checkTimeout();

  //  for (uint8_t i = 0; i < kNumDataHandlers; ++i) {
  //    handlers[i].checkTimeout();
  //  }
}
