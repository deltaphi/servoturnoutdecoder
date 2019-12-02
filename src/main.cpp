#include <Arduino.h>


#include <Decoder145027.h>
#include <Detect145027.h>

#include "ExampleDataHandler.h"

//Globally create a detector.
Detect145027 detector(Detect145027::TURNOUT_NOMINAL_PULSE_WIDTH_MICROS);
Decoder145027 decoder;

void dumpServos();

void handler() {

  //Do read the pin that corresponds to the interrupt as fast as possible.
  //This is machine-dependent!
  int fastReadResult = PIND & (1 << PD2);

  //Call the appropriate Method on the detector.
  if (fastReadResult != 0) {
    //state is up, must have been leading edge..
    detector.pickUpLeadingEdge();
  } else {
    //must have been trailing edge then...
    detector.pickUpTrailingEdge();
  }
}

constexpr uint8_t kNumDataHandlers = 6;
uint8_t servo_index;

/* Left board */
// addr, pin, red, green
ExampleDataHandler handlers[kNumDataHandlers] = {
  ExampleDataHandler(100, 9, 75, 120), // done
  ExampleDataHandler(101, 10, 55, 110), // done
  ExampleDataHandler(102, 11, 50, 125), // done
  ExampleDataHandler(103, A5, 40, 100, 5, 6), // done
  ExampleDataHandler(104, A4, 85, 150, 7, 8),
  ExampleDataHandler(105, A3, 70, 140)  // N/A
};
/**/

/* Right board
// addr, pin, red, green
ExampleDataHandler handlers[kNumDataHandlers] = {
  ExampleDataHandler(109, 9, 30, 115), // done   - Turnout 9
  ExampleDataHandler(2, 10, 55, 110), // pin broken
  ExampleDataHandler(106, 11, 50, 100), // done                 - Turnout 6
  ExampleDataHandler(107, A5, 105, 60, 5, 6), // done           - Turnout 7
  ExampleDataHandler(108, A4, 60, 120, 7, 8), // done           - Turnout 8
  ExampleDataHandler(110, A3, 110, 50)  // done                - Turnout 10
};
/**/

//The setup function is called once at startup of the sketch
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing!");

  pinMode(2, INPUT);

  attachInterrupt(0, &handler, CHANGE);
  //Do not use two interrupt handlers on the same pin!
  //(RISING, FALLING)... That didn't work for me.
  //Use CHANGE instead and do a fast read in the interrupt handler
  //to tell leading from trailing edges!

  for (uint8_t i = 0; i < kNumDataHandlers; ++i) {
    handlers[i].init();
    decoder.addHandler(&handlers[i]);
  }
  servo_index = kNumDataHandlers;
  dumpServos();
}

void dumpServos() {
  for (uint8_t i = 0; i < kNumDataHandlers; ++i) {
    Serial.print(", Servo ");
    Serial.print(i+1, DEC);
    Serial.print(": ");
    Serial.print(handlers[i].servo_pos, DEC);
  }
  Serial.println(".");
}

String readString;

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

  //Serial.write("\n");
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      Serial.write("Read String: '");
      Serial.print(readString);
      
      int value = readString.toInt();

      if (servo_index == kNumDataHandlers) {
        // No servo selected
        if (value <= kNumDataHandlers) {
          // Switch to servo control mode
          servo_index = value - 1;
        } else {
          // Explicit turnout toggle mode. Addresses are 11/12, 21/22, 31/32, 41/42, 51/52, 61/62
          // odd is green
          uint8_t turnout = value / 10;
          if (0 < turnout && turnout <= kNumDataHandlers) {
            turnout -= 1;

           uint8_t gerade = value % 2;
           handlers[turnout].setLed(gerade);
          }
        }
      } else {
        // Servo selected
        if (value > 180) {
          // Invalid servo angle, switch to no-servo-selected mode
          servo_index = kNumDataHandlers;
        } else {
          handlers[servo_index].servo_pos = value;
          handlers[servo_index].updateServo();
        }
      }
      
      Serial.write("', Servo ");
      Serial.print(servo_index + 1, DEC);
      Serial.write(" at position ");
      if (servo_index < kNumDataHandlers) {
        Serial.println(handlers[servo_index].servo_pos, DEC);
      } else {
        Serial.println("N/A.");
      }
      
      readString = "";
      dumpServos();
    } else {
      readString += c;
    }
  }

  for (uint8_t i = 0; i < kNumDataHandlers; ++i) {
    handlers[i].checkTimeout();
  }

}
