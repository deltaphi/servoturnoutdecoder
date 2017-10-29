#include <Servo.h>

#include <DataHandlerInterface.h>
#include <Decoder145027.h>
#include <Detect145027.h>
#include <ProgrammableTurnoutDataHandler.h>
#include <ProgrammableTurnoutDataHandlerNormalState.h>
#include <ProgrammableTurnoutDataHandlerProgState.h>
#include <ProgrammableTurnoutDataHandlerStateInterface.h>
#include <TurnOutDataHandler.h>


/*
 * 2013-03-31: Ulrich Schwenk
 * You may use this software under the terms of LGPLv3 or any newer version of the LGPL.
 */


//#include <avr/io.h>
//#include <math.h>

//Globally create a detector.
Detect145027 detector(Detect145027::TURNOUT_NOMINAL_PULSE_WIDTH_MICROS);
Decoder145027 decoder;

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

class ExampleDataHandler: public DataHandlerInterface {
public:
  const uint8_t address_;
  const uint8_t servo_pin;
  uint8_t red_angle;
  uint8_t green_angle;
  Servo servo_;
  int servo_pos;

  ExampleDataHandler(uint8_t address, uint8_t servo_pin, uint8_t red_angle, uint8_t green_angle):
  address_(address), servo_pin(servo_pin), red_angle(red_angle), green_angle(green_angle), servo_pos(green_angle) {
  }

  void init() {
    // TODO: Set PWM to 20ms
    servo_.attach(servo_pin);
    updateServo();
  }

  void handleEvent(unsigned char address, unsigned char data) override {
    if (address == address_) {
      Serial.print(F("ServoHandler ("));
      Serial.print(address_, DEC);
      Serial.print(F(") received data: 0x"));
      Serial.println(data, HEX);
      // 0x02 = green
      // 0x00 = read
      // 0x10 = user pressed button
      // 0x00 = user released button

      setLed(data);
    }
  }

  void setLed(uint8_t data) {
    data = data & 0x02;
    if (data) {
      servo_pos = red_angle;
    } else {
      servo_pos = green_angle;
    }
    updateServo();
  }

  void updateServo() {
    servo_.write(servo_pos);
  }
};

constexpr uint8_t kNumDataHandlers = 6;
uint8_t servo_index;

// addr, pin, red, green
ExampleDataHandler handlers[kNumDataHandlers] = {
  ExampleDataHandler(1, 9, 80, 120), // Old: (70..105..140)
  ExampleDataHandler(2, 10, 70, 110), // done
  ExampleDataHandler(3, 11, 60, 125), // done
  ExampleDataHandler(4, A5, 50, 100), // done
  ExampleDataHandler(5, A4, 90, 140),
  ExampleDataHandler(6, A3, 70, 140)  // N/A
};

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


    Serial.print("Received Data:");
    for (int i = 0; i < 18; i++) {
      Serial.print((int) datagram[i]);
    }
    Serial.println();

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

  //dataHandler.handleTimeouts();

}

/*
void moveServo(int destination) {
  bool dir = (destination > pos);

  // start by skipping 20 steps
  if (dir) {
    pos += 20;
  } else {
    pos -= 20;
  }
  
  int startPos = pos;
  long startTime = millis();
  //long maxTime = abs(destination - pos) * kMsPerStep;
  long delta_time = 0;

  while (
    (dir && pos < destination) || (!dir && pos > destination)
    ) {
    // randomly expect an iteration to be 10ms
    long now = millis();
    delta_time = now - startTime;
    int delta_steps = delta_time / kMsPerStep;
    if (dir) {
    pos = startPos + delta_steps;
    } else {
      pos = startPos - delta_steps;
    }
    myservo.write(pos);
  }
  
  pos = destination;
}*/
