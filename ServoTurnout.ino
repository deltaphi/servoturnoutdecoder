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

/*
template<typename T>
class optional {
  private:
    char[sizeof(T)] data_;
  public:
    template<Args... args>
    void emplace(Args... args) {
      new (static_cast<T>(data_)) (std::forward<Args>(args...));
    }

    void reset() {
      static_cast<T>(data_).~T();
    }
}
*/

template<uint8_t led_pin, uint8_t servo_pin, uint8_t lower_angle, uint8_t upper_angle>
class ExampleDataHandler: public DataHandlerInterface {
public:
  uint8_t address_;
  Servo servo_;
  int servo_pos;

  ExampleDataHandler(uint8_t address): address_(address), servo_pos(lower_angle) {
    pinMode(led_pin, OUTPUT);
    pinMode(servo_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
  //init();
  }

  void init() {
    // TODO: Set PWM to 20ms
    servo_.attach(servo_pin);
    updateServo();
  }

  void handleEvent(unsigned char address, unsigned char data) override {
    if (address == address_) {
      Serial.print(F("ExampleDataHandler ("));
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
      digitalWrite(led_pin, HIGH);
      servo_pos = upper_angle;
    } else {
      digitalWrite(led_pin, LOW);
      servo_pos = lower_angle;
    }
    updateServo();
  }

  void updateServo() {
    servo_.write(servo_pos);
  }
};

ExampleDataHandler<13, 9, 70, 120> dataHandler(1);

unsigned long int lastTimeMillis = 0;

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

  decoder.addHandler(&dataHandler);
  dataHandler.init();
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

  Serial.write("Read String: '");
  Serial.print(readString);
  Serial.write("', Servo: ");
  Serial.println(dataHandler.servo_pos);
  //Serial.write("\n");
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      dataHandler.servo_pos = readString.toInt();
      dataHandler.updateServo();
      readString = "";
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
