#include "MotorDataHandler.h"

void MotorDataHandler::setPins(Pin_t motorPin1, Pin_t motorPin2,
                               Pin_t relayHighPin, Pin_t relayLowPin) {
  this->motorPin1 = motorPin1;
  pinMode(this->motorPin1, OUTPUT);
  analogWrite(this->motorPin1, 0);

  this->motorPin2 = motorPin2;
  pinMode(this->motorPin2, OUTPUT);
  analogWrite(this->motorPin2, 0);

  this->relayHighPin = relayHighPin;
  pinMode(this->relayHighPin, OUTPUT);
  digitalWrite(relayHighPin, LOW);

  this->relayLowPin = relayLowPin;
  pinMode(this->relayLowPin, OUTPUT);
  digitalWrite(relayLowPin, LOW);
}

void MotorDataHandler::setSpeed(Speed_t newSpeed) {
  Pwm_t lowValue = 0;
  Pwm_t highValue;
  uint8_t relayValue = LOW;

  switch (newSpeed) {
    case Speed_t::OFF:
      highValue = kPwmOff;
      relayValue = HIGH;
      break;
    case Speed_t::SLOW:
      highValue = kPwmSlow;
      relayValue = HIGH;
      break;
    case Speed_t::FULL:
      highValue = kPwmFull;
      relayValue = LOW;
      break;
  }

  if (relayValue == LOW) {
    digitalWrite(relayHighPin, relayValue);
  }

  if (direction == Direction_t::RIGHT) {
    analogWrite(motorPin1, lowValue);
    analogWrite(motorPin2, highValue);
  } else {
    analogWrite(motorPin1, highValue);
    analogWrite(motorPin2, lowValue);
  }

  if (relayValue == HIGH) {
    // Briefly wait for the PWM to apply before switching the relay
    delay(10);
    digitalWrite(relayHighPin, relayValue);
  }

  speed = newSpeed;
  lastSpeedChange = millis();
}

void MotorDataHandler::handleEvent(unsigned char address, unsigned char data) {
  /*
  Serial.print(F("Received apacket for "));
  Serial.print((uint8_t)address, DEC);
  Serial.print(F(" with data 0x"));
  Serial.print(data, BIN);
  Serial.println(".");
  */
  uint8_t pressed = data & 0x10;
  if (!pressed) {
    return;
  }

  uint8_t gerade = (data & 0x02);
  data &= 0x0F;
  data >>= 2;

  uint16_t turnout_address = (static_cast<uint16_t>(address) << 2) | data;
  // Possibly missing a warparound from 0 to 320. Then again, we can't handle
  // 320 in an unsigned char anyways.
  if (address == 0) {
    turnout_address += 320;
  }
  turnout_address -= 3;
  /*
  Serial.print("Decoded address: ");
  Serial.println(turnout_address, DEC);
  */
  if (turnout_address == directionAddress) {
    Serial.print(F("Setting Motor direction to "));
    if (gerade) {
      Serial.println("LEFT.");
      setDirection(Direction_t::LEFT);
    } else {
      Serial.println("RIGHT.");
      setDirection(Direction_t::RIGHT);
    }
  } else if (turnout_address == relayAddress) {
    if (gerade) {
      // make it go
      Serial.println(F("Activating relay. Setting Motor speed to FULL."));
      setSpeed(Speed_t::FULL);
    } else {
      Serial.println(F("Deactivating relay. Setting Motor speed to SLOW."));
      setSpeed(Speed_t::SLOW);
    }
  }
}

void MotorDataHandler::setDirection(Direction_t direction) {
  if (this->direction != direction) {
    if (speed != Speed_t::OFF) {
      // There is a direction change in full flight going on. Stop the motor
      // first.
      setSpeed(Speed_t::OFF);
      delay(100);
    }
    this->direction = direction;
  }
}

void MotorDataHandler::checkTimeout() {
  if (speed != Speed_t::SLOW) {
    // only slow has a timeout value
    return;
  }

  unsigned int now = millis();
  if (now - lastSpeedChange > kLowSpeedTimeout) {
    Serial.println(F("Low Timeout expired. Setting Motor speed to OFF."));
    setSpeed(Speed_t::OFF);
  }
}