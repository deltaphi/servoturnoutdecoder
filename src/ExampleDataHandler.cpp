#include "ExampleDataHandler.h"

ExampleDataHandler::ExampleDataHandler(uint16_t address,
                                       uint8_t polarize_red_pin,
                                       uint8_t polarize_green_pin)
    : address_(address),
      polarize_red_pin(polarize_red_pin),
      polarize_green_pin(polarize_green_pin) {}

void ExampleDataHandler::init() {
  if (polarize_red_pin != 255) {
    pinMode(polarize_red_pin, OUTPUT);
    digitalWrite(polarize_red_pin, LOW);
  }
  if (polarize_green_pin != 255) {
    pinMode(polarize_green_pin, OUTPUT);
    digitalWrite(polarize_green_pin, LOW);
  }
}

/**
 * \brief Handle a MM message
 *
 * \param address The address of the turnout decoder
 * \param data The addres of the Turnout (bits 0..3) and the key press mode (bit
 * 4)
 *
 * address is the address of a turnout decoder (switch address div 4) -> Not
 * actually hepful? (Range: 0x1..0x4F, 0 -> Wraps around!) 0x1F is relevant data
 * contains several bits: 7 6 5 4 3 2 1 0 7..5 appear to be unused 4 is key
 * pressed (set)/released (cleared) 3..0 are a turnout address. Even numbers are
 * red, odd numbers are green
 */
void ExampleDataHandler::handleEvent(unsigned char decoder_address,
                                     unsigned char data) {
  uint8_t pressed = data & 0x10;

  if (pressed) {
    uint8_t gerade = (data & 0x02);
    data &= 0x0F;
    data >>= 2;

    uint16_t turnout_address =
        (static_cast<uint16_t>(decoder_address) << 2) | data;
    if (decoder_address == 0) {
      turnout_address += 320;
    }
    turnout_address -= 3;

    if (turnout_address == address_) {
      Serial.print(F("Turnout "));
      Serial.print(turnout_address, DEC);
      Serial.print(F(" set to "));
      if (gerade) {
        Serial.print(F(" green"));
      } else {
        Serial.print(F(" red"));
      }
      Serial.println();

      setLed(gerade);
    }
  }
}

void ExampleDataHandler::setLed(uint8_t data) {
  // data = data & 0x02;
  if (data) {
    polarize(RED);
  } else {
    polarize(GREEN);
  }
}

void ExampleDataHandler::polarize(TurnoutDirectionType dir) {
  if (dir == RED) {
    if (polarize_green_pin != 255) {
      Serial.println(F(" Polarize GREEN LOW"));
      digitalWrite(polarize_green_pin, LOW);
    }
    if (polarize_red_pin != 255) {
      Serial.println(F(" Polarize RED HIGH"));
      digitalWrite(polarize_red_pin, HIGH);
    }
  } else {
    if (polarize_red_pin != 255) {
      Serial.println(F(" Polarize RED LOW"));
      digitalWrite(polarize_red_pin, LOW);
    }
    if (polarize_green_pin != 255) {
      Serial.println(F(" Polarize GREEN HIGH"));
      digitalWrite(polarize_green_pin, HIGH);
    }
  }
  lastActionTime = millis();
  active = true;
}

void ExampleDataHandler::checkTimeout() {
  unsigned long now = millis();
  if (active && (now - lastActionTime) >= kRelayOnDuration_ms) {
    handleTimeout();
  }
}

void ExampleDataHandler::handleTimeout() {
  Serial.println(F("Timeout"));
  active = false;
  if (polarize_red_pin != 255) {
    digitalWrite(polarize_red_pin, LOW);
  }
  if (polarize_green_pin != 255) {
    digitalWrite(polarize_green_pin, LOW);
  }
}
