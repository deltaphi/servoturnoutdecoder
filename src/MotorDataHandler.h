#ifndef __MOTORDATAHANDLER_H__
#define __MOTORDATAHANDLER_H__

#include <Arduino.h>

#include "DataHandlerInterface.h"

/*
 * \brief Class MotorDataHandler
 */
class MotorDataHandler : public DataHandlerInterface {
 public:
  using Pin_t = uint8_t;
  using Pwm_t = uint8_t;
  using Address_t = unsigned char;

  static constexpr const Pin_t kNoPin = 255;

  static constexpr const Pwm_t kPwmOff = 0;
  static constexpr const Pwm_t kPwmSlow = 200;
  static constexpr const Pwm_t kPwmFull = 255;
  static constexpr const unsigned int kLowSpeedTimeout = 4000;

  enum class Speed_t { OFF = 0, SLOW, FULL };

  enum class Direction_t { RIGHT = 0, LEFT = 1 };

  void setAddress(Address_t relay, Address_t direction) {
    relayAddress = relay;
    directionAddress = direction;
  }

  void setPins(Pin_t motorPin1, Pin_t motorPin2, Pin_t relayHighPin,
               Pin_t relayLowPin);
  void setSpeed(Speed_t newSpeed);
  void setDirection(Direction_t direction);

  void handleEvent(unsigned char address, unsigned char data) override;

  void checkTimeout();

 private:
  Address_t directionAddress;
  Address_t relayAddress;
  Pin_t motorPin1 = kNoPin;
  Pin_t motorPin2 = kNoPin;
  Pin_t relayHighPin = kNoPin;
  Pin_t relayLowPin = kNoPin;
  Direction_t direction = Direction_t::RIGHT;
  Speed_t speed = Speed_t::OFF;

  unsigned int lastSpeedChange;
};

#endif  // __MOTORDATAHANDLER_H__
