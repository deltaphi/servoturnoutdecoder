#ifndef __EXAMPLEDATAHANDLER_H__
#define __EXAMPLEDATAHANDLER_H__

#include <Arduino.h>

#include <DataHandlerInterface.h>

constexpr long kRelayOnDuration_ms = 20; 

/*
 * \brief Class ExampleDataHandler
 */    
class ExampleDataHandler: public DataHandlerInterface {
public:
  const uint16_t address_;

  uint8_t polarize_red_pin;
  uint8_t polarize_green_pin;

  bool active = false;
  unsigned long lastActionTime;

  enum TurnoutDirectionType {
    RED, GREEN
  };

  ExampleDataHandler(uint16_t address, uint8_t polarize_red_pin, uint8_t polarize_green_pin);
  void init();
  void handleEvent(unsigned char decoder_address, unsigned char data) override;
  void setLed(uint8_t data);
  void polarize(TurnoutDirectionType dir);
  void updateServo();
  void checkTimeout();
  void handleTimeout();
  
};

#endif  // __EXAMPLEDATAHANDLER_H__
