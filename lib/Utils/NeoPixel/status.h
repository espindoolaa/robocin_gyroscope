#ifndef STATUS_H
#define STATUS_H

#include "mbed.h"
#include "neopixel.h"
#include "colorspace.h"

enum LED_POSITION {
  MOTOR_1_LED = 0,
  MOTOR_2_LED,
  MOTOR_3_LED,
  MOTOR_4_LED,
  DRIBBLER_LED,
  ALL_LEDS
};

enum CODE { INIT = 0, FAIL, NOT_CONNECT, CLEAR, RUNNING, WAIT, BOOTING };

class Status {
 public:
  static void
  init(PinName pin, bool normalize = false, double scale = 0.5, unsigned char NLeds = 5);
  static void send(CODE status, LED_POSITION motor = ALL_LEDS);
  static void send(int hexColor, LED_POSITION motor = ALL_LEDS);
  static void sendAllColor(int hex, int delay);
  static void sendRoundColor(int hexColor, int delay);
  static void clearColors();
  static void sendDefault();

 private:
  static int getColor(CODE status);
  static void setStripColor(CODE status, Pixel* strip);
};

#endif
