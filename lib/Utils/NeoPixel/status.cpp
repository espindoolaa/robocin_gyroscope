#include "status.h"

static NeoPixelOut* neoPixel;
static Pixel* ledStrip;
static unsigned char numLeds;

void Status::init(PinName pin, bool normalize, double scale, unsigned char NLed) {
  numLeds = NLed;

  ledStrip = new Pixel[numLeds];
  neoPixel = new NeoPixelOut(pin, normalize, scale);

  for (int i = 0; i < numLeds; i++) {
    ledStrip[i].hex = 0x000000;
  }
  neoPixel->send(ledStrip, numLeds);
}

void Status::sendDefault() {
  Pixel* ledStripp = new Pixel[numLeds];
  for (int i = 0; i < numLeds; i++) {
    ledStripp[i].hex = 0xFF0000;
  }
  neoPixel->send(ledStripp, numLeds);
}

void Status::send(int hexColor, LED_POSITION motor) {
  if (motor <= DRIBBLER_LED) {
    ledStrip[(int) motor].hex = hexColor;
  } else {
    for (int i = 0; i < numLeds; ++i)
      ledStrip[i].hex = hexColor;
  }
  neoPixel->send(ledStrip, numLeds);
}

void Status::send(CODE code, LED_POSITION motor) {
  if (motor <= DRIBBLER_LED) {
    ledStrip[(int) motor].hex = getColor(code);
  } else {
    setStripColor(code, ledStrip);
  }
}
void Status::sendAllColor(int hex, int delay) {
  clearColors();
  for (int j = 0; j < numLeds; j++) {
    ledStrip[j].hex = hex;
    neoPixel->send(ledStrip, numLeds);
  }
  ThisThread::sleep_for(chrono::milliseconds(delay));
  clearColors();
}
void Status::sendRoundColor(int hex, int delay) {
  for (int j = 0; j < numLeds; j++) {
    clearColors();
    ledStrip[j].hex = hex;
    neoPixel->send(ledStrip, numLeds);
    ThisThread::sleep_for(chrono::milliseconds(delay));
  }
  clearColors();
}
void Status::clearColors() {
  for (int i = 0; i < numLeds; ++i) {
    ledStrip[i].hex = 0x000000;
  }
  neoPixel->send(ledStrip, numLeds);
}
int Status::getColor(CODE code) {
  return 0x00FF00;
}

void Status::setStripColor(CODE code, Pixel* ledStrip) {
  int color = 0x000000;
  for (int i = 0; i < numLeds; ++i) {
    if (code == FAIL) {
      color = 0xFF0000;
    } else if (code == INIT) {
      color = 0xFFFFFF;
    } else if (code == NOT_CONNECT) {
      color = 0xFF00FF;
    } else if (code == CLEAR) {
      color = 0x000000;
    } else if (code == RUNNING) {
      color = 0x00FF00;
    } else if (code == BOOTING) {
      color = 0xFFFF00;
    } else
      color = 0x000000;

    ledStrip[i].hex = color;
  }
}
