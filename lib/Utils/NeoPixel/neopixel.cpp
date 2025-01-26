#include "mbed.h"
#include "neopixel.h"

NeoPixelOut::NeoPixelOut(PinName pin, bool normalize, double scale) : DigitalOut(pin) {
  normalize = normalize;
  global_scale = scale;
}

// The timing should be approximately 800ns/300ns, 300ns/800ns
void NeoPixelOut::byte(register uint32_t byte) {
  for (int i = 0; i < 8; i++) {
    gpio_write(&gpio, 1);
    // duty cycle determines bit value
    if (byte & 0x80) {
      // one
      for (int j = 0; j < 73; j++)
        __ASM("NOP\n\t");

      gpio_write(&gpio, 0);
      for (int j = 0; j < 36; j++)
        __ASM("NOP\n\t");
    } else {
      // zero
      for (int j = 0; j < 36; j++)
        __ASM("NOP\n\t");

      gpio_write(&gpio, 0);
      for (int j = 0; j < 73; j++)
        __ASM("NOP\n\t");
    }

    byte = byte << 1; // shift to next bit
  }
}

void NeoPixelOut::send(Pixel* colors, uint32_t count, bool flipwait) {
  // Disable interrupts in the critical section
  __disable_irq();

  Pixel* rgb;
  float fr, fg, fb;
  for (unsigned int i = 0; i < count; i++) {
    rgb = colors++;
    fr = (int) rgb->r;
    fg = (int) rgb->g;
    fb = (int) rgb->b;

    if (normalize) {
      float scale = 255.0f / (fr + fg + fb);
      fr *= scale;
      fg *= scale;
      fb *= scale;
    }

    fr *= global_scale;
    fg *= global_scale;
    fb *= global_scale;

    if (fr > 255)
      fr = 255;
    if (fg > 255)
      fg = 255;
    if (fb > 255)
      fb = 255;
    if (fr < 0)
      fr = 0;
    if (fg < 0)
      fg = 0;
    if (fb < 0)
      fb = 0;

// Black magic to fix distorted timing
#ifdef __HAL_FLASH_INSTRUCTION_CACHE_DISABLE
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
#endif

    byte((int) fg);
    byte((int) fr);
    byte((int) fb);

#ifdef __HAL_FLASH_INSTRUCTION_CACHE_ENABLE
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
#endif
  }

  __enable_irq();

  if (flipwait)
    flip();
}

void NeoPixelOut::flip(void) {
  wait_us(50);
}
