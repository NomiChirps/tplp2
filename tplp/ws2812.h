#ifndef TPLP_WS2812_H
#define TPLP_WS2812_H

#include "hardware/pio.h"

void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq, bool rgbw);

#endif  // TPLP_WS2812_H