#!/usr/bin/env python3

import time
import board
import neopixel_spi as neopixel

COLORS = (0xFF0000, 0x00FF00, 0x0000FF)

class Animations:
    def __init__(self):
        num_pixels = 30
        pixel_order = neopixel.GRB
        spi = board.SPI()
        self.pixels = neopixel.NeoPixel_SPI(spi, num_pixels, pixel_order=pixel_order, 
                                            auto_write=False)
        self.stop_flashing = False

    def solid(self, color):
        self.pixels.fill(color)
        self.pixels.show()

    def flashing(self, color, frequency):
        duty_time = 1 / frequency / 2
        while not self.stop_flashing:
            self.pixels.fill(color)
            self.pixels.show()
            time.sleep(duty_time)
            self.pixels.fill(0)
            self.pixels.show()
            time.sleep(duty_time)
        self.pixels.fill(0)
        self.pixels.show()


try:
    led_animations = Animations()
    #led_animations.solid(COLORS[0])
    led_animations.flashing(COLORS[0], 1)
except KeyboardInterrupt:
    led_animations.solid(0)
