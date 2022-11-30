#!/usr/bin/env python3

import time
import threading
import board
import neopixel_spi as neopixel

class Animations:
    def __init__(self):
        num_pixels = 60
        pixel_order = neopixel.GRB
        spi = board.SPI()
        self.pixels = neopixel.NeoPixel_SPI(spi, num_pixels, pixel_order=pixel_order, 
                                            auto_write=False)
        self.flashing_thread = threading.Thread(target=self.flashing_animation, args=(0, 1))
        self.stop_flashing = False

    def solid(self, color):
        while self.flashing_thread.is_alive():
            time.sleep(0.1)
        self.pixels.fill(color)
        self.pixels.show()

    def flashing(self, color, frequency):
        self.flashing_thread = threading.Thread(target=self.flashing_animation, args=(color, frequency))
        self.flashing_thread.start()

    def flashing_animation(self, color, frequency):
        duty_time = 1 / frequency / 2
        while not self.stop_flashing:
            self.pixels.fill(color)
            self.pixels.show()
            time.sleep(duty_time)
            self.pixels.fill(0)
            self.pixels.show()
            time.sleep(duty_time)
        self.stop_flashing = False


if __name__ == "__main__":
    COLORS = (0xFF0000, 0x00FF00, 0x0000FF)
    try:
        led_animations = Animations()
        while True:
            led_animations.solid(COLORS[0])
            time.sleep(5)
            led_animations.solid(COLORS[2])
            time.sleep(5)
            led_animations.flashing(COLORS[1], 2)
            time.sleep(5)
            led_animations.stop_flashing = True
            led_animations.solid(0)
    except KeyboardInterrupt:
        led_animations.solid(0)
