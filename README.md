# LED_controller

## Setup
### Enable SPI
1. `sudo /opt/nvidia/jetson-io/jetson-io.py`
2. Select *Configure 40-pin expansion header*
3. Select *spi1 (19, 21, 23, 24, 26)*
4. Select Save pin changes and then Save and reboot to reconfigure pins.
5. After the Nano boots up again, verify you have the I2C devices with the command `ls /dev/i2c* /dev/spi*`
6. Set user permissions
    ```bash
    sudo groupadd -f -r gpio
    sudo usermod -a -G gpio <user>
    ```
7. Copy rules
    ```bash
    99-gpio.rules /etc/udev/rules.d
    ```
8. Reboot

### Wiring
Wire the NeoPixel's `DIN` pin to the Nano's `SPI_1_MOSI` pin (19)
