# CH341 USB to SPI Linux kernel driver

Linux kernel driver for the CH341A SPI interface. It can be used with CH341 USB to UART/I2C/SPI adapter boards to connect SPI slave to a Linux host as SPI master.

## Limitations

The driver uses the hardware SPI interface that is limited to

- SPI mode 0 (CPOL=0, CPHA=0)
- fixed clock frequency of about 1.4 MHz
- low active CS signal
- single bit transfers, and
- 8 bits per word

Because of the very limited documentation and applications that are almost all in Chinese, it is impossible to figure out, whether these parameters can be changed by means of control commands. Therefore you have to live with this configuration at the moment as it is :-(

## Hardware configuration

The driver uses following pins of CH341.

| Pin | Name | Direction | Function SPI (CH341) |
| --- | ---- | --------- | -------------------- |
| 18 | D3 | output | SCK (DCK) |
| 20 | D5 | output | MOSI (DOUT)|
| 22 | D7 | input | MISO (DIN) |
| 15 | D0 | output | CS0 |
| 16 | D1 | output | CS1 |
| 17 | D2 | output | CS2 |

With these three CS signals, at most three slaves can be used at the interface.

## SPI master interface

Once the driver is loaded successfully, it provides three SPI devices on next available SPI bus, e.g.,

```
/dev/spidev0.0
/dev/spidev0.1
/dev/spidev0.2
```

These devices can be used with normal I/O operations like open, ioctl and close to communicate with one of the slaves connected to the SPI. 

## Installation

Compile the driver using following commands:
```
git clone https://github.com/gschorcht/spi-ch341-usb.git
cd spi-ch341-usb
make install
```

Driver should be loaded automatically when you connect a device with USB device id ```1a86:5512```. If not try to figure out, whether the USB device is detected properly using

```
lsusb
```
and try to load it manually with
```
insmod spi-ch341-usb.ko
```

To uninstall the module just use
```
make uninstall
```

## Conflicts USB to I2C Linux kernel driver

Since the CH341 provides the I2C interface as USB device with same id, you have to unload the driver module with

```
rmmod spi-ch341-usb
```

before you load the driver module for the I2C interface.


