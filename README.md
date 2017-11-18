# CH341 USB to SPI Linux kernel driver

Linux kernel driver for the CH341A SPI interface. It can be used with CH341 USB to UART/I2C/SPI adapter boards to connect SPI slave to a Linux host as SPI master.

The driver uses either a the fast SPI hardware interface that is limited in configuration parameters or a slow SPI bit banging implementation.

## Limitations

The hardware interface implementation is limited to

- SPI mode 0 (CPOL=0, CPHA=0)
- fixed clock frequency of about 1.4 MHz
- low active CS signal
- single bit transfers, and
- 8 bits per word

Because of the very limited documentation and applications that are almost all in Chinese, it is impossible to figure out, whether these parameters can be changed by means of control commands. Therefore you have to live with this configuration at the moment as it is if you want to use the hardware implementation :-(

The bit banging protocol allows following configurations

- SPI mode 1 (CPOL=0, CPHA=1)
- SPI mode 2 (CPOL=0, CPHA=1)
- SPI mode 3 (CPOL=0, CPHA=1)

Both implementations allow the transmission with LSB first (SPI_LSB_FIRST).

The bit banging implementation is very slow. A SCK clock frequency of about 400 kHz can be reached, so that one byte takes around 14 us. However, each byte of a message has to be sent as a separate USB message to the adapter because of the very limited buffer size at the adapter of only 32 byte and the bitwise implementation. This results into a delay of about 6.5 ms between each byte. Additionally, handling the CS signal before the transfer and after the transfer causes an additional delay of about 20 ms. Thus, it takes about 20 ms + n*0.014 ms + (n-1)*6.5 ms to transfer a message of n bytes.

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


## Usage

Once the driver is loaded successfully, it provides three SPI devices on next available SPI bus, e.g.,

```
/dev/spidev0.0
/dev/spidev0.1
/dev/spidev0.2
```

These devices can be used with normal I/O operations like open, ioctl and close to communicate with one of the slaves connected to the SPI.

To open an SPI device simply use:
```
int spi = open("/dev/spidev0.0, O_RDWR));
```

Once the device is opened successfully, you can modify configurations using ```ioctl``` function.

```
uint8_t mode = SPI_MODE_0;
uint8_t lsb = SPI_LSB_FIRST;
...
ioctl(spi, SPI_IOC_WR_MODE, &mode);
ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb);
```
Function ```ioctl``` is also used to tranfser data:

```
uint8_t *mosi; // output data
uint8_t *miso; // input data
...
// fill mosi with output data
...
struct spi_ioc_transfer spi_trans;
memset(&spi_trans, 0, sizeof(spi_trans));

spi_trans.tx_buf = (unsigned long) mosi;
spi_trans.rx_buf = (unsigned long) miso;
spi_trans.len = len;

int status = ioctl (spi_device[bus][cs], SPI_IOC_MESSAGE(1), &spi_trans);

// use input data in miso
```
