# CH341A USB to SPI and GPIO Linux kernel driver

The driver can be used with CH341A USB to UART/I2C/SPI adapter boards to connect SPI slaves to a Linux host. It uses either the **fast SPI hardware interface** which is, however, limited to SPI mode 0 **or** a **slow SPI bit banging implementation**.

Additionally, CH341A data pins that are not used for the SPI interface can be configured as **GPIO** pins. The driver can generate **software interrupts** for all input pins. **One input** pin can be connected with the CH341A interrupt pin to generate **hardware interrupts**. However, since USB is an asynchronous communication system, it is not possible to guarantee exact timings for GPIOs and interrupts.

## Limitations of the SPI interface

The **SPI hardware interface** implementation is limited to

- **SPI mode 0** (CPOL=0, CPHA=0)
- fixed clock frequency of about **1.4 MHz**,
- low active CS signal,
- single bit transfers,
- 8 bits per word, and
- 3 slaves at maximum.

Because of the very limited documentation and applications that are almost all in Chinese, it is impossible to figure out whether these parameters can be changed by means of control commands. Therefore you have to live with this configuration as it is if you want to use the hardware implementation :-(

The **bit banging implementation** allows the following SPI modes

- SPI mode 1 (CPOL=0, CPHA=1)
- SPI mode 2 (CPOL=0, CPHA=1)
- SPI mode 3 (CPOL=0, CPHA=1)

as well as high active CS signals. It is **very slow**. Only a SCK clock frequency of about 400 kHz can be reached, so that one byte takes around 14 us. However, each byte of a message has to be sent as a separate USB message to the adapter because of its bitwise implementation and the very limited USB endpoint buffer sizes. This results into a delay of about 6.5 ms between each byte. Additionally, handling the CS signal before the transfer and after the transfer causes an additional delay of about 20 ms. Thus, it takes about 

       20 ms + n * 0.014 ms + (n-1) * 6.5 ms
    
to transfer a message of n bytes.

Both implementations allow the transmission with MSB first (```SPI_MSB_FIRST```) and LSB first (```SPI_LSB_FIRST```).

## SPI configuration

The driver uses following CH341A pins for the SPI interface.

| Pin | Name | Direction | Function SPI (CH341A) |
| --- | ---- | --------- | --------------------- |
| 18 | D3 | output | SCK (DCK) |
| 20 | D5 | output | MOSI (DOUT)|
| 22 | D7 | input | MISO (DIN) |
| 15 | D0 | output | CS0 |
| 16 | D1 | output | CS1 |
| 17 | D2 | output | CS2 |

With these three CS signals, three slaves can be used at maximum at the interface.

## GPIO configuration

Five of the data pins can be configured as GPIO pins if they are not used for the SPI interface or as chip select signals:

| Pin | Name | SPI Function (default) | Configurable as (**CH341 default in bold face**)   |
| --- | ---- | ---------------------- | ------------------ |
| 15  | D0   | CS0                    | Input, Output, **CS**  |
| 16  | D1   | CS0                    | Input, Output, **CS**  |
| 17  | D2   | CS0                    | Input, Output, **CS**  |
| 19  | D4   | OUT2                   | Input, **Output**      |
| 21  | D6   | IN2                    | **Input**              |

**Please note:** 
- Direction of pins that are configured as input or output can be changed during runtime.
- Pin 21 (D6/IN2) can only be configured as input. It's direction can't be changed during runtime.
- At least one of the CS data pins D0...D2 has to be configured as CS signal.
- One of the inputs can be configured to generate **hardware interrupts for rising edges** of signals. For that purpose, the pin has to be connected with the CH341A **INT** pin 7. 


## Installation of the driver

#### Prerequisites

To compile the driver, you must have installed current **kernel header files**. 

Even though it is not mandatory, it is highly recommended to use **DKMS** (dynamic kernel module support) for the installation of the driver. DKMS allows to manage kernel modules whose sources reside outside the kernel source tree. Such modules are then automatically rebuilt when a new kernel version is installed.

To use DKMS, it has to be installed before, e.g., with command
```
sudo apt-get install dkms
```
on Debian based systems.

#### Installaton

The driver can be compiled with following commands:

```
git clone https://github.com/gschorcht/spi-ch341-usb.git
cd spi-ch341-usb
make

sudo make install
```

If **DKMS** is installed (**recommended**), command ```sudo make install``` adds the driver to the DKMS tree so that the driver is recompiled automatically when a new kernel version is installed.

In case you have not installed DKMS, command ```sudo make install``` simply copies the driver after compilation to the kernel modules directory. However, the module will not be loadable anymore and have to be recompiled explicitly when kernel version changes.

If you do not want to install the driver in the kernel directory at all because you only want to load it manually when needed, simply omit the ```sudo make install```.

#### Loading

Once the driver is installed, it should be loaded automatically when you connect a device with USB device id ```1a86:5512```. If not try to figure out, whether the USB device is detected correctly using command

```
lsusb
```
and try to load it manually with command:
```
insmod spi-ch341-usb.ko
```

#### Uninstallation

To uninstall the module simply use command
```
make uninstall
```
in the source directory.

#### Conflicts with CH341A USB to I2C Linux kernel driver

Since the CH341A also provides an I2C interface as USB device with same id, you have to unload the driver module with

```
rmmod spi-ch341-usb
```

before you can load the driver module for the I2C interface.

## Configuration of the driver

Per default, the driver configures the GPIOs as following and polls the inputs with a default rate of 100 Hz and 10 ms period, respectively.

| Pin | SPI Function | GPIO function | GPIO name | IRQ      |
| --- | ------------ | --------------|---------- | ---------|
| 15  | CS0          | -             | -         | -        |
| 16  | CS1          | -             | -         | -        |
| 17  | CS2          | -             | -         | -        |
| 19  | -            | Input         | gpio4     | hardware |
| 21  | -            | Input         | gpio5     | software |

GPIO configuration as well their polling rate can be changed according to your requirements. The direction of GPIO pins configured as inputs or outputs can be changed during runtime.

### GPIO configuration

To change **GPIO configuration**, simply change the variable ```ch341_board_config``` that should be self-explaining. This variable contains structured entries for each configurable pin. Each entry consists of the pin number, the GPIO mode used for the pin, the name used for the GPIO in the Linux host and a flag whether the pin is connected with the CH341A hardware interrupt pin **INT**. Default configuration is:

```
struct ch341_pin_config ch341_board_config[CH341_GPIO_NUM_PINS] = 
{
    // pin  GPIO mode           GPIO name   hwirq
    {   15, CH341_PIN_MODE_CS , "cs0"     , 0 }, // used as CS0
    {   16, CH341_PIN_MODE_CS , "cs1"     , 0 }, // used as CS1
    {   17, CH341_PIN_MODE_CS , "cs2"     , 0 }, // used as CS2
    {   19, CH341_PIN_MODE_IN , "gpio4"   , 1 }, // used as input with hardware IRQ
    {   21, CH341_PIN_MODE_IN , "gpio5"   , 0 }  // used as input
};
```
In this configuration, pins 15 to 17 are used as CS signals while pin 19 and 21 are used as inputs. Additionally, pin 19 is connected with the CH341A hardware interrupt pin **INT** that produces hardware interrupts on rising edge of the signal connected to pin 19.

To define a pin as output, simply change the GPIO mode to ```CH341_PIN_MODE_OUT```. For example, if you would like to configure only one CS signal and the other CS signal pins as GPIO outputs, the configuration could look like the following:

```
struct ch341_pin_config ch341_board_config[CH341_GPIO_NUM_PINS] = 
{
    // pin  GPIO mode           GPIO name   hwirq
    {   15, CH341_PIN_MODE_CS , "cs0"     , 0 }, // used as CS0
    {   16, CH341_PIN_MODE_OUT, "gpio2"   , 0 }, // used as output
    {   17, CH341_PIN_MODE_OUT, "gpio3"   , 0 }, // used as output
    {   19, CH341_PIN_MODE_IN , "gpio4"   , 1 }, // used as input with hardware IRQ
    {   21, CH341_PIN_MODE_IN , "gpio5"   , 0 }  // used as input
};
```

**Please note:** 
- Pin 21 can only be configured as input. It's direction can't be changed during runtime.
- At least one of the CS signal pins 15...17 (D0...D2) has to be configured as CS signal.
- Hardware interrupts can only be generated for rising edges of signals.
- Only one of the input pins can be configured to generate hardware interrupts (```hwirq``` set to 1).
- The signal at the input pin that is configured to generate hardware interrupts (```hwirq``` set to 1) **MUST** also be connected to the CH341A **INT** pin 7.
- If ther is no input should generate hardware interrupts, set ```hwirq``` to 0 for all entries.

### GPIO polling rate

GPIO inputs are polled periodically by a separate kernel thread. GPIO polling rate defines the **rate at which the kernel thread reads GPIO inputs** and determines whether to generate **software interrupts**. That is, it defines the maximum rate at which changes at GPIO inputs can be recognized and software interrupts can be generated. 

The GPIO polling rate is defined by its period in milliseconds using the constant ```CH341_POLL_PERIOD_MS```. The period must be at least 10 ms, but should be 20 ms or more if possible dependent on the performance of your system. Please check your ```syslog``` for messages like ```"GPIO poll period is too short by at least %n msecs"```. This message is thrown if the defined ```CH341_POLL_PERIOD_MS``` is shorter than the time required for one reading of the GPIOs. 

The higher GPIO polling rate is, the higher is the system usage by the kernel thread. On the other hand, the probability that short interrupt events will be lost grows, the lower the GPIO polling rate becomes.

GPIO polling rate can also be changed using the **module parameter** ```poll_rate``` either when loading the module, e.g.,

```
sudo modprobe spi_ch341_usb poll_rate=50
```
or as real ```root``` during runtime using sysfs, e.g.,
```
echo 50 > /sys/module/spi_ch341_usb/parameters/poll_period
```

**Please note:** Since the CH341A hardware interrupt signal **INT** uses a separate USB endpoint, the maximum rate of hardware interrupts is independent on the GPIO polling rate and can reach up to 400 Hz.


## Usage from user space

### Using SPI slaves

Once the driver is loaded successfully, it provides up to three SPI slave devices on next available SPI bus, e.g.,

```
/dev/spidev0.0
/dev/spidev0.1
/dev/spidev0.2
```

according to the naming scheme ```/dev/spidev<bus>.<cs>```. ```<bus>``` is the bus number selected automatically by the driver and ```<cs>``` is the chip select signal of the according pin. Please note that independent on how many pins are configured as chip select signals, pin 15 gives always 0, pin 16 gives always 1, and pin 17 gives always 2 as chip select signal.

Standard I/O functions like ```open```, ```ioctl``` and ```close``` can be used to communicate with one of the slaves connected to the SPI.

To open an SPI device simply use:
```
int spi = open("/dev/spidev0.0", O_RDWR));
```

Once the device is opened successfully, you can modify SPI configurations and transfer data using ```ioctl``` function.

```
uint8_t mode = SPI_MODE_0;
uint8_t lsb = SPI_LSB_FIRST;
...
ioctl(spi, SPI_IOC_WR_MODE, &mode);
ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb);
```
Function ```ioctl``` is also used to transfer data:

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

int status = ioctl (spi, SPI_IOC_MESSAGE(1), &spi_trans);

// use input data in miso
```

### Using GPIOs

To access GPIOs from user space, ```sysfs``` can be used . For each configured GPIO, a directory 
```
/sys/class/gpio/<gpio>/
```
is created by the system, where ```<gpio>``` is the name of the GPIO as defined in the driver variable ```ch341_board_config```. These directories contain

- the file ```value``` that can be used to read from and write to GPIOs,
- the file ```edge``` that can be used to control whether and what type of interrupt is enabled, and
- the file ```direction``` that can be used to change the direction of the GPIO if possible.

**Please note:** For read and write operations from and/or to these files, the user requires read and/or write permissions, respectively.

#### Open a GPIO

Before a GPIO can be used, file ```value``` has to be opened

```
int  fd;

if ((fd = open("/sys/class/gpio/<gpio>/value", O_RDWR)) == -1) 
{
    perror("open");
    return -1;
}
```
where ```<gpio>``` is again the name of the GPIO.

#### Write GPIO output

Once the file ```value``` is opened, you can use standard I/O functions to read and write. To write a GPIO value, simply use function ```write``` as following. The value is written to the GPIO out immediately.

```
if (write(fd, value ? "1" : "0", 1) == -1) 
{
    perror ("write");
	return -1;
}
```

#### Read GPIO input

To read values from GPIOs immediately, you can simply use function ```read``` as following:

```
char buf;

if (read(fd, &buf, 1) == -1) 
{
    perror("read");
    return -1;
}

value = (buf == '0') ? 0 : 1;
```
After each read operation, file position has to be rewound to first character before the next value can be read.
```
if (lseek(fd, 0, SEEK_SET) == -1) {
    perror("lseek");
    return -1;
}
```

#### Reacting on GPIO input interrupt

Function ```poll``` can be used before function ```read``` to react and read values from the GPIO only on interrupts.

```
struct pollfd fds[1];

fds[0].fd = fd;
fds[0].events = POLLPRI;

if (poll(fds, 1, -1) == -1) 
{
    perror("poll");
    return -1;
}
```

Function ```poll``` blocks until the specified event on the file descriptor happened.

**Please note**: The interrupt has to be activated before by ```root``` with command
```
echo <type> > /sys/class/gpio/<gpio>/edge
```
where ```<gpio>``` is again the name of the GPIO and ```<type>``` is the type of the interrupt that should be used. Possible interrupt types are 

- ```rising``` for interrupts on rising signal edges,
- ```falling``` for interrupts on falling signal edges, and
- ```both``` for interrupts on rising as well as falling signal edges.

For example, following command would activate interrupts for rising edges of the signal connected to ```gpio4```. The command has to be executed as real ```root```, using ```sudo``` command doesn't work.

```
echo rising > /sys/class/gpio/gpio4/edge
```

Even though the driver defines software interrupts for GPIO inputs as well as GPIO outputs, they can be activated only for GPIO inputs.

Full examples for GPIO output and interrupt input can be found in the driver's directory.

#### Change the GPIO direction

To change the direction of a GPIO pin configured as input or output, simply write as ```root``` keyword ```in``` or keyword ```out``` to the file ```direction```, e.g.

```
echo out > /sys/class/gpio/gpio4/direction
```
