/*
 * Driver for the CH341 USB to SPI and GPIO adapter
 *
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 *
 * Derived from
 *  
 *  i2c-ch341-usb.c Copyright (c) 2016 Tse Lun Bien
 *  i2c-ch341.c     Copyright (c) 2014 Marco Gittler
 *  i2c-tiny-usb.c  Copyright (c) 2006-2007 Till Harbaum (Till@Harbaum.org)
 *
 * and extended by GPIO and interrupt handling capabilities.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

// uncomment following line to activate kernel debug handling
#define DEBUG
   #define DEBUG_PRINTK

#ifdef DEBUG_PRINTK
#define PRINTK(fmt,...) printk("%s: "fmt"\n", __func__, ##__VA_ARGS__)
#else
#define PRINTK(fmt,...)
#endif 

#define CH341_IF_ADDR (&(ch341_dev->usb_if->dev))
#define DEV_ERR(d,f,...)  dev_err (d,"%s: "f"\n", __FUNCTION__, ##__VA_ARGS__)
#define DEV_DBG(d,f,...)  dev_dbg (d,"%s: "f"\n", __FUNCTION__, ##__VA_ARGS__)
#define DEV_INFO(d,f,...) dev_info(d,"%s: "f"\n", __FUNCTION__, ##__VA_ARGS__)

// check for condition and return with or without err code if it fails
#define CHECK_PARAM_RET(cond,err) if (!(cond)) return err;
#define CHECK_PARAM(cond)         if (!(cond)) return;

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#error The driver requires at least kernel version 3.10
#else

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/irq.h>

/** 
  * ATTENTION: 
  *
  * CH341_POLL_PERIOD_MS in milliseconds defines the rate at which GPIOs are
  * read from CH341 via USB and thus the maximum rate at which changes on 
  * interrupt driven input ports can be recognized at all.
  * 
  * This value must be at least 10 ms, but should be 20 ms or more (if 
  * possible) dependent on the performance of your system. Please check your
  * syslog for messages like "GPIO poll period is too short by at least %n
  * msecs". This message is thrown if the defined CH341_POLL_PERIOD_MS is
  * shorter than the time required for one reading of the GPIOs. 
  */
#define CH341_POLL_PERIOD_MS        100    // see above

#define CH341_USB_MAX_BULK_SIZE     32    // CH341A wMaxPacketSize for ep_02 and ep_82
#define CH341_USB_MAX_INTR_SIZE     8     // CH341A wMaxPacketSize for ep_81

#define CH341_CMD_PARA_INIT         0xB1
#define CH341_CMD_GET_STATUS        0xA0  // See ch341_spi_get_status
#define CH341_CMD_SET_OUTPUT        0xA1
#define CH341_CMD_SPI_STREAM        0xA8  // SPI command
#define CH341_CMD_UIO_STREAM        0xAB  // UIO command

#define CH341_CMD_UIO_STM_IN        0x00  // UIO interface IN  command (D0~D7)
#define CH341_CMD_UIO_STM_OUT       0x80  // UIO interface OUT command (D0~D5)
#define CH341_CMD_UIO_STM_DIR       0x40  // UIO interface DIR command (D0~D5)
#define CH341_CMD_UIO_STM_END       0x20  // UIO interface END command
#define CH341_CMD_UIO_STM_US        0xc0  // UIO interface US  command

#define CH341_SPI_MAX_NUM_DEVICES   3
#define CH341_SPI_BUS_NUM           0
#define CH341_SPI_MODALIAS          "spidev"
#define CH341_SPI_MODE              SPI_MODE_0
#define CH341_SPI_MIN_FREQ          400
#define CH341_SPI_MAX_FREQ          1e6
#define CH341_SPI_MIN_BITS_PER_WORD 4
#define CH341_SPI_MAX_BITS_PER_WORD 32

#define CH341_PIN_MODE_OUT          0
#define CH341_PIN_MODE_IN           1
#define CH341_PIN_MODE_CS           2

#define CH341_OK                    0

/**
 *  
 *  Change the default values in *ch341_board_config* for your configuraton
 *
 *  Configurable are:
 *
 *  - Pin 15 (D0/CS0  ) as input/output/CS (CH341_PIN_MODE_IN/CH341_PIN_MODE_OUT/CH341_PIN_MODE_CS)
 *  - Pin 16 (D1/CS1  ) as input/output/CS (CH341_PIN_MODE_IN/CH341_PIN_MODE_OUT/CH341_PIN_MODE_CS)
 *  - Pin 17 (D2/CS2  ) as input/output/CS (CH341_PIN_MODE_IN/CH341_PIN_MODE_OUT/CH341_PIN_MODE_CS)
 *  - Pin 19 (D4/DOUT2) as input/output    (CH341_PIN_MODE_IN/CH341_PIN_MODE_OUT)
 *  - Pin 21 (D6/DIN2 ) as input           (CH341_PIN_MODE_IN)
 *
 *  Pins 18 (sck), 20 (mosi), 22 (miso) have fix configuraton and are used as SPI signals.
 * 
 *  Note: To keep compatibility with the original version of this driver, I'm still tracking "pin numbers" in the 
 *  board_config.  But usually it is more meaningful to use the following bit numbers in the gpio_io_data work:
 * 
 *  bit 0-7 CH341  D7-D0,
 bit 8 对应 CH341 的 ERR#引脚, (pin 5)
 bit 9 对应 CH341 的 PEMP 引脚, (pin 6)
 bit 10 对应 CH341 的 INT#引脚, (pin 7) 
 bit 11 对应 CH341 的 SLCT 引脚, (pin 8)
 bit 13 对应 CH341 的 BUSY/WAIT#引脚, (pin 27)
 bit 14 对应 CH341 的 AUTOFD#/DATAS#引脚, (pin 4)
 bit 15 对应 CH341 的 SLCTIN#/ADDRS#引脚, (pin 3)
 bit 23 对应 CH341 的 SDA 引脚 (pin 23)
 */

struct ch341_pin_config {
    uint8_t bitNum;    // pin number of CH341 chip
    uint8_t mode;   // GPIO mode
    const char* name;   // GPIO name
    bool    hwirq;  // connected to hardware interrupt (only one pin can have true)
};

// Which bitnums be used for inputs or outputs, note: only used for non-spi D0-D7, other bits do not support change of direction at all
#define CH341_IN_OK_MASK        0x0000ef50 // gpio4, gpio6, none of the status bits, no MISO,MOSI, SCK
#define CH341_OUT_OK_MASK       0x000f0017 // cs0, cs1, cs2, gpio4 only

// Which bitnums can be used as a chip select
#define CH341_CS_OK_MASK        0x0007

// the bitmqask for MISO/MOSI/SCK
#define CH341_GPIO_SPI_MASK     0xa8

// bitmask for SPI SCK
#define SCK_H  0x08
#define CH341_MOSI_MASK 0x20


struct ch341_pin_config ch341_board_config[] =
{
    // bitnum  GPIO mode           GPIO name   hwirq
    {   0, CH341_PIN_MODE_CS , "cs0"     , 0 }, // used as a chip select by default
    {   1, CH341_PIN_MODE_CS , "cs1"     , 0 }, // used as a chip select by default
    {   2, CH341_PIN_MODE_CS , "cs2"     , 0 }, // used as a chip select by default
    // {   3, CH341_PIN_MODE_IN , "sck"     , 0 }, // expose to userspace to allow readonly access (for hardware debugging)

    {   4, CH341_PIN_MODE_IN , "gpio4"   , 0 }, // used as input with hardware IRQ
    // {   5, CH341_PIN_MODE_IN , "mosi"    , 0 }, // expose to userspace to allow readonly access (for hardware debugging)
    {   6, CH341_PIN_MODE_IN , "gpio6"   , 0 }, // used as input (only)
    // {   7, CH341_PIN_MODE_IN , "miso"    , 0 }, // expose to userspace to allow readonly access (for hardware debugging)

    {   8, CH341_PIN_MODE_IN , "err"     , 0 },
    {   9, CH341_PIN_MODE_IN , "pemp"    , 0 },
    {   10, CH341_PIN_MODE_IN , "int"    , 1 },
    {   11, CH341_PIN_MODE_IN , "slct"   , 0 },
    
    {   13, CH341_PIN_MODE_IN , "wait"   , 0 },
    {   14, CH341_PIN_MODE_IN , "autofd" , 0 },
    {   15, CH341_PIN_MODE_IN , "addr"   , 0 },

    {   16, CH341_PIN_MODE_OUT, "ini"  , 0 },
    {   17, CH341_PIN_MODE_OUT, "write"  , 0 },
    {   18, CH341_PIN_MODE_OUT, "scl"    , 0 },
    {   19, CH341_PIN_MODE_OUT, "sda"    , 0 } // Example code says this is GPIO 29 but I think that is a typo (leaving out for now)
};

#define CH341_GPIO_NUM_PINS (sizeof(ch341_board_config) / sizeof(ch341_board_config[0]))

static struct spi_board_info ch341_spi_devices[CH341_SPI_MAX_NUM_DEVICES];

struct spi_board_info ch341_spi_device_template = 
{
        .modalias     = "spidev",
        .max_speed_hz = CH341_SPI_MAX_FREQ,
        .bus_num      = 0,
        .chip_select  = 0,
        .mode         = SPI_MODE_0,
};

// device specific structure
struct ch341_device 
{
    // USB device description
    struct usb_device*    usb_dev;  // usb device
    struct usb_interface* usb_if;   // usb interface

    struct usb_endpoint_descriptor *ep_in;      // usb endpoint bulk in
    struct usb_endpoint_descriptor *ep_out;     // usb endpoint bulk out
    struct usb_endpoint_descriptor *ep_intr;    // usb endpoint interrupt in

    uint8_t in_buf  [CH341_USB_MAX_BULK_SIZE]; // usb input buffer
    uint8_t out_buf [CH341_USB_MAX_BULK_SIZE]; // usb outpu buffer
    uint8_t intr_buf[CH341_USB_MAX_INTR_SIZE]; // usb interrupt buffer
    
    struct urb* intr_urb;

    // SPI device description
    struct spi_master*  master;   // spi master
    struct spi_device*  slaves[CH341_SPI_MAX_NUM_DEVICES];
    int                 slave_num;

    // GPIO device description
    struct gpio_chip         gpio;                              // chip descriptor for GPIOs
    uint8_t                  gpio_num;                          // number of pins used as GPIOs    
    uint32_t                 gpio_mask;                         // configuratoin mask defines IN/OUT pins
    uint32_t                 gpio_io_data;                      // current value of CH341 I/O register
    struct task_struct *     gpio_thread;                       // GPIO poll thread
    struct completion        gpio_thread_complete;              // Used to wait for thread exit
    struct ch341_pin_config* gpio_pins   [CH341_GPIO_NUM_PINS]; // pin configurations (gpio_num elements)
    uint32_t                 gpio_bits   [CH341_GPIO_NUM_PINS]; // bitmask in status word (gpio_num elements)
    const char*              gpio_names  [CH341_GPIO_NUM_PINS]; // pin names  (gpio_num elements)
    int                      gpio_irq_map[CH341_GPIO_NUM_PINS]; // GPIO to IRQ map (gpio_num elements)
    uint32_t                 gpio_out_ok_mask;                  // set bits indicate pins which can be outputs
    uint32_t                 gpio_in_ok_mask;                   // set bits indicate pins which can be inputs

    // IRQ device description
    struct irq_chip   irq;                                // chip descriptor for IRQs
    uint8_t           irq_num;                            // number of pins with IRQs
    int               irq_base;                           // base IRQ allocated
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
    struct irq_desc * irq_descs    [CH341_GPIO_NUM_PINS]; // IRQ descriptors used (irq_num elements)
#endif
    int               irq_types    [CH341_GPIO_NUM_PINS]; // IRQ types (irq_num elements)
    bool              irq_enabled  [CH341_GPIO_NUM_PINS]; // IRQ enabled flag (irq_num elements)
    int               irq_gpio_map [CH341_GPIO_NUM_PINS]; // IRQ to GPIO pin map (irq_num elements)
    int               irq_hw;                             // IRQ for GPIO with hardware IRQ (default -1)
};

// ----- variables configurable during runtime ---------------------------

static uint poll_period = CH341_POLL_PERIOD_MS;       // module parameter poll period

// ----- function prototypes ---------------------------------------------

static int ch341_usb_transfer (struct ch341_device *dev, int out_len, int in_len);

// ----- board configuration layer begin ---------------------------------

static int ch341_cfg_probe (struct ch341_device* ch341_dev)
{
    struct ch341_pin_config* cfg;
    int i;

    CHECK_PARAM_RET (ch341_dev, -EINVAL);

    ch341_dev->gpio_mask   = 0x3f;
    ch341_dev->gpio_out_ok_mask = CH341_OUT_OK_MASK;
    ch341_dev->gpio_in_ok_mask = CH341_IN_OK_MASK;    
    ch341_dev->gpio_num    = 0;
    ch341_dev->gpio_thread = 0;

    ch341_dev->irq_num     = 0;
    ch341_dev->irq_base    = 0;
    ch341_dev->irq_hw      = -1;

    for (i = 0; i < CH341_GPIO_NUM_PINS; i++)
    {
        uint32_t msk;

        cfg = ch341_board_config + i;
            
        // --- check correct pin configuration ------------
        msk = 1 << cfg->bitNum;
            
        // is pin configured correctly as input
        if (!(msk & CH341_OUT_OK_MASK) && cfg->mode != CH341_PIN_MODE_IN)
        {
            DEV_ERR(CH341_IF_ADDR, "bit %d: must be an input", cfg->bitNum);
            return -EINVAL;
        }

        // is pin configurable as CS signal
        else if (!(msk & CH341_CS_OK_MASK) && cfg->mode == CH341_PIN_MODE_CS)
        {
            DEV_ERR(CH341_IF_ADDR, "bit %d: can't be used as CS signal", cfg->bitNum);
            return -EINVAL;
        }

        // --- read in pin configuration
        ch341_dev->gpio_bits[ch341_dev->gpio_num] = msk;

        if (cfg->mode == CH341_PIN_MODE_CS)
        {
            // if pin is CS signal, set SPI slave device configuration
            ch341_spi_devices[ch341_dev->slave_num] = ch341_spi_device_template;
            ch341_spi_devices[ch341_dev->slave_num].bus_num      = CH341_SPI_BUS_NUM;
            ch341_spi_devices[ch341_dev->slave_num].mode         = CH341_SPI_MODE;
            ch341_spi_devices[ch341_dev->slave_num].chip_select  = cfg->bitNum;

            // cs pins can't be changed to be inputs
            ch341_dev->gpio_in_ok_mask &= ~msk;

            DEV_INFO (CH341_IF_ADDR, "output %s SPI slave with cs=%d", 
                      cfg->name, ch341_spi_devices[ch341_dev->slave_num].chip_select);

            ch341_dev->slave_num++;    
        }
      
        // now expose as a GPIO (note: even if a pin is a chip select, we also expose it as a GPIO, so apps can do low level control if needed)
        ch341_dev->gpio_names  [ch341_dev->gpio_num] = cfg->name;
        ch341_dev->gpio_pins   [ch341_dev->gpio_num] = cfg;
        ch341_dev->gpio_irq_map[ch341_dev->gpio_num] = -1; // no valid IRQ
        
        // GPIO pins can generate IRQs when set to input mode
        ch341_dev->gpio_irq_map[ch341_dev->gpio_num] = ch341_dev->irq_num;
        ch341_dev->irq_gpio_map[ch341_dev->irq_num]  = ch341_dev->gpio_num;
            
        if (cfg->hwirq)
        {
            if (ch341_dev->irq_hw != -1)
            {
                DEV_ERR(CH341_IF_ADDR, 
                "bit %d: only one GPIO can be connected to the hardware IRQ", 
                cfg->bitNum);
                return -EINVAL;
            }
            
            ch341_dev->irq_hw = ch341_dev->irq_num;
        }
            
        if (cfg->mode == CH341_PIN_MODE_IN && (msk & CH341_IN_OK_MASK))
            // if pin is INPUT, it has to be masked out in GPIO direction mask
            ch341_dev->gpio_mask &= ~msk;
        else
            ch341_dev->gpio_mask |= msk;

        DEV_INFO (CH341_IF_ADDR, "%s %s gpio=%d irq=%d %s", 
                    cfg->mode == CH341_PIN_MODE_IN ? "input " : "output",
                    cfg->name, ch341_dev->gpio_num, ch341_dev->irq_num,
                    cfg->hwirq ? "(hwirq)" : "");

        ch341_dev->irq_num++;
        ch341_dev->gpio_num++;
    }
    
    if (ch341_dev->slave_num == 0)
    {
        DEV_ERR(CH341_IF_ADDR, "at least one of the pins 15 ... 17 has to be configured as CS signal");
        return -EINVAL;
    }

    return CH341_OK;
}

static void ch341_cfg_remove (struct ch341_device* ch341_dev)
{
    CHECK_PARAM (ch341_dev);

    return;
}

// ----- board configuration layer end -----------------------------------

// ----- spi layer begin -------------------------------------------------

static struct mutex ch341_lock;

#define ch341_spi_maser_to_dev(m) *((struct ch341_device**)spi_master_get_devdata(m))

/*
Get Status Reverse Engineering
A note for future developers @icenowy reverse engineered the windows DLL getstatus operation. @geeksville used this great information
to add an implementation for this linux driver.
Her email is here:
I reverse-engineered the CH341GetInput function in the DLL.

bool CH341GetInput(ULONG iIndex,PULONG iStatus)
{
  bool ret;
  bool wr_ret;
  byte buf [3];
  ULONG dev_index;

  // 0x2cac  19  CH341GetInput
  dev_index = iIndex;
  if (CH341ICVersions[iIndex] < 0x20) {
                    
    ret = CH341GetStatus(iIndex,iStatus); // IC Ver is original CH341 
  }
  else {
    iIndex = 0;
    buf[0] = 0xa0;
    wr_ret = CH341WriteRead(dev_index,1,buf,0x20,1,&iIndex,buf);
    if (wr_ret == 0) {
      ret = false;
    }
    else {
      *iStatus = ((buf[2] & 0x80) << 8 | buf[1] & 0xef) << 8 |
(uint)buf[0];
      ret = true;
    }
  }
  return ret;
}

CH341WriteRead seems to be doing bulk transfer, and new CH341F chips
should not be IC Ver 0x10 (IC ver has 3 values, 0x10, 0x20 and 0x30).

For this function, the result format is

 位 7-位 0 对应 CH341 的 D7-D0 引脚,
 位 8 对应 CH341 的 ERR#引脚,
 位 9 对应 CH341 的 PEMP 引脚,
 位 10 对应 CH341 的 INT#引脚,
 位 11 对应 CH341 的 SLCT 引脚,
 位 13 对应 CH341 的 BUSY/WAIT#引脚,
 位 14 对应 CH341 的 AUTOFD#/DATAS#引脚,
 位 15 对应 CH341 的 SLCTIN#/ADDRS#引脚,
 位 23 对应 CH341 的 SDA 引脚
*/
static int ch341_spi_get_status (struct ch341_device* ch341_dev)
{
    int result;
    uint32_t status;

    mutex_lock (&ch341_lock);

    ch341_dev->out_buf[0] = CH341_CMD_GET_STATUS;

    result = ch341_usb_transfer(ch341_dev, 1, 3);

    status = ((ch341_dev->in_buf[2] & 0x80) << 16) | ((ch341_dev->in_buf[1] & 0xef) << 8) | ch341_dev->in_buf[0];
    status &= ~CH341_GPIO_SPI_MASK; // Don't readback bogus SPI pin values
    ch341_dev->gpio_io_data = (ch341_dev->gpio_io_data & ch341_dev->gpio_mask) | (~ch341_dev->gpio_mask & status); // these bits include current GPIO values (and more), only change bits not marked as outputs

    mutex_unlock (&ch341_lock);

    return (result < 0) ? result : CH341_OK;
}

/* No longer used, we now use the get_status operation found by @icesnoy - because it returns all pins (not just the gpios)

static int ch341_spi_read_inputs (struct ch341_device* ch341_dev)
{
    int result;

    mutex_lock (&ch341_lock);

    ch341_dev->out_buf[0] = CH341_CMD_UIO_STREAM;
    ch341_dev->out_buf[1] = CH341_CMD_UIO_STM_DIR | ch341_dev->gpio_mask;
    ch341_dev->out_buf[2] = CH341_CMD_UIO_STM_IN;
    ch341_dev->out_buf[3] = CH341_CMD_UIO_STM_END;

    result = ch341_usb_transfer(ch341_dev, 4, 1);

    ch341_dev->gpio_io_data &= ch341_dev->gpio_mask;
    ch341_dev->gpio_io_data |= ch341_dev->in_buf[0] & ~ch341_dev->gpio_mask;

    mutex_unlock (&ch341_lock);

    return (result < 0) ? result : CH341_OK;
}
*/

/**
 * A new more generalized version of write_outputs that can write ALL of the possible output pins
 * 
 * Based on this example code:
 * 
 * ********************************************************************
 * FUNCTION : Set direction and output data of CH341 
 * arg:
 * Data :	Control direction and data 
 
 * iEnbale : set direction and data enable
 * 			   --> Bit16 High :	effect on Bit15~8 of iSetDataOut
 * 			   --> Bit17 High :	effect on Bit15~8 of iSetDirOut
 * 			   --> Bit18 High :	effect on Bit7~0 of iSetDataOut
 * 			   --> Bit19 High :	effect on Bit7~0 of iSetDirOut
 *			   --> Bit20 High :	effect on Bit23~16 of iSetDataOut
 * iSetDirOut : set io direction
 *			  -- > Bit High : Output 
 *			  -- > Bit Low : Input
 * iSetDataOut : set io data
 * 			 Output:
 *			  -- > Bit High : High level
 *			  -- > Bit Low : Low level
 * Note:
 * Bit7~Bit0<==>D7-D0 
 * Bit8<==>ERR#    Bit9<==>PEMP    Bit10<==>INT#    Bit11<==>SLCT    Bit13<==>WAIT#    Bit14<==>DATAS#/READ#    Bit15<==>ADDRS#/ADDR/ALE
 * The pins below can only be used in output mode:
 * Bit16<==>RESET#    Bit17<==>WRITE#    Bit18<==>SCL    Bit29<==>SDA
 * ********************************************************************

BOOL CH34xSetOutput( ULONG	iEnable, ULONG iSetDirOut, ULONG iSetDataOut)
{
	ULONG mLength;
	UCHAR mBuffer[32];
	mBuffer[0] = CH341A_CMD_SET_OUTPUT;
	mBuffer[1] = 0x6A;
	mBuffer[2] = (UCHAR)( iEnable & 0x1F );
	mBuffer[3] = (UCHAR)( iSetDataOut >> 8 & 0xEF );
	mBuffer[4] = (UCHAR)( iSetDirOut >> 8 & 0xEF | 0x10 );
	mBuffer[5] = (UCHAR)( iSetDataOut & 0xFF );
	mBuffer[6] = (UCHAR)( iSetDirOut & 0xFF );
	mBuffer[7] = (UCHAR)( iSetDataOut >> 16 & 0x0F );
	mBuffer[8] = 0;
	mBuffer[9] = 0;
	mBuffer[10] = 0;
	mLength = 11;
	if ( CH34xWriteData(  mBuffer, &mLength ) ) {  //Write Data 
		if ( mLength >= 8 ) return( true);
	}
	return( false);
}
*/
static int ch341_spi_write_outputs (struct ch341_device* ch341_dev)
{
    int result;
    uint32_t data;

    mutex_lock (&ch341_lock);

    data = ch341_dev->gpio_io_data & ch341_dev->gpio_mask;

    // FIXME - set idle SCK based on clock phase
    // (spi->mode & SPI_CPOL)

    ch341_dev->out_buf[0] = CH341_CMD_SET_OUTPUT;
    ch341_dev->out_buf[1] = 0x6a; // (data & 0xff & ~CH341_GPIO_SPI_MASK); 
    ch341_dev->out_buf[2] = 0x1f; // (ch341_dev->gpio_mask & 0xff & ~CH341_GPIO_SPI_MASK) | SCK_H; // 0x1f; // FIXME?
    ch341_dev->out_buf[3] = (data >> 8) & 0xef;
    ch341_dev->out_buf[4] = ((ch341_dev->gpio_mask >> 8) & 0xef) | 0x10;
    ch341_dev->out_buf[5] = (data & 0xff & ~CH341_GPIO_SPI_MASK) | CH341_MOSI_MASK; // FIXME, set SCK state based on CPOL
    ch341_dev->out_buf[6] = (ch341_dev->gpio_mask & 0xff & ~CH341_GPIO_SPI_MASK) | SCK_H | CH341_MOSI_MASK; // Never accidentally drive the SPI signals as GPIOs
    ch341_dev->out_buf[7] = (data >> 16) & 0x0f;
    ch341_dev->out_buf[8] = 0;
    ch341_dev->out_buf[9] = 0;
    ch341_dev->out_buf[10] = 0;

    // DEV_DBG(CH341_IF_ADDR, "mask=0x%08x data=0x%08x", ch341_dev->gpio_mask, data);
    
    result = ch341_usb_transfer(ch341_dev, 11, 0);

    mutex_unlock (&ch341_lock);

    return (result < 0) ? result : CH341_OK;
}


/* THis old implementation of write_outputs wrote just D0-5

static int ch341_spi_write_outputs (struct ch341_device* ch341_dev)
{
    int result;

    mutex_lock (&ch341_lock);

    ch341_dev->out_buf[0] = CH341_CMD_UIO_STREAM;
    ch341_dev->out_buf[1] = CH341_CMD_UIO_STM_DIR | (ch341_dev->gpio_mask & CH341_GPIO_OUT_MASK);
    ch341_dev->out_buf[2] = CH341_CMD_UIO_STM_OUT | (ch341_dev->gpio_io_data & ch341_dev->gpio_mask & CH341_GPIO_OUT_MASK);
    ch341_dev->out_buf[3] = CH341_CMD_UIO_STM_END;

    DEV_DBG(CH341_IF_ADDR, "mask=0x%08x dir=0x%02x val=0x%02x", ch341_dev->gpio_mask, ch341_dev->out_buf[1], ch341_dev->out_buf[2]);
    
    result = ch341_usb_transfer(ch341_dev, 4, 0);

    mutex_unlock (&ch341_lock);

    return (result < 0) ? result : CH341_OK;
}
*/

/** NOT USED YET (possibly never)
 * Based on example code
 * //Init Parallel Mode
//iMode-> 00/01 EPP
//iMode-> 02	MEM
static int CH34xInitParallel( unsigned char iMode, struct ch34x_pis *dev )
{
	int retval;
	__u8 RequestType = VENDOR_WRITE_TYPE;
	__u8 Request = CH34x_PARA_INIT;
	__u16 Value = ( iMode << 8 )|( iMode < 0x00000100 ? 0x02 : 0x00 );
	__u16 Index = 0;
	__u16 len = 0;
	retval = usb_control_msg( dev->udev, 
			usb_sndctrlpipe(dev->udev, 0), Request,
			RequestType, Value, Index, NULL, len, 1000);

	return retval;
} */
static int ch341_init_parallel (struct ch341_device* ch341_dev)
{
    int result;

    mutex_lock (&ch341_lock);

    ch341_dev->out_buf[0] = CH341_CMD_PARA_INIT;
    ch341_dev->out_buf[1] = 0x02;
    ch341_dev->out_buf[2] = 0;

    DEV_DBG(CH341_IF_ADDR, "init");
    
    result = ch341_usb_transfer(ch341_dev, 3, 0);

    mutex_unlock (&ch341_lock);

    return (result < 0) ? result : CH341_OK;
}

static uint8_t ch341_spi_swap_byte(const uint8_t byte)
{
    uint8_t orig = byte;
    uint8_t swap = 0;
    int i;
    
    for (i = 0; i < 8; ++i)
    {
        swap = swap << 1;
        swap |= (orig & 1);
        orig = orig >> 1;
    }
    return swap;
}

static const int cs_bits[CH341_SPI_MAX_NUM_DEVICES] = { 0x01, 0x02, 0x04 };

static int ch341_spi_set_cs (struct spi_device *spi, bool active)
{
    struct ch341_device* ch341_dev;
    int result;
    uint32_t old_gpio;
    
    CHECK_PARAM_RET (spi, -EINVAL);
    CHECK_PARAM_RET (ch341_dev = ch341_spi_maser_to_dev(spi->master), -EINVAL);

    // DEV_DBG (CH341_IF_ADDR, "active %s", active ? "true" : "false");

    if (spi->chip_select > CH341_SPI_MAX_NUM_DEVICES)
    {
        DEV_ERR (CH341_IF_ADDR, "invalid CS value %d, 0~%d are available", 
                 spi->chip_select, CH341_SPI_MAX_NUM_DEVICES-1);
        return -EINVAL;
    }
    
    old_gpio = ch341_dev->gpio_io_data;

    if (active)
        ch341_dev->gpio_io_data &= ~cs_bits[spi->chip_select];
    else
        ch341_dev->gpio_io_data |= cs_bits[spi->chip_select];

    if(ch341_dev->gpio_io_data != old_gpio) 
    { 
        // if no pin change, don't bother sending a USB transation
        ch341_dev->out_buf[0]  = CH341_CMD_UIO_STREAM;
        ch341_dev->out_buf[1]  = CH341_CMD_UIO_STM_DIR | (ch341_dev->gpio_mask & 0xff) | SCK_H; // FIXME, set SCK state based on CPOL
        ch341_dev->out_buf[2]  = CH341_CMD_UIO_STM_OUT | (ch341_dev->gpio_io_data & ch341_dev->gpio_mask & 0xff);
        ch341_dev->out_buf[3]  = CH341_CMD_UIO_STM_END;

        // DEV_DBG(CH341_IF_ADDR, "mask=0x%x dir=0x%02x val=0x%02x", ch341_dev->gpio_mask, ch341_dev->out_buf[1], ch341_dev->out_buf[2]);
            
        result = ch341_usb_transfer(ch341_dev, 4, 0);

        return (result < 0) ? result : CH341_OK;
    }
    else 
    {
        return CH341_OK;
    }
}

// Implementation of bit banging protocol uses following IOs to be compatible
// with the hardware SPI interface
//
//      D7     D6     D5     D4     D3     D2     D1     D0
//      MISO   IN2    MOSI   OUT2   SCK    CS2    CS1    CS0

static int ch341_spi_bitbang (struct ch341_device* ch341_dev, 
                              struct spi_device *spi, 
                              const uint8_t* tx, uint8_t* rx, int len)
{
    uint8_t  byte, bit; 
    uint8_t* io = ch341_dev->out_buf;
    int result = 0;
    int k = 0;
    int i, b;

    // CPOL=0, CPHA=0   data must be stable while clock is high, can be changed while clock is low
    // mode 0           data sampled on raising clock edge
    //
    // CPOL=0, CPHA=1   data must be stable while clock is low, can be changed while clock is high
    // mode=1           data sampled on falling clock edge
    //
    // CPOL=1, CPHA=0   data must be stable while clock is low, can be changed while clock is high
    // mode=2           data sampled on falling clock edge
    //
    // CPOL=1, CPHA=1   data must be stable while clock is high, can be changed while clock is low
    // mode=3           data sampled on raising clock edge

    uint8_t SCK_L  = 0;
    uint8_t CPOL   = (spi->mode & SPI_CPOL) ? 0x08 : 0;
    uint8_t CS_H   = cs_bits[spi->chip_select];
    uint8_t CS_L   = 0;
    uint8_t MOSI_H = 0x20;
    uint8_t MASK   = ch341_dev->gpio_mask;
    uint8_t DATA   = ch341_dev->gpio_io_data & ch341_dev->gpio_mask;

    uint8_t mode = spi->mode & SPI_MODE_3;
    bool    lsb  = spi->mode & SPI_LSB_FIRST;

    // DEV_DBG (CH341_IF_ADDR, "start");

    // mask SPI GPIO data
    DATA &= ~MOSI_H & ~SCK_H & ~CS_H;
    
    k = 0;
    io[k++] = CH341_CMD_UIO_STREAM;
    io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_H | CPOL; // set defaults CS#=HIGH, SCK=CPOL
    io[k++] = CH341_CMD_UIO_STM_DIR | MASK;               // input: MISO, IN2; output MOSI, OUT2, SCK, CS#;
    io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_L | CPOL; // start with CS0=LOW, SCK=CPOL
    io[k++] = CH341_CMD_UIO_STM_END;
    if ((result = ch341_usb_transfer(ch341_dev, k, 0)) < 0)
        return result;
    
    for (b = 0; b < len; b++)
    {
        k = 0;
        io[k++] = CH341_CMD_UIO_STREAM;
        
        byte = lsb ? ch341_spi_swap_byte(tx[b]) : tx[b];
        for (i = 0; i < 8; i++)
        {
            bit = byte & 0x80 ? 0x20 : 0;  // lsb
            byte = byte << 1;
            
            if (mode == SPI_MODE_0 || mode == SPI_MODE_3)
            {
                io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_L | SCK_L | bit; // keep CS0=LOW, set SCK=LOW , set MOSI
                io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_L | SCK_H | bit; // keep CS0=LOW, set SCK=HIGH, keep MOSI
                io[k++] = CH341_CMD_UIO_STM_IN; // read MISO
            }
            else
            {
                io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_L | SCK_L | bit; // keep CS0=LOW, set SCK=HIGH, set MOSI
                io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_L | SCK_H | bit; // keep CS0=LOW, set SCK=LOW , keep MOSI
                io[k++] = CH341_CMD_UIO_STM_IN; // read MISO
            }
        }
        io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_L | CPOL; // keep CS0=LOW, SCK=CPOL, MOSI=LOW
        io[k++] = CH341_CMD_UIO_STM_END;
        if ((result = ch341_usb_transfer(ch341_dev, k, 8)) < 0)
            return result;

        byte = 0;
        for (i = 0; i < 8; i++)
        {
            byte = byte << 1;
            byte = byte | ((ch341_dev->in_buf[i] & 0x80) ? 1 : 0);
        }
        rx[b] =  lsb ? ch341_spi_swap_byte(byte) : byte;
    }
    
    k = 0;
    io[k++] = CH341_CMD_UIO_STREAM;
    io[k++] = CH341_CMD_UIO_STM_OUT | DATA | CS_H | CPOL; // default status: CS#=HIGH, SCK=CPOL
    io[k++] = CH341_CMD_UIO_STM_END;
    if ((result = ch341_usb_transfer(ch341_dev, k, 0)) < 0)
        return result;

    // save last I/O data byte
    DATA  = ch341_dev->gpio_io_data;
    DATA &= ~MOSI_H & ~SCK_H & ~CS_H;
    DATA |= CS_H | CPOL;
    
    // DEV_DBG (CH341_IF_ADDR, "done");

    return 0;
}

static void ch341_set_cs(struct spi_device *spi, bool enable) {
    // struct ch341_device* ch341_dev = ch341_spi_maser_to_dev(spi->master);
    // DEV_DBG (CH341_IF_ADDR, "cs=%d", enable);    
    ch341_spi_set_cs (spi, enable);
}

static int ch341_spi_transfer_low(struct spi_master *master,
                                  struct spi_device *spi, 
                                  struct spi_transfer* t)
{
    struct ch341_device* ch341_dev = ch341_spi_maser_to_dev(spi->master);
    const uint8_t* tx;
    uint8_t* rx;
    bool lsb; 
    int result = 0;
    int i;

    CHECK_PARAM_RET (ch341_dev, EIO);
    CHECK_PARAM_RET (master   , EIO);
    CHECK_PARAM_RET (spi      , EIO);
    CHECK_PARAM_RET (t        , EIO); 
    CHECK_PARAM_RET (t->len <= CH341_USB_MAX_BULK_SIZE, EIO);

    // DEV_DBG (CH341_IF_ADDR, "");

    // use slow bitbang implementation for SPI_MODE_1, SPI_MODE_2 and SPI_MODE_3
    if (spi->mode & SPI_MODE_3) {
        result = ch341_spi_bitbang (ch341_dev, spi, t->tx_buf, t->rx_buf, t->len);
    }
    else
    {
        // otherwise the faster hardware implementation 
        lsb = spi->mode & SPI_LSB_FIRST;
        tx  = t->tx_buf;
        rx  = t->rx_buf;
    
        // activate cs (always)
        ch341_spi_set_cs (spi, true);

        // fill output buffer with command and output data, controller expects lsb first
        ch341_dev->out_buf[0] = CH341_CMD_SPI_STREAM;
        for (i = 0; i < t->len; i++) {
            uint8_t b = lsb ? tx[i] : ch341_spi_swap_byte(tx[i]);
            ch341_dev->out_buf[i+1] = b;
            // DEV_DBG (CH341_IF_ADDR, "outb=0x%02x", tx[i]);
        }

        // transfer output and input data
        if(t->len)
            result = ch341_usb_transfer(ch341_dev, t->len + 1, t->len);

        // deactivate cs
        if (t->cs_change) // we must be running on an older kernel, newer kernels would have called set_cs instead
            ch341_spi_set_cs (spi, false);

        // fill input data with input buffer, controller delivers lsb first
        if (result >= 0 && rx)
            for (i = 0; i < t->len; i++) {
                rx[i] = lsb ? ch341_dev->in_buf[i] : ch341_spi_swap_byte(ch341_dev->in_buf[i]);
                // DEV_DBG (CH341_IF_ADDR, "inb=0x%02x", rx[i]);
            }

    }

    if(t->len == 1) // show bytes transfered if in the common single byte case
        DEV_DBG (CH341_IF_ADDR, "len=%u, csChange=%d, result=%d, txb=0x%02x, rxb=0x%02x", t->len, t->cs_change, result, tx[0], rx[0]);
    else
        DEV_DBG (CH341_IF_ADDR, "len=%u, csChange=%d, result=%d", t->len, t->cs_change, result);

    return result;
}

static int ch341_spi_transfer_one(struct spi_master *master,
                                  struct spi_device *spi, 
                                  struct spi_transfer* t)
{
    int result;

    // DEV_DBG (CH341_IF_ADDR, "");

    mutex_lock (&ch341_lock);

    result = ch341_spi_transfer_low(master, spi, t);

    spi_finalize_current_transfer(master);

    mutex_unlock (&ch341_lock);

    return result;
}

/*
 * spi_transfer_one_message - Workaround for cs deassertion (IMO bug) in recent kernels (aprox >=5.10)
 * replace the kernel version with our impl
 */
static int spi_transfer_one_message(struct spi_master *master,
				    struct spi_message *msg)
{
    struct spi_device *spi = msg->spi;
    // struct ch341_device* ch341_dev = ch341_spi_maser_to_dev(master);

	struct spi_transfer *xfer;
	int ret = 0;

    mutex_lock (&ch341_lock);
	ch341_set_cs(msg->spi, true);

    msg->actual_length = 0;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {

        ret = ch341_spi_transfer_low(master, spi, xfer);

		msg->actual_length += xfer->len;
	}

	msg->status = ret;

	spi_finalize_current_message(master);
    mutex_unlock (&ch341_lock);

    if(ret > 0) // transfer_one_message does not want a count as a result, just 0 for success
        ret = 0;

    // DEV_DBG (CH341_IF_ADDR, "ret=%d", ret);
	return ret;
}


static int ch341_spi_probe (struct ch341_device* ch341_dev)
{
    int bus = 0;
    int result;
    int i;

    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    
    DEV_DBG (CH341_IF_ADDR, "start");

    // search for next free bus number
    while ((ch341_dev->master = spi_busnum_to_master(bus)))
    { 
        // returns a refcounted pointer to an existing master
        spi_master_put (ch341_dev->master);
        bus++;
    }

    // allocate a new SPI master with a pointer to ch341_device as device data
    ch341_dev->master = spi_alloc_master(CH341_IF_ADDR, sizeof(struct ch341_device*));
    if (!ch341_dev->master)
    {
        DEV_ERR (CH341_IF_ADDR, "SPI master allocation failed");
        return -ENOMEM;
    }
    
    // save the pointer to ch341_dev in the SPI master device data field
    ch341_spi_maser_to_dev (ch341_dev->master) = ch341_dev;

    DEV_INFO (CH341_IF_ADDR, "SPI master connected to SPI bus %d", bus);

    // set SPI master configuration
    ch341_dev->master->bus_num = bus;
    ch341_dev->master->num_chipselect = CH341_SPI_MAX_NUM_DEVICES;
    ch341_dev->master->mode_bits = SPI_MODE_3 | SPI_LSB_FIRST;
    ch341_dev->master->flags = SPI_MASTER_MUST_RX | SPI_MASTER_MUST_TX;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,0,0)
    ch341_dev->master->bits_per_word_mask = SPI_BIT_MASK(8);
#else
    ch341_dev->master->bits_per_word_mask = SPI_BPW_MASK(8);
#endif
    // ch341_dev->master->transfer_one = ch341_spi_transfer_one;
    ch341_dev->master->transfer_one_message = spi_transfer_one_message;
    ch341_dev->master->set_cs = ch341_set_cs;
    ch341_dev->master->max_speed_hz = CH341_SPI_MAX_FREQ;
    ch341_dev->master->min_speed_hz = CH341_SPI_MIN_FREQ;

    // register the new master
    if ((result = spi_register_master (ch341_dev->master)))
    {
        DEV_ERR(CH341_IF_ADDR, "could not register SPI master");
        spi_master_put(ch341_dev->master);
        // in case of error, reset the master to avoid crash during free
        ch341_dev->master = 0;
        return result;
    }

    // create SPI slaves
    for (i = 0; i < ch341_dev->slave_num; i++)
    {
        ch341_spi_devices[i].bus_num = bus;
        if ((ch341_dev->slaves[i] = spi_new_device(ch341_dev->master, &ch341_spi_devices[i])))
        {
            DEV_INFO (CH341_IF_ADDR, "SPI device /dev/spidev%d.%d created", 
                      bus, ch341_spi_devices[i].chip_select);
            ch341_spi_set_cs (ch341_dev->slaves[i], false);
        }
    }

    mutex_init (&ch341_lock);

    DEV_DBG (CH341_IF_ADDR, "done");

    return CH341_OK;
}

static void ch341_spi_remove (struct ch341_device* ch341_dev)
{
    CHECK_PARAM (ch341_dev);
    
    // Not needed because spi_unregister_master (spi_unregister_controller) will do this automatically
    /* for (i = 0; i < ch341_dev->slave_num; i++)
        if (ch341_dev->slaves[i])
            spi_unregister_device (ch341_dev->slaves[i]);
    */

    if (ch341_dev->master)
    {
        spi_unregister_master (ch341_dev->master);

        // based on inspection of existing spi drivers in the kernel, this seems unnecessary (and possibly harmful)
        // spi_master_put (ch341_dev->master);
    }

    return;
}

// ----- spi layer end ---------------------------------------------------

// ----- irq layer begin -------------------------------------------------

void ch341_irq_enable_disable (struct irq_data *data, bool enable)
{    
    struct ch341_device *ch341_dev;
    int irq;
    
    CHECK_PARAM (data && (ch341_dev = irq_data_get_irq_chip_data(data)));

    // calculate local IRQ
    irq = data->irq - ch341_dev->irq_base;

    // valid IRQ is in range 0 ... ch341_dev->irq_num-1, invalid IRQ is -1
    if (irq < 0 || irq >= ch341_dev->irq_num) return;
    
    // enable local IRQ
    ch341_dev->irq_enabled[irq] = enable;

    DEV_INFO (CH341_IF_ADDR, "irq=%d enabled=%d", 
              data->irq, ch341_dev->irq_enabled[irq] ? 1 : 0);
}

void ch341_irq_enable (struct irq_data *data)
{
    ch341_irq_enable_disable (data, true);
}

void ch341_irq_disable (struct irq_data *data)
{
    ch341_irq_enable_disable (data, false);
}

int ch341_irq_set_type (struct irq_data *data, unsigned int type)
{
    struct ch341_device *ch341_dev;
    int irq;
    
    CHECK_PARAM_RET (data && (ch341_dev = irq_data_get_irq_chip_data(data)), -EINVAL);
    
    // calculate local IRQ
    irq = data->irq - ch341_dev->irq_base;

    // valid IRQ is in range 0 ... ch341_dev->irq_num-1, invalid IRQ is -1
    if (irq < 0 || irq >= ch341_dev->irq_num) return -EINVAL;
    
    ch341_dev->irq_types[irq] = type;    

    DEV_INFO (CH341_IF_ADDR, "irq=%d flow_type=%d", data->irq, type);
    
    return CH341_OK;
}

static int ch341_irq_check (struct ch341_device* ch341_dev, uint8_t irq,
                            uint8_t old, uint8_t new, bool hardware)
{
    int type;

    CHECK_PARAM_RET (old != new, CH341_OK)
    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    CHECK_PARAM_RET (irq < ch341_dev->irq_num, -EINVAL);

    // valid IRQ is in range 0 ... ch341_dev->irq_num-1, invalid IRQ is -1
    if (irq < 0 || irq >= ch341_dev->irq_num) return -EINVAL;

    // if IRQ is disabled, just return with success
    if (!ch341_dev->irq_enabled[irq]) return CH341_OK;
    
    type = ch341_dev->irq_types[irq];

    // for software IRQs dont check if IRQ is the hardware IRQ for rising edges
    if (!hardware && irq == ch341_dev->irq_hw && new > old)
        return CH341_OK;

    if ((type & IRQ_TYPE_EDGE_FALLING && old > new) ||
        (type & IRQ_TYPE_EDGE_RISING  && new > old))
    {
        DEV_DBG (CH341_IF_ADDR, "%s irq=%d %d %s", 
                  hardware ? "hardware" : "software", 
                  irq, type, (old > new) ? "falling" : "rising");

        #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,3,0)
		// handle_simple_irq (ch341_dev->irq_descs[irq]);
        handle_nested_irq(ch341_dev->irq_base+irq);
        #else
		handle_simple_irq (ch341_dev->irq_base+irq, ch341_dev->irq_descs[irq]);
        #endif
    }
    
    return CH341_OK;
}

static int ch341_irq_probe (struct ch341_device* ch341_dev)
{
    int i;
    int result;

    CHECK_PARAM_RET (ch341_dev, -EINVAL);

    DEV_DBG (CH341_IF_ADDR, "start");

    ch341_dev->irq.name         = "ch341";
    ch341_dev->irq.irq_enable   = ch341_irq_enable;
    ch341_dev->irq.irq_disable  = ch341_irq_disable;
    ch341_dev->irq.irq_set_type = ch341_irq_set_type;

    if (!ch341_dev->irq_num) return CH341_OK;

    if ((result = irq_alloc_descs(-1, 0, ch341_dev->irq_num, 0)) < 0)
    {
        DEV_ERR (CH341_IF_ADDR, "failed to allocate IRQ descriptors");
        return result;
    }
    
    ch341_dev->irq_base = result;
    
    DEV_DBG (CH341_IF_ADDR, "irq_base=%d", ch341_dev->irq_base);

    for (i = 0; i < ch341_dev->irq_num; i++)
    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)
        ch341_dev->irq_descs[i]   = irq_to_desc(ch341_dev->irq_base + i);
#endif
        ch341_dev->irq_enabled[i] = false;
        
        irq_set_chip          (ch341_dev->irq_base + i, &ch341_dev->irq);
        irq_set_chip_data     (ch341_dev->irq_base + i, ch341_dev);
        irq_clear_status_flags(ch341_dev->irq_base + i, IRQ_NOREQUEST | IRQ_NOPROBE);
    }
    
    DEV_DBG (CH341_IF_ADDR, "done");
   
    return CH341_OK;
}

static void ch341_irq_remove (struct ch341_device* ch341_dev)
{
    CHECK_PARAM (ch341_dev);

    if (ch341_dev->irq_base)
        irq_free_descs (ch341_dev->irq_base, ch341_dev->irq_num);
        
    return;
}

// ----- irq layer end ---------------------------------------------------

// ----- gpio layer begin ------------------------------------------------

void ch341_gpio_read_inputs (struct ch341_device* ch341_dev)
{
    uint32_t old_io_data;
    uint8_t old_value;
    uint8_t new_value;
    uint8_t gpio;
    int i;

    CHECK_PARAM (ch341_dev);

    // DEV_DBG (CH341_IF_ADDR, "start");

    // save old value
    old_io_data = ch341_dev->gpio_io_data;

    // read current values
    ch341_spi_get_status (ch341_dev);

    // if(old_io_data != ch341_dev->gpio_io_data) DEV_DBG (CH341_IF_ADDR, "pins changed 0x%x, oldval 0x%x, newval 0x%x", (old_io_data ^ ch341_dev->gpio_io_data), old_io_data, ch341_dev->gpio_io_data);

    for (i = 0; i < ch341_dev->irq_num; i++)
    {
        // determine local GPIO for each IRQ
        gpio = ch341_dev->irq_gpio_map[i];
            
        // determin old an new value of the bit
        old_value = (old_io_data & ch341_dev->gpio_bits[gpio]) ? 1 : 0;
        new_value = (ch341_dev->gpio_io_data & ch341_dev->gpio_bits[gpio]) ? 1 : 0;
            
        // check for interrupt
        ch341_irq_check (ch341_dev, i, old_value, new_value, false);
    }
    
    // DEV_DBG (CH341_IF_ADDR, "done");
}

// #define CH341_POLL_WITH_SLEEP

static int ch341_gpio_poll_function (void* argument)
{
    struct ch341_device* ch341_dev = (struct ch341_device*)argument;
    unsigned int next_poll_ms = jiffies_to_msecs(jiffies);
    unsigned int jiffies_ms;
    int drift_ms = 0;
    int corr_ms  = 0;
    int sleep_ms = 0;

    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    
    DEV_DBG (CH341_IF_ADDR, "start");

    while (!kthread_should_stop())
    {
        // current time in ms
        jiffies_ms = jiffies_to_msecs(jiffies);
        drift_ms   = jiffies_ms - next_poll_ms;
        
        if (poll_period == 0)
        {
            poll_period = CH341_POLL_PERIOD_MS;
            DEV_ERR (CH341_IF_ADDR,
                     "Poll period 0 ms is invalid, set back to the default of %d ms",
                     CH341_POLL_PERIOD_MS);
        }

        if (drift_ms < 0)
        {
            // period was to short, increase corr_ms by 1 ms
            // DEV_DBG (CH341_IF_ADDR, "polling GPIO is %u ms too early", -drift_ms); 
            corr_ms = (corr_ms > 0) ? corr_ms - 1 : 0;
        }   
        else if (drift_ms > 0 && drift_ms < poll_period)
        {
            // period was to long, decrease corr_ms by 1 ms
            // DEV_DBG (CH341_IF_ADDR, "polling GPIO is %u ms too late", drift_ms); 
            corr_ms = (corr_ms < poll_period) ? corr_ms + 1 : 0;
        }

        next_poll_ms = jiffies_ms + poll_period;

        // DEV_DBG (CH341_IF_ADDR, "read CH341 GPIOs");
        ch341_gpio_read_inputs (ch341_dev);

        jiffies_ms = jiffies_to_msecs(jiffies);
        
        // if GPIO read took longer than poll period, do not sleep
        if (jiffies_ms > next_poll_ms)
        {
            DEV_ERR (CH341_IF_ADDR, 
                     "GPIO poll period is too short by at least %u msecs", 
                     jiffies_ms - next_poll_ms);
        }
        else
        {
            sleep_ms = next_poll_ms - jiffies_ms - corr_ms;
            
            #ifdef CH341_POLL_WITH_SLEEP
            msleep ((sleep_ms <= 0) ? 1 : sleep_ms);
            #else
            set_current_state(TASK_UNINTERRUPTIBLE);
            schedule_timeout(msecs_to_jiffies((sleep_ms <= 0) ? 1 : sleep_ms));
            #endif
        }
    }
    #ifndef CH341_POLL_WITH_SLEEP
    __set_current_state(TASK_RUNNING);
    #endif

    complete(&ch341_dev->gpio_thread_complete);

    DEV_DBG (CH341_IF_ADDR, "stop");

    return 0;
}

int ch341_gpio_get (struct gpio_chip *chip, unsigned offset)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ch341_device* ch341_dev = (struct ch341_device*)gpiochip_get_data(chip);
    #else
    struct ch341_device* ch341_dev = container_of(chip, struct ch341_device, gpio);
    #endif
    int value;
    
    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    CHECK_PARAM_RET (offset < ch341_dev->gpio_num, -EINVAL);

    value = (ch341_dev->gpio_io_data & ch341_dev->gpio_bits[offset]) ? 1 : 0;
    
    // DEV_DBG (CH341_IF_ADDR, "offset=%u value=%d io_data=%08x", 
    //          offset, value, ch341_dev->gpio_io_data);
    
    return value;
}

// FIXME: not tested at the moment (will be introduced with kernel 4.15.0)
int ch341_gpio_get_multiple (struct gpio_chip *chip, 
                             unsigned long *mask, unsigned long *bits)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ch341_device* ch341_dev = (struct ch341_device*)gpiochip_get_data(chip);
    #else
    struct ch341_device* ch341_dev = container_of(chip, struct ch341_device, gpio);
    #endif
    int i;
    
    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    CHECK_PARAM_RET (mask, -EINVAL);
    CHECK_PARAM_RET (bits, -EINVAL);

    *bits = 0; // init to zero

    for (i = 0; i < ch341_dev->gpio_num; i++)
        if (*mask & (1 << i))
        {
            *bits &= ~(1 << i);
            *bits |= (((ch341_dev->gpio_io_data & ch341_dev->gpio_bits[i]) ? 1 : 0) << i);
        }

    // DEV_DBG (CH341_IF_ADDR, "mask=%08lx bit=0x%08lx io_data=0x%08x", *mask, *bits, ch341_dev->gpio_io_data);

    return CH341_OK;
}

void ch341_gpio_set (struct gpio_chip *chip, unsigned offset, int value)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ch341_device* ch341_dev = (struct ch341_device*)gpiochip_get_data(chip);
    #else
    struct ch341_device* ch341_dev = container_of(chip, struct ch341_device, gpio);
    #endif
    
    CHECK_PARAM (ch341_dev);
    CHECK_PARAM (offset < ch341_dev->gpio_num);
    CHECK_PARAM (ch341_dev->gpio_pins[offset]->mode != CH341_PIN_MODE_IN);

    if (value) {
        ch341_dev->gpio_io_data |= ch341_dev->gpio_bits[offset];
    }
    else
        ch341_dev->gpio_io_data &= ~ch341_dev->gpio_bits[offset];

    DEV_DBG (CH341_IF_ADDR, "name=%s value=%d io_data=0x%08x", 
              ch341_dev->gpio_pins[offset]->name, value, ch341_dev->gpio_io_data);

    ch341_spi_write_outputs (ch341_dev);
}

void ch341_gpio_set_multiple (struct gpio_chip *chip, 
                              unsigned long *mask, unsigned long *bits)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ch341_device* ch341_dev = (struct ch341_device*)gpiochip_get_data(chip);
    #else
    struct ch341_device* ch341_dev = container_of(chip, struct ch341_device, gpio);
    #endif
    int i;
    
    CHECK_PARAM (ch341_dev);
    CHECK_PARAM (mask);
    CHECK_PARAM (bits);

    for (i = 0; i < ch341_dev->gpio_num; i++) {
        uint8_t mode = ch341_dev->gpio_pins[i]->mode;

        if (*mask & (1 << i)) 
        {
            if (mode != CH341_PIN_MODE_IN) // if out or CS we will set the pin
            {
                if (*bits & (1 << i)) 
                    ch341_dev->gpio_io_data |= ch341_dev->gpio_bits[i];
                else
                    ch341_dev->gpio_io_data &= ~ch341_dev->gpio_bits[i];
            }
        }
    }
        
    // DEV_DBG (CH341_IF_ADDR, "mask=%08lx bit=%08lx io_data=0x%08x", *mask, *bits, ch341_dev->gpio_io_data);

    ch341_spi_write_outputs (ch341_dev);

    return;
}


int ch341_gpio_get_direction (struct gpio_chip *chip, unsigned offset)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ch341_device* ch341_dev = (struct ch341_device*)gpiochip_get_data(chip);
    #else
    struct ch341_device* ch341_dev = container_of(chip, struct ch341_device, gpio);
    #endif
    int mode;
    
    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    CHECK_PARAM_RET (offset < ch341_dev->gpio_num, -EINVAL);

    mode = (ch341_dev->gpio_pins[offset]->mode == CH341_PIN_MODE_IN) ? 1 : 0;

    DEV_DBG (CH341_IF_ADDR, "gpio=%s dir=%d", ch341_dev->gpio_pins[offset]->name, mode);

    return mode;
}

int ch341_gpio_set_direction (struct gpio_chip *chip, unsigned offset, bool input)
{
    uint32_t msk;
    bool currentlyInput;

    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ch341_device* ch341_dev = (struct ch341_device*)gpiochip_get_data(chip);
    #else
    struct ch341_device* ch341_dev = container_of(chip, struct ch341_device, gpio);
    #endif

    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    CHECK_PARAM_RET (offset < ch341_dev->gpio_num, -EINVAL);

    currentlyInput = ch341_dev->gpio_pins[offset]->mode == CH341_PIN_MODE_IN;
    if(input == currentlyInput) {
        DEV_INFO (CH341_IF_ADDR, "no-change gpio=%s direction=%s", ch341_dev->gpio_pins[offset]->name, input ? "input" :  "output");
    }
    else {
        // pin configured correctly?
        msk = 1 << ch341_dev->gpio_pins[offset]->bitNum;
        if (!(msk & ch341_dev->gpio_out_ok_mask) && !input)
        {
            DEV_ERR(CH341_IF_ADDR, "pin must be an input");
            return -EINVAL;
        }

        if (!(msk & ch341_dev->gpio_in_ok_mask) && input)
        {
            DEV_ERR(CH341_IF_ADDR, "pin must be an output");
            return -EINVAL;
        }    

        DEV_INFO (CH341_IF_ADDR, "gpio=%s direction=%s", ch341_dev->gpio_pins[offset]->name, input ? "input" :  "output");

        // We allow users to set chip select pins to be "outputs", and let those users write directly to those pins
        // but we don't want to forget that the pin is special
        if(ch341_dev->gpio_pins[offset]->mode != CH341_PIN_MODE_CS)
            ch341_dev->gpio_pins[offset]->mode = input ? CH341_PIN_MODE_IN : CH341_PIN_MODE_OUT;

        // mask in / mask out the according bit in direction mask
        if (ch341_dev->gpio_pins[offset]->mode != CH341_PIN_MODE_IN)    
            ch341_dev->gpio_mask |= ch341_dev->gpio_bits[offset]; // if in CS or OUT mode
        else
            ch341_dev->gpio_mask &= ~ch341_dev->gpio_bits[offset];
    }
    
    return CH341_OK;
}

int ch341_gpio_direction_input (struct gpio_chip *chip, unsigned offset)
{
    return ch341_gpio_set_direction (chip, offset, true);
}

int ch341_gpio_direction_output (struct gpio_chip *chip, unsigned offset, int value)
{
    int result = CH341_OK;

    if ((result = ch341_gpio_set_direction (chip, offset, false)) == CH341_OK)
        // set initial output value
        ch341_gpio_set (chip, offset, value);
    
    return result;
}

int ch341_gpio_to_irq (struct gpio_chip *chip, unsigned offset)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ch341_device* ch341_dev = (struct ch341_device*)gpiochip_get_data(chip);
    #else
    struct ch341_device* ch341_dev = container_of(chip, struct ch341_device, gpio);
    #endif
    int irq;
        
    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    CHECK_PARAM_RET (offset < ch341_dev->gpio_num, -EINVAL);

    // valid IRQ is in range 0 ... ch341_dev->irq_num, invalid IRQ is -1
    irq = ch341_dev->gpio_irq_map[offset];
    irq = (irq >= 0 ? ch341_dev->irq_base + irq : 0);

    DEV_DBG (CH341_IF_ADDR, "gpio=%d irq=%d", offset, irq);

    return irq;
}


static int ch341_gpio_probe (struct ch341_device* ch341_dev)
{
    struct gpio_chip *gpio = &ch341_dev->gpio;
    int result;
    // int i, j = 0;

    CHECK_PARAM_RET (ch341_dev, -EINVAL);
    
    DEV_DBG (CH341_IF_ADDR, "start");

    gpio->label     = "ch341";

    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    gpio->parent = &ch341_dev->usb_dev->dev;
    #else
    gpio->dev    = &ch341_dev->usb_dev->dev;
    #endif

    gpio->owner  = THIS_MODULE;
    gpio->request= NULL;
    gpio->free   = NULL;
    
    gpio->base   = -1;   // request dynamic ID allocation
    gpio->ngpio  = ch341_dev->gpio_num;
    
    gpio->can_sleep = 1;
    gpio->names     = (void*)ch341_dev->gpio_names;

    gpio->get_direction     = ch341_gpio_get_direction;
    gpio->direction_input   = ch341_gpio_direction_input;
    gpio->direction_output  = ch341_gpio_direction_output;
    gpio->get               = ch341_gpio_get;
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)
    gpio->get_multiple      = ch341_gpio_get_multiple;
    #endif
    gpio->set               = ch341_gpio_set;
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0)
    gpio->set_multiple      = ch341_gpio_set_multiple;
    #endif
    
    gpio->to_irq            = ch341_gpio_to_irq;

    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    if ((result = gpiochip_add_data (gpio, ch341_dev)))
    #else
    if ((result = gpiochip_add (gpio)))
    #endif
    {
        DEV_ERR (CH341_IF_ADDR, "failed to register GPIOs: %d", result);
        // in case of error, reset gpio->base to avoid crashes during free 
        gpio->base   = -1;
        return result;
    }

    DEV_DBG (CH341_IF_ADDR, "registered GPIOs from %d to %d", 
             gpio->base, gpio->base + gpio->ngpio - 1);

#if 0
    // @geeksville comment: It seems this code breaks usage on (newer?) kernels.  With kernel 5.8.0
    // any attempts to set gpio status results in EBUSY error getting returned because it thinks some
    // other driver (the ch341 driver) already owns this GPIO exclusively.

    for (i = 0; i < CH341_GPIO_NUM_PINS; i++)
        // in case the pin is not a CS signal, it is an GPIO pin
        if (ch341_board_config[i].mode != CH341_PIN_MODE_CS)
        {
            // add and export the GPIO pin
            if ((result = gpio_request(gpio->base + j, ch341_board_config[i].name)) ||
                (result = gpio_export (gpio->base + j, ch341_board_config[i].pin != 21 ? true : false)))
            {
                DEV_ERR (CH341_IF_ADDR, "failed to export GPIO %s: %d", 
                         ch341_board_config[i].name, result);
                // reduce number of GPIOs to avoid crashes during free in case of error
                ch341_dev->gpio_num = j ? j-1 : 0;
                return result;
            }
            j++;
        }
#endif

    init_completion(&ch341_dev->gpio_thread_complete);
    ch341_dev->gpio_thread = kthread_run (&ch341_gpio_poll_function, ch341_dev, "spi-ch341-usb-poll");

    DEV_DBG (CH341_IF_ADDR, "done");

    return 0;
}

static void ch341_gpio_remove (struct ch341_device* ch341_dev)
{
    int i;

    CHECK_PARAM (ch341_dev);

    if (ch341_dev->gpio_thread)
    {
        kthread_stop(ch341_dev->gpio_thread);
        wake_up_process (ch341_dev->gpio_thread);
        wait_for_completion(&ch341_dev->gpio_thread_complete);
    }
        
    if (ch341_dev->gpio.base > 0)
    {
        for (i = 0; i < ch341_dev->gpio_num; ++i)
           gpio_free(ch341_dev->gpio.base + i);

        gpiochip_remove(&ch341_dev->gpio);
    }
       
    return;
}

// ----- gpio layer end --------------------------------------------------

// ----- usb layer begin -------------------------------------------------

static const struct usb_device_id ch341_usb_table[] = {
    { USB_DEVICE(0x1a86, 0x5512) },
    { }
};

MODULE_DEVICE_TABLE(usb, ch341_usb_table);

static int ch341_usb_transfer(struct ch341_device *ch341_dev, int out_len, int in_len)
{
    int retval;
    int actual;

    // DEV_DBG (CH341_IF_ADDR, "bulk_out %d bytes, bulk_in %d bytes", 
    //          out_len, (in_len == 0) ? 0 : CH341_USB_MAX_BULK_SIZE);

    retval = usb_bulk_msg(ch341_dev->usb_dev, 
                          usb_sndbulkpipe(ch341_dev->usb_dev,
                                          usb_endpoint_num(ch341_dev->ep_out)),
                          ch341_dev->out_buf, out_len, 
                          &actual, 2000);
    if (retval < 0) { // -110 means timed out
        DEV_ERR (CH341_IF_ADDR, "usb_bulk write failed %d", retval);
        return retval;
    }

    if (in_len == 0)
        return actual;

    memset(ch341_dev->in_buf, 0, sizeof(ch341_dev->in_buf));
    retval = usb_bulk_msg(ch341_dev->usb_dev, 
                          usb_rcvbulkpipe(ch341_dev->usb_dev, 
                                          usb_endpoint_num(ch341_dev->ep_in)),
                          ch341_dev->in_buf, CH341_USB_MAX_BULK_SIZE, 
                          &actual, 2000);

    if (retval < 0) {
        DEV_ERR (CH341_IF_ADDR, "usb_bulk read failed %d", retval);
        return retval;
    }

    return actual;
}

static void ch341_usb_complete_intr_urb (struct urb *urb)
{
    struct ch341_device *ch341_dev;

    CHECK_PARAM (urb);
    CHECK_PARAM (ch341_dev = urb->context);

    if (!urb->status)
    {
        uint32_t mask = ch341_dev->gpio_bits[ch341_dev->irq_gpio_map[ch341_dev->irq_hw]];
        bool wasHigh = !!(ch341_dev->gpio_io_data & mask);

        // hardware IRQs are only generated for one IRQ and rising edges 0 -> 1
        // DEV_DBG (CH341_IF_ADDR, "hw irq rise %x", mask);

        // because of asynchronous GPIO read, the GPIO value has to be set to 1
        ch341_dev->gpio_io_data |= mask;
        
        // IRQ has to be triggered
        ch341_irq_check (ch341_dev, ch341_dev->irq_hw, wasHigh, 1, true);
        
        // submit next request
        usb_submit_urb(ch341_dev->intr_urb, GFP_ATOMIC);
    }
    else {
        // Never invoked
        // DEV_DBG (CH341_IF_ADDR, "irq fall");
    }
}

static void ch341_usb_free_device (struct ch341_device* ch341_dev)
{
    CHECK_PARAM (ch341_dev)

    ch341_gpio_remove (ch341_dev);
    ch341_irq_remove  (ch341_dev);
    ch341_spi_remove  (ch341_dev);
    ch341_cfg_remove  (ch341_dev);
        
    if (ch341_dev->intr_urb) usb_free_urb (ch341_dev->intr_urb);

    usb_set_intfdata (ch341_dev->usb_if, NULL);
    usb_put_dev (ch341_dev->usb_dev);

    kfree (ch341_dev);
}

static int ch341_usb_probe (struct usb_interface* usb_if,
                            const struct usb_device_id* usb_id)
{
    struct usb_device* usb_dev = usb_get_dev(interface_to_usbdev(usb_if));
    struct usb_endpoint_descriptor *epd;
    struct usb_host_interface *settings;
    struct ch341_device* ch341_dev;
    int i;
    int error;

    DEV_DBG (&usb_if->dev, "connect device");
    
    // create and initialize a new device data structure
    if (!(ch341_dev = kzalloc(sizeof(struct ch341_device), GFP_KERNEL)))
    {
        DEV_ERR (&usb_if->dev, "could not allocate device memory");
        usb_put_dev (ch341_dev->usb_dev);
        return -ENOMEM;
    } 

    // save USB device data
    ch341_dev->usb_dev = usb_dev;
    ch341_dev->usb_if  = usb_if;
    
    // find endpoints
    settings = usb_if->cur_altsetting;
    DEV_DBG (CH341_IF_ADDR, "bNumEndpoints=%d", settings->desc.bNumEndpoints);
    
    for (i = 0; i < settings->desc.bNumEndpoints; i++)
    {
        epd = &settings->endpoint[i].desc;

        DEV_DBG (CH341_IF_ADDR, "  endpoint=%d type=%d dir=%d addr=%0x", i, 
                 usb_endpoint_type(epd), usb_endpoint_dir_in(epd), 
                 usb_endpoint_num(epd));

        if (usb_endpoint_is_bulk_in (epd)) ch341_dev->ep_in   = epd; else
        if (usb_endpoint_is_bulk_out(epd)) ch341_dev->ep_out  = epd; else
        if (usb_endpoint_xfer_int   (epd)) ch341_dev->ep_intr = epd;
    }
    // create URBs for handling interrupts
    if (!(ch341_dev->intr_urb = usb_alloc_urb(0, GFP_KERNEL)))
    {
		DEV_ERR (&usb_if->dev, "failed to alloc URB");
		ch341_usb_free_device (ch341_dev);
		return -ENOMEM;
	}    
	
	usb_fill_int_urb (ch341_dev->intr_urb, ch341_dev->usb_dev,
                      usb_rcvintpipe(ch341_dev->usb_dev, 
                                     usb_endpoint_num(ch341_dev->ep_intr)),
                      ch341_dev->intr_buf, CH341_USB_MAX_INTR_SIZE,
                      ch341_usb_complete_intr_urb, ch341_dev, 
                      ch341_dev->ep_intr->bInterval);

    // save the pointer to the new ch341_device in USB interface device data
    usb_set_intfdata(usb_if, ch341_dev);
    
    if ((error = ch341_cfg_probe (ch341_dev)) ||  // initialize board configuration    
        (error = ch341_spi_probe (ch341_dev)) ||  // initialize SPI master and slaves
        (error = ch341_irq_probe (ch341_dev)) ||  // initialize IRQs
        (error = ch341_gpio_probe(ch341_dev)))    // initialize GPIOs
    {
        ch341_usb_free_device (ch341_dev);
        return error;
    }

    usb_submit_urb (ch341_dev->intr_urb, GFP_ATOMIC);

    DEV_INFO (CH341_IF_ADDR, "connected");

    return CH341_OK;
}

static void ch341_usb_disconnect(struct usb_interface *usb_if)
{
    struct ch341_device* ch341_dev = usb_get_intfdata(usb_if);
    
    DEV_INFO (CH341_IF_ADDR, "disconnected");
    
    ch341_usb_free_device (ch341_dev);
}

/* static int ch341_usb_suspend(struct usb_interface *intf, pm_message_t message) {
    return -1; // Disable USB suspend
} */

static struct usb_driver ch341_usb_driver = {
    .name       = "spi-ch341-usb",
    .id_table   = ch341_usb_table,
    .probe      = ch341_usb_probe,
    .disconnect = ch341_usb_disconnect
    // .suspend    = ch341_usb_suspend
};

module_usb_driver(ch341_usb_driver);

// ----- usb layer end ---------------------------------------------------

MODULE_ALIAS("spi:ch341");
MODULE_AUTHOR("Gunar Schorcht <gunar@schorcht.net>");
MODULE_DESCRIPTION("spi-ch341-usb driver v1.0.1");
MODULE_LICENSE("GPL");

module_param(poll_period, uint, 0644);
MODULE_PARM_DESC(poll_period, "GPIO polling period in ms (default 100 ms)");

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)

