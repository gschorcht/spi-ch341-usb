/*
 * Driver for the CH341 USB-SPI/I2C adapter
 *
 * Copyright (c) 2017 Gunar Schorcht
 *
 * Derived from:
 *  
 *  i2c-ch341-usb.c Copyright (c) 2016 Tse Lun Bien
 *  i2c-ch341.c     Copyright (c) 2014 Marco Gittler
 *  i2c-tiny-usb.c  Copyright (c) 2006-2007 Till Harbaum (Till@Harbaum.org)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */
// #define DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>

#define CH341_USB_MAX_TRANSFER_SIZE 32    // CH341A wMaxPacketSize

#define CH341_CMD_SPI_STREAM        0xA8  // SPI command
#define CH341_CMD_UIO_STREAM        0xAB  // UIO command

#define CH341_CMD_UIO_STM_IN        0x00  // UIO interface IN  command (D0~D7)
#define CH341_CMD_UIO_STM_OUT       0x80  // UIO interface OUT command (D0~D5)
#define CH341_CMD_UIO_STM_DIR       0x40  // UIO interface DIR command (D0~D5)
#define CH341_CMD_UIO_STM_END       0x20  // UIO interface END command
#define CH341_CMD_UIO_STM_US        0xc0  // UIO interface US  command

#define CH341_SPI_NUM_DEVICES       3
#define CH341_SPI_MIN_FREQ          400
#define CH341_SPI_MAX_FREQ          1e6
#define CH341_SPI_MIN_BITS_PER_WORD 4
#define CH341_SPI_MAX_BITS_PER_WORD 32

static struct spi_board_info ch341_spi_devices[CH341_SPI_NUM_DEVICES] = 
{
    {
        .modalias     = "spidev",
        .max_speed_hz = CH341_SPI_MAX_FREQ,
        .bus_num      = 0,
        .chip_select  = 0,
        .mode         = SPI_MODE_0,
    }, 
    {
        .modalias     = "spidev",
        .chip_select  = 1,
        .max_speed_hz = CH341_SPI_MAX_FREQ,
        .bus_num      = 0,
        .mode         = SPI_MODE_0,
    },
    {
        .modalias     = "spidev",
        .chip_select  = 2,
        .max_speed_hz = CH341_SPI_MAX_FREQ,
        .bus_num      = 0,
        .mode         = SPI_MODE_0,
    },
}; 

// device specific structure
struct ch341_device 
{
    struct usb_device*    usb_dev; // usb device
    struct usb_interface* usb_if;  // usb interface
    struct spi_master*    master;  // spi master
    struct spi_device*    slaves[CH341_SPI_NUM_DEVICES];

    int ep_in;
    int ep_out;

    uint8_t in_buf [CH341_USB_MAX_TRANSFER_SIZE];
    uint8_t out_buf[CH341_USB_MAX_TRANSFER_SIZE];
};

static int ch341_usb_transfer (struct ch341_device *dev, int out_len, int in_len);

// ----- begin of spi layer ----------------------------------------------

static void ch341_set_cs (struct spi_device *spi, bool active)
{
    static const int csio[CH341_SPI_NUM_DEVICES] = {0x36, 0x35, 0x33};
    struct ch341_device* ch341_dev = spi_master_get_devdata(spi->master);
    
    if (!ch341_dev) return;

    dev_dbg (&ch341_dev->usb_if->dev, "%s: active %s\n", __FUNCTION__, active ? "true" : "false");

    if (spi->chip_select > CH341_SPI_NUM_DEVICES)
    {
        dev_err (&spi->dev, "invalid CS value %d, 0~%d are available", 
                 spi->chip_select, CH341_SPI_NUM_DEVICES-1);
        return;
    }
   
    ch341_dev->out_buf[0] = CH341_CMD_UIO_STREAM;
    ch341_dev->out_buf[1] = CH341_CMD_UIO_STM_DIR | 0x3f;
    ch341_dev->out_buf[2] = CH341_CMD_UIO_STM_OUT | (active ? csio[spi->chip_select] : 0x37);
    ch341_dev->out_buf[3] = CH341_CMD_UIO_STM_END;

    ch341_usb_transfer(ch341_dev, 4, 0);
}


static uint8_t ch341_swap_byte(const uint8_t byte)
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


// Implementation of bit banging protocol uses following IOs to be compatible
// with the hardware SPI interface
//
//      D7     D6     D5     D4     D3     D2     D1     D0
//      MISO   IN2    MOSI   OUT2   SCK    CS2    CS1    CS0

static int ch341_bitbang (struct ch341_device* ch341_dev, 
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

    uint8_t CH341_SCK_H = 1 << 3;
    uint8_t CH341_SCK_L = ~CH341_SCK_H & 0x08;
    uint8_t CH341_CPOL  = (spi->mode & SPI_CPOL) ? 0x08 : 0;
    uint8_t CH341_CS_H  = 0x07;
    uint8_t CH341_CS_L  = CH341_CS_H & ~(1 << spi->chip_select);
    
    uint8_t mode = spi->mode & SPI_MODE_3;
    bool    lsb  = spi->mode & SPI_LSB_FIRST;

    k = 0;
    io[k++] = CH341_CMD_UIO_STREAM;
    io[k++] = CH341_CMD_UIO_STM_OUT | CH341_CS_H | CH341_CPOL; // set defaults CS#=HIGH, SCK=CPOL
    io[k++] = CH341_CMD_UIO_STM_DIR | 0x3f;                    // input: MISO, IN2; output MOSI, OUT2, SCK, CS#;
    io[k++] = CH341_CMD_UIO_STM_OUT | CH341_CS_L | CH341_CPOL; // start with CS0=LOW, SCK=CPOL
    io[k++] = CH341_CMD_UIO_STM_END;
    if ((result = ch341_usb_transfer(ch341_dev, k, 0)) < 0)
        return result;
    
    for (b = 0; b < len; b++)
    {
        k = 0;
        io[k++] = CH341_CMD_UIO_STREAM;
        
        byte = lsb ? ch341_swap_byte(tx[b]) : tx[b];
        for (i = 0; i < 8; i++)
        {
            bit = byte & 0x80 ? 0x20 : 0;  // lsb
            byte = byte << 1;
            
            if (mode == SPI_MODE_0 || mode == SPI_MODE_3)
            {
                io[k++] = CH341_CMD_UIO_STM_OUT | CH341_CS_L | CH341_SCK_L | bit; // keep CS0=LOW, set SCK=LOW , set MOSI
                io[k++] = CH341_CMD_UIO_STM_OUT | CH341_CS_L | CH341_SCK_H | bit; // keep CS0=LOW, set SCK=HIGH, keep MOSI
                io[k++] = CH341_CMD_UIO_STM_IN; // read MISO
            }
            else
            {
                io[k++] = CH341_CMD_UIO_STM_OUT | CH341_CS_L | CH341_SCK_L | bit; // keep CS0=LOW, set SCK=HIGH, set MOSI
                io[k++] = CH341_CMD_UIO_STM_OUT | CH341_CS_L | CH341_SCK_H | bit; // keep CS0=LOW, set SCK=LOW , keep MOSI
                io[k++] = CH341_CMD_UIO_STM_IN; // read MISO
            }
        }
        io[k++] = CH341_CMD_UIO_STM_OUT | CH341_CS_L | CH341_CPOL; // keep CS0=LOW, SCK=CPOL, MOSI=LOW
        io[k++] = CH341_CMD_UIO_STM_END;
        if ((result = ch341_usb_transfer(ch341_dev, k, 8)) < 0)
            return result;

        byte = 0;
        for (i = 0; i < 8; i++)
        {
            byte = byte << 1;
            byte = byte | ((ch341_dev->in_buf[i] & 0x80) ? 1 : 0);
        }
        rx[b] =  lsb ? ch341_swap_byte(byte) : byte;
    }
    
    k = 0;
    io[k++] = CH341_CMD_UIO_STREAM;
    io[k++] = CH341_CMD_UIO_STM_OUT | CH341_CS_H | CH341_CPOL; // default status: CS#=HIGH, SCK=CPOL
    io[k++] = CH341_CMD_UIO_STM_END;
    if ((result = ch341_usb_transfer(ch341_dev, k, 0)) < 0)
        return result;
    
    return 0;
}


static int ch341_transfer_one(struct spi_master *master,
                              struct spi_device *spi, 
                              struct spi_transfer* t)
{
    struct ch341_device* ch341_dev = spi_master_get_devdata(spi->master);
    const uint8_t* tx;
    uint8_t* rx;
    bool lsb; 
    int result;
    int i;

    if (!ch341_dev || !master || !spi || !t) return EIO; 
    
    dev_dbg (&ch341_dev->usb_if->dev, "%s\n", __FUNCTION__);

    // use slow bitbang implementation for SPI_MODE_1, SPI_MODE_2 and SPI_MODE_3
    if (spi->mode & SPI_MODE_3)
    {    
        if ((result = ch341_bitbang (ch341_dev, spi, t->tx_buf, t->rx_buf, t->len)) < 0)
        {
            spi_finalize_current_transfer(master);
            return result;
        }
    }
    
    // otherwise the faster hardware implementation    
    else
    {
        if ((t->len + 1) > CH341_USB_MAX_TRANSFER_SIZE)
            return -EIO;

        lsb = spi->mode & SPI_LSB_FIRST;
        tx  = t->tx_buf;
        rx  = t->rx_buf;
    
        // activate cs
        ch341_set_cs (spi, true);

        // fill output buffer with command and output data, controller expects lsb first
        ch341_dev->out_buf[0] = CH341_CMD_SPI_STREAM;
        for (i = 0; i < t->len; i++)
            ch341_dev->out_buf[i+1] = lsb ? tx[i] : ch341_swap_byte(tx[i]);

        // transfer output and input data
        result = ch341_usb_transfer(ch341_dev, t->len + 1, t->len);

        // deactivate cs
        ch341_set_cs (spi, false);

        if (result < 0) 
        {
            spi_finalize_current_transfer(master);
            return result;
        }
    
        // fill input data with input buffer, controller delivers lsb first
        if (rx)
            for (i = 0; i < t->len; i++)
            rx[i] = lsb ? ch341_dev->in_buf[i] : ch341_swap_byte(ch341_dev->in_buf[i]);
    }

    spi_finalize_current_transfer(master);
    
    return 0;
}

// ----- end of spi layer ------------------------------------------------

// ----- begin of usb layer ----------------------------------------------

static const struct usb_device_id ch341_spi_table[] = {
    { USB_DEVICE(0x1a86, 0x5512) },
    { }
};

MODULE_DEVICE_TABLE(usb, ch341_spi_table);

static int ch341_usb_transfer(struct ch341_device *dev, int out_len, int in_len)
{
    int retval;
    int actual;

    dev_dbg(&dev->usb_if->dev, "%s: bulk_out %d bytes, bulk_in %d bytes\n", 
            __FUNCTION__, out_len, (in_len == 0) ? 0 : CH341_USB_MAX_TRANSFER_SIZE);

    retval = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
                          dev->out_buf, out_len, &actual, 2000);
    if (retval < 0)
        return retval;
    if (in_len == 0)
        return actual;

    memset(dev->in_buf, 0, sizeof(dev->in_buf));
    retval = usb_bulk_msg(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
                          dev->in_buf, CH341_USB_MAX_TRANSFER_SIZE, &actual, 2000);
    if (retval < 0)
        return retval;

    return actual;
}

static int ch341_spi_probe (struct usb_interface* usb_if,
                            const struct usb_device_id* usb_id)
{
    struct usb_device*   usb_dev = usb_get_dev(interface_to_usbdev(usb_if));
    struct ch341_device* ch341_dev;
    struct spi_master*   master;

    int result;
    int bus = 0;
    int i;

    dev_dbg(&usb_if->dev, "%s: connect device\n", __FUNCTION__);
    
    // -- SPI adapter part --------------------------------
    
    // search for next free bus number
 
    while ((master = spi_busnum_to_master(bus)))
    { 
        // returns a refcounted pointer to an existing master
        spi_master_put (master);
        bus++;
    }

    master = spi_alloc_master(&usb_if->dev, sizeof(ch341_dev));
    if (!master)
    {
        dev_err(&usb_dev->dev, "SPI master allocation failed\n");
        return -ENOMEM;
    }
    
    dev_dbg(&usb_if->dev, "%s: master connected to SPI bus %d\n", __FUNCTION__, bus);

    master->bus_num = bus;
    master->num_chipselect = CH341_SPI_NUM_DEVICES;
    master->mode_bits = SPI_MODE_3 | SPI_LSB_FIRST;
    master->bits_per_word_mask = SPI_BIT_MASK(8);
    master->flags = SPI_MASTER_MUST_RX | SPI_MASTER_MUST_TX;
    master->transfer_one = ch341_transfer_one;
    master->max_speed_hz = CH341_SPI_MAX_FREQ;
    master->min_speed_hz = CH341_SPI_MIN_FREQ;

    ch341_dev = spi_master_get_devdata(master);

    ch341_dev->usb_dev = usb_dev;
    ch341_dev->usb_if  = usb_if;
    ch341_dev->master  = master;

    ch341_dev->ep_out = usb_if->cur_altsetting->endpoint[1].desc.bEndpointAddress;
    ch341_dev->ep_in  = usb_if->cur_altsetting->endpoint[0].desc.bEndpointAddress;
       
    // if ((result = devm_spi_register_master (&usb_dev->dev, master)))
    if ((result = spi_register_master (master)))
    {
        dev_err(&usb_dev->dev, "could not register SPI master\n");
        spi_master_put(master);
        return result;
    }
    
    for (i = 0; i < CH341_SPI_NUM_DEVICES; i++)
    {
        ch341_spi_devices[i].bus_num = bus;
        ch341_dev->slaves[i] = spi_new_device(master, &ch341_spi_devices[i]);
        if (ch341_dev->slaves[i])
        {
            dev_dbg (&usb_if->dev, "%s: SPI device /dev/spidev%d.%d created\n", 
                     __FUNCTION__, bus, i);
            ch341_set_cs (ch341_dev->slaves[i], false);
        }
    }

    // save our data pointer in this interface device
    usb_set_intfdata(usb_if, ch341_dev);

    dev_info(&usb_if->dev, "connected\n");
    
    return 0;    

}

static void ch341_spi_disconnect(struct usb_interface *usb_if)
{
    struct ch341_device* ch341_dev = usb_get_intfdata(usb_if);
    int i;
    
    dev_info(&usb_if->dev, "disconnect\n");
    
    for (i = 0; i < CH341_SPI_NUM_DEVICES; i++)
        spi_unregister_device(ch341_dev->slaves[i]);

    spi_unregister_master (ch341_dev->master);
    spi_master_put(ch341_dev->master);

    usb_set_intfdata(usb_if, NULL);
    usb_put_dev(ch341_dev->usb_dev);
}

static struct usb_driver ch341_spi_driver = {
    .name       = "spi-ch341-usb",
    .id_table   = ch341_spi_table,
    .probe      = ch341_spi_probe,
    .disconnect = ch341_spi_disconnect
};

module_usb_driver(ch341_spi_driver);

// ----- end of usb layer ------------------------------------------------

MODULE_ALIAS("spi:ch341");
MODULE_AUTHOR("Gunar Schorcht <gunar@schorcht.net>");
MODULE_DESCRIPTION("spi-ch341-usb driver v0.2");
MODULE_LICENSE("GPL");
