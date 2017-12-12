PWD         := $(shell pwd) 
KVERSION    := $(shell uname -r)
KERNEL_DIR   = /usr/src/linux-headers-$(KVERSION)/
MODULE_DIR   = /lib/modules/$(KVERSION)

MODULE_NAME  = spi-ch341-usb
obj-m       := $(MODULE_NAME).o   

$(MODULE_NAME).ko: $(MODULE_NAME).c
	make -C $(KERNEL_DIR) M=$(PWD) modules

all:
	make -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f examples/gpio_input examples/gpio_output

install: $(MODULE_NAME).ko
	cp $(MODULE_NAME).ko $(MODULE_DIR)/kernel/drivers/spi
	depmod
	
uninstall:
	rm -f $(MODULE_DIR)/kernel/drivers/spi/$(MODULE_NAME).ko
	depmod
