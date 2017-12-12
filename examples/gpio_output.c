/*
 * Simple program for testing CH341A GPIO output using sysfs
 *
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * Compile: gcc -o gpio_output gpio_output.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

int main(int argc, char *argv[]) 
{
	if (argc != 2)
	{
    	fprintf(stderr, "usage: sudo %s <file_name>\n\n", argv[0]);
    	fprintf(stderr, "    where <file_name> is the name of a sysfs gpio value file.\n");
    	fprintf(stderr, "    example: %s /sys/class/gpio/gpio4/value\n\n", argv[0]);
    	return -1;
    }
    
	int fd;
	
	if ((fd = open(argv[1], O_RDWR)) == -1) 
	{
		perror("open");
		return -1;
	}

    int count = 0;
    
	while (1) 
	{
		if (write(fd, count % 2 ? "1" : "0", 1) == -1) 
		{
			perror("write");
			return -1;
		}
		count++;
		sleep(1);
	}

	return 0;
}
