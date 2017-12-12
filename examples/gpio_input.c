/*
 * Simple program for testing CH341A GPIO interrupt input using sysfs
 *
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * Compile: gcc -o gpio_input gpio_input.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>

int main(int argc, char *argv[]) 
{
	if (argc != 2)
	{
    	fprintf(stderr, "Usage: sudo %s <file_name>\n\n", argv[0]);
    	fprintf(stderr, "    where <file_name> is the name of a sysfs GPIO value file,\n");
    	fprintf(stderr, "    e.g.: sudo %s /sys/class/gpio/gpio4/value\n\n", argv[0]);
    	fprintf(stderr, "Please note: interrupt has to be activated before by root with command\n\n");
    	fprintf(stderr, "   echo both > /sys/class/gpio/gpio4/edge\n\n");
    	return -1;
    }
    
	int fd;
	
	if ((fd = open(argv[1], O_RDWR)) == -1) 
	{
		perror("open");
		return -1;
	}

	struct pollfd fds[1];
	
    fds[0].fd = fd;
	fds[0].events = POLLPRI;

	char buf;

	while (1) 
	{
	    // wait for new GPIO value interrupt
		if (poll(fds, 1, -1) == -1) 
		{
			perror("poll");
			return -1;
		}

        // read one char from GPIO value file
		if (read(fd, &buf, 1) == -1) 
		{
			perror("read");
			return -1;
		}

        // rewind GPIO value file to first char
		if (lseek(fd, 0, SEEK_SET) == -1) {
			perror("lseek");
			return -1;
		}

		printf("new value on pin %s, value = %c\n", argv[1], buf);
	}

	return 0;
}
