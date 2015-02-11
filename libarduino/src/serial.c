/**
 * @file Serial.cpp
 * @brief 
 *
 * @date Feb 4, 2015
 * @author carrier.nicolas0@gmail.com
 */
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>

#include <unistd.h>

#include <assert.h>
#include <time.h>
#include <stdio.h>

#include "Serial.h"

struct HwSerial Serial;

/* TODO */
int readBytes(__attribute__((unused)) void *buffer,
		__attribute__((unused)) size_t len)
{
	fd_set readfds;
	ssize_t sret;
	int ret;
	struct timeval timeout;

	timeout.tv_sec = Serial.timeout / 1000;
	timeout.tv_usec = 1000 * (Serial.timeout % 1000);
	FD_ZERO(&readfds);
	FD_SET(STDIN_FILENO, &readfds);
	ret = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
	switch (ret) {
	case -1:
		perror("select");
		return 0;
	case 0:
		/* timeout */
		return 0;

	default:
		assert(FD_ISSET(STDIN_FILENO, &readfds));
		sret = read(STDIN_FILENO, buffer, len);
		if (sret == -1) {
			perror("read");
			return 0;
		}
		return sret;
	}

	/* never reached */
	return 0;
}

static int print(const char *s)
{
	return printf(s);
}

static int printi(int i)
{
	return printf("%d", i);
}

void begin(__attribute__((unused)) unsigned long baud,
		__attribute__((unused)) uint8_t config)
{

}

void setTimeout(int timeout)
{
	Serial.timeout = timeout;
}

static void __attribute__ ((constructor)) serial_init(void)
{
	Serial.readBytes = readBytes;
	Serial.print = print;
	Serial.printi = printi;
	Serial.begin = begin;
	Serial.setTimeout = setTimeout;
}
