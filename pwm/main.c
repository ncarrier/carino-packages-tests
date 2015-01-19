#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <errno.h>
#include <stdlib.h>
#include <stdbool.h>

#include <error.h>

#include <Arduino.h>

static void usage(int status)
{
	printf("tests_pwm GPIO\n\twith GPIO being in [0,13]U[A0,A5]\n");

	exit(status);
}

#define PERIOD 20000

int main(int argc, char *argv[])
{
	int i = 100;
	int pin = 13;
	int t;

	/* TODO handle the A0, A1 ... cases */
	if (argc > 1) {
		if (argv[1][0] == 'A') {
			if (argv[1][1] == '\0')
				usage(EXIT_FAILURE);

			argv[1][2] = '\0';
			pin = atoi(argv[1] + 1) + 14;
		} else {
			pin = atoi(argv[1]);
		}
	}

	printf("working with pin %d\n", pin);

	printf("period is %f\n", PERIOD / 1000000000.);

	pinMode(pin, OUTPUT);

	/*
	0
	0.83
	1.65
	2.48
	*/
	t = 0;
	printf("duty cycle is %f\n", t / 100.);
	while (i--) {
		digitalWrite(pin, HIGH);
		usleep(t * PERIOD / 100);
		digitalWrite(pin, LOW);
		usleep((100 - t) * PERIOD / 100);
	}
	i = 100;
	t = 25;
	printf("duty cycle is %f\n", t / 100.);
	while (i--) {
		digitalWrite(pin, HIGH);
		usleep(t * PERIOD / 100);
		digitalWrite(pin, LOW);
		usleep((100 - t) * PERIOD / 100);
	}
	i = 100;
	t = 50;
	printf("duty cycle is %f\n", t / 100.);
	while (i--) {
		digitalWrite(pin, HIGH);
		usleep(t * PERIOD / 100);
		digitalWrite(pin, LOW);
		usleep((100 - t) * PERIOD / 100);
	}
	i = 100;
	t = 75;
	printf("duty cycle is %f\n", t / 100.);
	while (i--) {
		digitalWrite(pin, HIGH);
		usleep(t * PERIOD / 100);
		digitalWrite(pin, LOW);
		usleep((100 - t) * PERIOD / 100);
	}
	i = 100;
	t = 100;
	printf("duty cycle is %f\n", t / 100.);
	while (i--) {
		digitalWrite(pin, HIGH);
		usleep(t * PERIOD / 100);
		digitalWrite(pin, LOW);
		usleep((100 - t) * PERIOD / 100);
	}

	digitalWrite(pin, HIGH);

	return EXIT_SUCCESS;
}
