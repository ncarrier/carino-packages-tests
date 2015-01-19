#include <unistd.h>

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>

#include <Arduino.h>

static void usage(int status)
{
	printf("tests_pwm GPIO\n\twith GPIO being in [0,13]U[A0,A5]\n");

	exit(status);
}

int main(int argc, char *argv[])
{
	int i;
	int pin = 3;

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

	pinMode(pin, OUTPUT);

	for (i = 0; i <= 255; i++) {
		analogWrite(pin, i);
		usleep(3906);
	}
	for (i = 255; i >= 0; i--) {
		analogWrite(pin, i);
		usleep(3906);
	}
	for (i = 0; i <= 255; i++) {
		analogWrite(pin, i);
		usleep(3906);
	}
	for (i = 255; i >= 0; i--) {
		analogWrite(pin, i);
		usleep(3906);
	}

	return EXIT_SUCCESS;
}
