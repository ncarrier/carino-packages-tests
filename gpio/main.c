#include <stdio.h>
#include <stdlib.h>

#include <Arduino.h>

static void usage(int status)
{
	printf("tests_gpio PIN\n\twith PIN being in [0,13]U[A0,A5]\n"
			"\tMakes the selected 'blink' at 0.5Hz\n");

	exit(status);
}

int main(int argc, char *argv[])
{
	int pin = 13;

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

	/* let it blink, let it blink, let it blink, oh let it blink ! */
	printf("value: %d\n", digitalRead(pin));
	digitalWrite(pin, HIGH);
	printf("value: %d\n", digitalRead(pin));
	sleep(1);
	digitalWrite(pin, LOW);
	printf("value: %d\n", digitalRead(pin));
	sleep(1);
	digitalWrite(pin, HIGH);
	printf("value: %d\n", digitalRead(pin));
	sleep(1);
	digitalWrite(pin, LOW);
	printf("value: %d\n", digitalRead(pin));
	sleep(1);
	digitalWrite(pin, HIGH);
	printf("value: %d\n", digitalRead(pin));
	sleep(1);
	digitalWrite(pin, LOW);
	printf("value: %d\n", digitalRead(pin));
	sleep(1);

	return EXIT_SUCCESS;
}
