#include <stdio.h>
#include <stdlib.h>

#include <Arduino.h>

#define DEFAULT_PIN 13

static void usage(int status)
{
	printf("tests_gpio high|low [PIN]\n\twith PIN being in [0,13]U[A0,A5]\n"
			"\tSets the selected pin to the high or low pin\n"
			"\tIf not given, PIN defaults to %d\n", DEFAULT_PIN);

	exit(status);
}

int main(int argc, char *argv[])
{
	int pin = DEFAULT_PIN;
	uint8_t value;
	const char *strvalue;
	char *strpin;

	if (argc < 2 || argc > 3)
		usage(EXIT_FAILURE);
	strvalue = argv[1];

	if (strcasecmp("high", strvalue) == 0)
		value = HIGH;
	else if (strcasecmp("low", strvalue) == 0)
		value = LOW;
	else
		usage(EXIT_FAILURE);

	if (argc == 3) {
		strpin = argv[2];
		if (strpin[0] == 'A') {
			if (strpin[1] == '\0')
				usage(EXIT_FAILURE);

			strpin[1] = '\0';
			pin = atoi(strpin + 1) + 14;
		} else {
			pin = atoi(strpin);
		}
	}

	if (getenv("DEBUG") != NULL)
		printf("set pin %d to state %d\n", pin, value);

	return EXIT_SUCCESS;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, value);

	return EXIT_SUCCESS;
}
