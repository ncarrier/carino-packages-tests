#include <unistd.h>

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdbool.h>

#include <Servo.h>

static void usage(int status)
{
	printf("tests_servo PIN angle\n\twith GPIO being in \n"
			"{3, 5, 6}U[[9, 11]]\n"
			"\twith 5 and 6, based on real pwms and the others "
			"are simulated with gpios.\n"
			"\tmake a servo controlled by pin PIN go to a given "
			"angle in [0°, 180°]\n");

	exit(status);
}

int main(int argc, char *argv[])
{
	int angle;
	int pin = 3;
	Servo s;

	if (argc > 1) {
		if (argv[1][0] == 'A') {
			if (argv[1][1] == '\0')
				usage(EXIT_FAILURE);

			argv[1][2] = '\0';
			pin = atoi(argv[1] + 1) + 14;
		} else {
			pin = atoi(argv[1]);
		}
		if (argc > 2)
			angle = atoi(argv[2]);
	}

	servo(&s);
	s.attach(&s, pin);

	printf("set servo on pin %d to angle %d\n", pin, angle);

	s.write(&s, angle);

	usleep(1000);

	return EXIT_SUCCESS;
}
