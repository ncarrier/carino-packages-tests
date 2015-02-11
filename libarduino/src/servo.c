/**
 * @file servo.c
 * @brief 
 *
 * @date Feb 5, 2015
 * @author carrier.nicolas0@gmail.com
 */
#include <stdlib.h>
#include <stdio.h>

#include "Servo.h"
#include "arduino_private.h"

static uint8_t attach(struct Servo *servo, int pin)
{
	if (servo == NULL)
		return 0;

	servo->pin = pin;
	init_servo(pin);

	return 0; /* TODO should return channel number, what is it ? */
}

static void write(struct Servo *servo, int value)
{
	if (servo == NULL)
		return;

	setServoValue(servo->pin, value);
}

Servo servo(struct Servo *servo)
{
	if (servo == NULL)
		return (struct Servo){.self = NULL, .pin = -1, .value = -1,
				.attach = attach,.write = write,};

	*servo = (struct Servo) {.self = servo, .pin = -1, .value = -1,
		.attach = attach, .write = write,};

	return *servo;
}

