#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Servo;
struct Servo {
	struct Servo *self;
	int pin;
	int value;

	uint8_t (*attach)(struct Servo *servo, int pin); // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
//  uint8_t attach(int pin, int min, int max); // as above but also sets min and max values for writes.
	void (*write)(struct Servo *servo, int value); // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
};

typedef struct Servo Servo;

Servo servo(struct Servo *servo);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SERVO_H_ */
