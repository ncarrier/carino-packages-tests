#ifndef Serial_h
#define Serial_h

#include <stdint.h>
#include <inttypes.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Define config for Serial.begin(baud, config);
#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

#define SERIAL_BUFFER_SIZE 1024

struct HwSerial
{
	int timeout; // TODO check type

	int (*readBytes)(void *buffer, size_t len);
	int (*print)(const char *s);
	int (*printi)(int i);
	void (*begin)(unsigned long, uint8_t config);
	void (*setTimeout)(int timeout);
};

extern void serialEventRun(void) __attribute__((weak));
extern struct HwSerial Serial;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
