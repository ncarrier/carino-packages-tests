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

/* documentation used is A20 user manual V1.2 20131210.pdf */
#define A20_REG_SIZE sizeof(uint32_t)

#define A20_PIO_BASE_ADDR 0x01C20800
#define A20_PIO_LAST_REG_OFF A20_REG_PIO_INT_DEB_OFF

/* base + last register offset + last register size */
#define A20_PIO_UPPER_ADDR (A20_PIO_BASE_ADDR + A20_PIO_LAST_REG_OFF + \
		A20_REG_SIZE)

#define A20_REG_PB_CFG0_OFF 0x24
#define A20_REG_PB_DAT_OFF 0x34

#define A20_REG_PH_CFG0_OFF 0xFC
#define A20_REG_PH_CFG1_OFF 0x100
#define A20_REG_PH_DAT_OFF 0x10C

#define A20_REG_PI_CFG0_OFF 0x120
#define A20_REG_PI_CFG1_OFF 0x124
#define A20_REG_PI_CFG2_OFF 0x128
#define A20_REG_PI_DAT_OFF 0x130

#define A20_REG_PIO_INT_DEB_OFF 0x218


#define A20_TP_BASE_ADDR 0x01C25000
#define A20_TP_LAST_REG_OFF A20_REG_TP_PORT_DATA_OFF
#define A20_TP_UPPER_ADDR (A20_TP_BASE_ADDR + A20_TP_LAST_REG_OFF + \
		A20_REG_SIZE)

#define A20_REG_TP_IO_CONFIG_OFF 0x28
#define A20_REG_TP_PORT_DATA_OFF 0x2c


#define A20_LRADC_BASE_ADDR 0x01C22800
#define A20_LRADC_LAST_REG_OFF A20_REG_LRADC_DATA_1_OFF
#define A20_LRADC_UPPER_ADDR (A20_LRADC_BASE_ADDR + A20_LRADC_LAST_REG_OFF + \
		A20_REG_SIZE)

#define A20_REG_LRADC_CTRL_OFF 0x00
#define A20_REG_LRADC_DATA_0_OFF 0xC
#define A20_REG_LRADC_DATA_1_OFF 0x10


#define A20_GPIO_IN 0
#define A20_GPIO_OUT 1

#define LOW 0
#define HIGH 1

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

enum mapping_name {
	MAPPING_PIO,
	MAPPING_TP,
	MAPPING_LRADC,

	MAPPING_FIRST = MAPPING_PIO,
	MAPPING_LAST = MAPPING_LRADC,
};

struct mapping {
	/* input constant parameters */
	/** start offset of the port registers mapping */
	int base_offset;
	/** upper bound of the register addresses we're interested in */
	int upper_addr;

	/* fields initialized at init */
	/** address at which the mmapping of /dev/mem starts */
	void *base;
	/** where is mapped the first register of PIO */
	void *start;
	/** size mapped in memory, starting from base */
	long size;
};

static struct mapping map[] = {
	[MAPPING_PIO] = {
		.base_offset = A20_PIO_BASE_ADDR,
		.upper_addr = A20_PIO_UPPER_ADDR,
		0,
	},
	[MAPPING_TP] = {
		.base_offset = A20_TP_BASE_ADDR,
		.upper_addr = A20_TP_UPPER_ADDR,
		0,
	},
	[MAPPING_LRADC] = {
		.base_offset = A20_LRADC_BASE_ADDR,
		.upper_addr = A20_LRADC_UPPER_ADDR,
		0,
	},
};

struct pin {
	/** index in pins array */
	uint8_t idx;

	/* configuration informations */
	/** offset of the register set, used to select the mapping too */
	uint32_t cfg_reg_off;
	/** low control bit for setting the pin's function upp bit is low + 2 */
	uint8_t low_bit;

	/* data informations */
	/** offset of the register holding the GPIO data */
	uint32_t dat_reg_off;
	/** bit to alter in the dat_reg for setting / reading the value */
	uint8_t dat_bit; /* must be equal to x in PIx, PHx ... */
};

const struct pin pins[] = {
	[0] = { /* port PI19 */
		.idx = 0,

		.cfg_reg_off = A20_REG_PI_CFG2_OFF,
		.low_bit = 12,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 19,
	},
	[1] = { /* port PI18 */
		.idx = 1,

		.cfg_reg_off = A20_REG_PI_CFG2_OFF,
		.low_bit = 8,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 18,
	},
	[2] = { /* port PH7 */
		.idx = 2,

		.cfg_reg_off = A20_REG_PH_CFG0_OFF,
		.low_bit = 28,

		.dat_reg_off = A20_REG_PH_DAT_OFF,
		.dat_bit = 7,
	},
	[3] = { /* port PH6 */
		.idx = 3,

		.cfg_reg_off = A20_REG_PH_CFG0_OFF,
		.low_bit = 24,

		.dat_reg_off = A20_REG_PH_DAT_OFF,
		.dat_bit = 6,
	},
	[4] = { /* port PH8 */
		.idx = 4,

		.cfg_reg_off = A20_REG_PH_CFG1_OFF,
		.low_bit = 0,

		.dat_reg_off = A20_REG_PH_DAT_OFF,
		.dat_bit = 8,
	},
	[5] = { /* port PB2 */
		.idx = 5,

		.cfg_reg_off = A20_REG_PB_CFG0_OFF,
		.low_bit = 8,

		.dat_reg_off = A20_REG_PB_DAT_OFF,
		.dat_bit = 2,
	},
	[6] = { /* port PI3 */
		.idx = 6,

		.cfg_reg_off = A20_REG_PI_CFG0_OFF,
		.low_bit = 12,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 3,
	},
	[7] = { /* port PH9 */
		.idx = 7,

		.cfg_reg_off = A20_REG_PH_CFG1_OFF,
		.low_bit = 4,

		.dat_reg_off = A20_REG_PH_DAT_OFF,
		.dat_bit = 9,
	},

	[8] = { /* port PH10 */
		.idx = 8,

		.cfg_reg_off = A20_REG_PH_CFG1_OFF,
		.low_bit = 8,

		.dat_reg_off = A20_REG_PH_DAT_OFF,
		.dat_bit = 10,
	},
	[9] = { /* port PH5 */
		.idx = 9,

		.cfg_reg_off = A20_REG_PH_CFG0_OFF,
		.low_bit = 20,

		.dat_reg_off = A20_REG_PH_DAT_OFF,
		.dat_bit = 5,
	},
	[10] = { /* port PI10 */
		.idx = 10,

		.cfg_reg_off = A20_REG_PI_CFG1_OFF,
		.low_bit = 8,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 10,
	},
	[11] = { /* port PI12 */
		.idx = 11,

		.cfg_reg_off = A20_REG_PI_CFG1_OFF,
		.low_bit = 16,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 12,
	},
	[12] = { /* port PI13 */
		.idx = 12,

		.cfg_reg_off = A20_REG_PI_CFG1_OFF,
		.low_bit = 20,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 13,
	},
	[13] = { /* port PI11 */
		.idx = 13,

		.cfg_reg_off = A20_REG_PI_CFG1_OFF,
		.low_bit = 12,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 11,
	},

	/* I absolutely don't understand how these two can function as GPIOs */
	[A0] = { /* port LRADC0 */
/*		.idx = A0,*/

/*		.cfg_reg_off = A20_REG_TP_IO_CONFIG_OFF,*/
/*		.low_bit = 12,*/

/*		.dat_reg_off = A20_REG_TP_PORT_DATA_OFF,*/
/*		.dat_bit = 3,*/
	},
	[A1] = { /* port LRADC1 */
/*		.idx = A1,*/

/*		.cfg_reg_off = A20_REG_TP_IO_CONFIG_OFF,*/
/*		.low_bit = 12,*/

/*		.dat_reg_off = A20_REG_TP_PORT_DATA_OFF,*/
/*		.dat_bit = 3,*/
	},

	[A2] = { /* port TP_XP / XP_TP */
		.idx = A2,

		.cfg_reg_off = A20_REG_TP_IO_CONFIG_OFF,
		.low_bit = 0,

		.dat_reg_off = A20_REG_TP_PORT_DATA_OFF,
		.dat_bit = 0,
	},
	[A3] = { /* port TP_XN / XN_TP */
		.idx = A3,

		.cfg_reg_off = A20_REG_TP_IO_CONFIG_OFF,
		.low_bit = 4,

		.dat_reg_off = A20_REG_TP_PORT_DATA_OFF,
		.dat_bit = 1,
	},
	[A4] = { /* port TP_YP / YP_TP */
		.idx = A4,

		.cfg_reg_off = A20_REG_TP_IO_CONFIG_OFF,
		.low_bit = 8,

		.dat_reg_off = A20_REG_TP_PORT_DATA_OFF,
		.dat_bit = 2,
	},
	[A5] = { /* port TP_YN / YN_TP */
		.idx = A5,

		.cfg_reg_off = A20_REG_TP_IO_CONFIG_OFF,
		.low_bit = 12,

		.dat_reg_off = A20_REG_TP_PORT_DATA_OFF,
		.dat_bit = 3,
	},
};

static enum mapping_name name_from_pin(uint8_t pin)
{
	if (pin <= 13)
		return MAPPING_PIO;
	if (pin >= A0 && pin <= A3) // TODO check that with the order of pins
		return MAPPING_TP;

	return MAPPING_LRADC;
}

//static uint32_t base_from_pin(uint8_t pin)
//{
//	if (pin <= 13)
//		return A20_PIO_BASE_ADDR;
//	if (pin >= A0 && pin <= A3) // TODO check that with the order of pins
//		return A20_TP_BASE_ADDR;
//
//	return A20_LRADC_BASE_ADDR;
//}
//
// TODO here, add a enum mapping_name parameter
static bool register_bit_range_value_base_prm_valid(void *reg_addr,
		enum mapping_name name,
		uint8_t low_bit, uint8_t upp_bit)
{
	/* check reg_addr is in range */
	if (reg_addr < map[name].start ||
			reg_addr > map[name].start + A20_PIO_LAST_REG_OFF)
		return false;
	/* check bounds are strictly ordered and less than 32 */
	if (upp_bit < low_bit || upp_bit >= 32 || low_bit >= 32)
		return false;

	return true;
}

static int get_register_bit_range_value(void *reg_addr, enum mapping_name name,
		uint8_t low_bit, uint8_t upp_bit, uint32_t *value)
{
	uint8_t bit_span;
	uint32_t bit_mask;

	if (value == NULL)
		return -EINVAL;
	if (!register_bit_range_value_base_prm_valid(reg_addr, name, low_bit,
			upp_bit))
		return -EINVAL;

	bit_span = upp_bit - low_bit + 1;
	bit_mask = (1 << bit_span) - 1;

	/* shift mask to it's right bit offset */
	bit_mask <<= low_bit;

	/* read, compute and update */
	*value = *(volatile uint32_t*)reg_addr;
	/* shut off the bits in range */
	*value &= bit_mask;
	*value >>= low_bit;

	return 0;
}

/* low_bit and upp_bit are inclusives */
static int set_register_bit_range_value(void *reg_addr, enum mapping_name name,
		uint8_t low_bit, uint8_t upp_bit, uint32_t value)
{
	uint32_t reg_value;
	uint8_t bit_span;
	uint32_t bit_mask;

	if (!register_bit_range_value_base_prm_valid(reg_addr, name, low_bit,
			upp_bit))
		return -EINVAL;

	bit_span = upp_bit - low_bit + 1;
	bit_mask = (1 << bit_span) - 1;
	/* check value doesn't have more bits than fit in the bit span */
	if ((value & bit_mask) != value) {
		fprintf(stderr, "value 0x%"PRIu32" doesn't fit in [%d:%d]\n",
				value, low_bit, upp_bit);
		return -EINVAL;
	}

	/* shift value and mask to their right bit offset */
	bit_mask <<= low_bit;
	value <<= low_bit;

	/* read, compute and update */
	reg_value = *(volatile uint32_t*)reg_addr;
	/* shut off the bits in range */
	reg_value &= ~bit_mask;
	reg_value |= value;
	*(volatile uint32_t*)reg_addr = reg_value;

	return 0;
}

static int pin_pinMode(const struct pin *pin, uint32_t mode)
{
	void *reg_addr;
	int ret;
	uint32_t value;
	enum mapping_name name;

	if (pin == NULL || (mode != A20_GPIO_IN && mode != A20_GPIO_OUT))
		return -EINVAL;
	name = name_from_pin(pin->idx);

	reg_addr = ((char *)map[name].start + pin->cfg_reg_off);

	ret = get_register_bit_range_value(reg_addr, name, pin->low_bit,
			pin->low_bit + 2, &value);
	if (ret < 0) {
		fprintf(stderr, "failed to read value of register 0x%x\n",
				pin->cfg_reg_off);
		return ret;
	}

	if (value == mode)
		return 0;

	return set_register_bit_range_value(reg_addr, name, pin->low_bit,
			pin->low_bit + 2, mode);
}

static int pin_digitalWrite(const struct pin *pin, uint8_t value)
{
	void *reg_addr;
	enum mapping_name name;

	if (pin == NULL)
		return -EINVAL;
	value = !!value;
	name = name_from_pin(pin->idx);

	reg_addr = ((char *)map[name].start + pin->dat_reg_off);

	return set_register_bit_range_value(reg_addr, name, pin->dat_bit,
			pin->dat_bit, value);
}

static int pin_digitalRead(const struct pin *pin, uint32_t *value)
{
	void *reg_addr;
	enum mapping_name name;

	if (pin == NULL || value == NULL)
		return -EINVAL;
	name = name_from_pin(pin->idx);

	reg_addr = ((char *)map[name].start + pin->dat_reg_off);

	return get_register_bit_range_value(reg_addr, name, pin->dat_bit,
			pin->dat_bit, value);
}

int digitalRead(uint8_t pin)
{
	int ret;
	uint32_t value;

	if (pin > A5)
		return 0;

	ret = pin_digitalRead(pins + pin, &value);
	if (ret < 0)
		return 0;

	return value;
}

static void __attribute__ ((destructor)) clean(void)
{
	enum mapping_name n;

	for (n = MAPPING_FIRST; n <= MAPPING_LAST; n++)
		munmap(map[n].base, map[n].size);
}

static void init_mapping(enum mapping_name n)
{
	int fd;
	long page_size_mask;
	long min_size;
	long page_size;
	/* absolute address to which corresponds the /dev/mem mapping start */
	long mapping_absolute_start;

	/* TODO open / close dev/mem only once */
	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1)
		error(EXIT_FAILURE, errno, "open");

	page_size = sysconf(_SC_PAGESIZE);
	/* assumes page_size is a power of two */
	page_size_mask = page_size - 1;

	/* start mapping at a page boundary */
	mapping_absolute_start = map[n].base_offset & ~page_size_mask;

	/* size necessary to access all the PIO registers */
	min_size = map[n].upper_addr - mapping_absolute_start;

	/* size rounded to the above page_size multiple */
	map[n].size = min_size & ~page_size_mask;
	if (map[n].size != min_size)
		map[n].size += page_size;

	map[n].base = mmap(NULL, map[n].size, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd, mapping_absolute_start);
	if (map[n].base == MAP_FAILED)
		error(EXIT_FAILURE, errno, "mmap");
	map[n].start = (void *)((int)map[n].base +
			(map[n].base_offset & page_size_mask));

	close(fd);
}

static void __attribute__ ((constructor)) init(void)
{
	enum mapping_name n;

	for (n = MAPPING_FIRST; n <= MAPPING_LAST; n++)
		init_mapping(n);
}

static void pinMode(uint8_t pin, uint8_t mode)
{
	const struct pin *p;

	if (pin > A5 || (mode != A20_GPIO_IN && mode != A20_GPIO_OUT))
		return;

	p = pins + pin;
	pin_pinMode(p, mode);
}

static void digitalWrite(uint8_t pin, uint8_t value)
{
	if (pin > A5)
		return;

	pin_digitalWrite(pins + pin, !!value);
}

static void usage(int status)
{
	printf("tests_gpio GPIO\n\twith GPIO being in [0,13]U[A0,A5]\n");

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

	pinMode(pin, A20_GPIO_OUT);

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
