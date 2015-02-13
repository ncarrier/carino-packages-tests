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

#include <pthread.h>

#define container_of(ptr, type, member) ({ \
	const typeof( ((type *)NULL)->member ) *__mptr = (ptr); \
	(type *)( (char *)__mptr - offsetof(type,member) );})

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


#define A20_PWM_BASE_ADDR 0x01C20C00
#define A20_PWM_LAST_REG_OFF A20_REG_PWM_CH1_PERIOD_OFF
#define A20_PWM_UPPER_ADDR (A20_PWM_BASE_ADDR + A20_PWM_LAST_REG_OFF + \
		A20_REG_SIZE)

#define A20_REG_PWM_CTRL_OFF 0x200
#define A20_REG_PWM_CH0_PERIOD_OFF 0x204
#define A20_REG_PWM_CH1_PERIOD_OFF 0x208

#define A20_LOW_BIT_PWM_ENTIRE_CYS 16
#define A20_LOW_BIT_PWM_ACTIVE_CYS 0

// by chance, the values correspond between config reg and arduino lib
#define A20_GPIO_IN INPUT
#define A20_GPIO_OUT OUTPUT

#define A20_GPIO_PWM 2

enum mapping_name {
	MAPPING_PIO,
	MAPPING_TP,
	MAPPING_PWM,
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
	[MAPPING_PWM] = {
		.base_offset = A20_PWM_BASE_ADDR,
		.upper_addr = A20_PWM_UPPER_ADDR,
		0,
	},
	[MAPPING_LRADC] = {
		.base_offset = A20_LRADC_BASE_ADDR,
		.upper_addr = A20_LRADC_UPPER_ADDR,
		0,
	},
};

struct simulated_pwm {
	pthread_t thread;
	sig_atomic_t value; /* between 0 and 255 */
};

struct real_pwm {
	/* bits for control register */
	uint8_t rdy;
	uint8_t bypass;
	uint8_t pulse_out_start;
	uint8_t mode;
	uint8_t clk_gating;
	uint8_t act_state;
	uint8_t en;
	uint8_t prescal; /* bits [prescal, prescal + 3] */

	uint32_t period_register;
};

struct pin;

struct pwm {
	/* operations on pin */
	void (* analogWrite)(struct pin *pin, int value);
	union {
		struct simulated_pwm simulated;
		struct real_pwm real;
	};
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

	struct pwm pwm;
};

static bool register_bit_range_value_base_prm_valid(void *reg_addr,
		enum mapping_name name,
		uint8_t low_bit, uint8_t bit_span)
{
	/* check reg_addr is in range */
	if (reg_addr < map[name].start
			|| (char *) reg_addr >= (char *)map[name].start
					+ map[name].upper_addr)
		return false;
	/* check bounds are strictly ordered and less than 32 */
	if (bit_span == 0 || low_bit + bit_span > 32)
		return false;

	return true;
}

static int get_register_bit_range_value(void *reg_addr, enum mapping_name name,
		uint8_t low_bit, uint8_t bit_span, uint32_t *value)
{
	uint32_t bit_mask;

	if (value == NULL)
		return -EINVAL;
	if (!register_bit_range_value_base_prm_valid(reg_addr, name, low_bit,
			bit_span))
		return -EINVAL;

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

/* low_bit is inclusive, bit_span is the total number of bits in [[1, 32]] */
static int set_register_bit_range_value(void *reg_addr, enum mapping_name name,
		uint8_t low_bit, uint8_t bit_span, uint32_t value)
{
	uint32_t reg_value;
	uint32_t bit_mask;

	if (!register_bit_range_value_base_prm_valid(reg_addr, name, low_bit,
			bit_span))
		return -EINVAL;

	bit_mask = (1 << bit_span) - 1;
	/* check value doesn't have more bits than fit in the bit span */
	if ((value & bit_mask) != value) {
		fprintf(stderr, "value 0x%"PRIu32" doesn't fit in [%d:%d]\n",
				value, low_bit, low_bit + bit_span - 1);
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

static bool is_pwm_pin(const struct pin *pin)
{
	return pin->pwm.analogWrite != NULL;
}

static bool is_simulated_pwm_pin(const struct pin *pin);

static void simulated_pwm_analogWrite(struct pin *pin, int value)
{
	if (is_simulated_pwm_pin(pin))
		pin->pwm.simulated.value = value;
}

static bool is_simulated_pwm_pin(const struct pin *pin)
{
	return pin->pwm.analogWrite == simulated_pwm_analogWrite;
}

static void real_pwm_analogWrite(struct pin *pin, int value)
{
	void *reg_addr;
	struct real_pwm *pwm = &pin->pwm.real;
	uint32_t busy;

	reg_addr = (char *)map[MAPPING_PWM].start + A20_REG_PWM_CTRL_OFF;
	do {
		get_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->rdy, 1,
				&busy);
		if (busy)
			fprintf(stderr, "busy\n");
	} while (busy);

	reg_addr = (char *)map[MAPPING_PWM].start + pwm->period_register;
	/* set number of the active cycles to 0 */
	set_register_bit_range_value(reg_addr, MAPPING_PWM,
			A20_LOW_BIT_PWM_ACTIVE_CYS, 0x10, value);
}

static bool is_real_pwm_pin(const struct pin *pin)
{
	return pin->pwm.analogWrite == real_pwm_analogWrite;
}

struct pin pins[] = {
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

		.pwm = {
				.analogWrite = simulated_pwm_analogWrite,
		},
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

		.pwm = {
				.analogWrite = real_pwm_analogWrite,
				.real = {
						.rdy = 28,
						.bypass = 9,
						.pulse_out_start = 8,
						.mode = 7,
						.clk_gating = 6,
						.act_state = 5,
						.en = 4,
						.prescal = 0,

						.period_register =
						A20_REG_PWM_CH0_PERIOD_OFF,
				},
		},
	},
	[6] = { /* port PI3 */
		.idx = 6,

		.cfg_reg_off = A20_REG_PI_CFG0_OFF,
		.low_bit = 12,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 3,

		.pwm = {
				.analogWrite = real_pwm_analogWrite,
				.real = {
						.rdy = 29,
						.bypass = 24,
						.pulse_out_start = 23,
						.mode = 22,
						.clk_gating = 21,
						.act_state = 20,
						.en = 19,
						.prescal = 15,

						.period_register =
						A20_REG_PWM_CH1_PERIOD_OFF,
				},
		},
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

		.pwm = {
				.analogWrite = simulated_pwm_analogWrite,
		},
	},
	[10] = { /* port PI10 */
		.idx = 10,

		.cfg_reg_off = A20_REG_PI_CFG1_OFF,
		.low_bit = 8,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 10,

		.pwm = {
				.analogWrite = simulated_pwm_analogWrite,
		},
	},
	[11] = { /* port PI12 */
		.idx = 11,

		.cfg_reg_off = A20_REG_PI_CFG1_OFF,
		.low_bit = 16,

		.dat_reg_off = A20_REG_PI_DAT_OFF,
		.dat_bit = 12,

		.pwm = {
				.analogWrite = simulated_pwm_analogWrite,
		},
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

	/*
	 * I absolutely don't understand how these two can function as GPIOs,
	 * so for now, the values set are placeholders TODO
	 */
	[A0] = { /* port LRADC0 */
		.idx = -1,

		.cfg_reg_off = -1,
		.low_bit = -1,

		.dat_reg_off = -1,
		.dat_bit = -1,
	},
	[A1] = { /* port LRADC1 */
		.idx = -1,

		.cfg_reg_off = -1,
		.low_bit = -1,

		.dat_reg_off = -1,
		.dat_bit = -1,
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
	if (pin >= A2 && pin <= A5)
		return MAPPING_TP;

	return MAPPING_LRADC;
}

static int pin_pinMode(const struct pin *pin, uint32_t mode)
{
	void *reg_addr;
	int ret;
	uint32_t value;
	enum mapping_name name;

	if (pin == NULL || (mode != A20_GPIO_IN && mode != A20_GPIO_OUT))
		return -EINVAL;
	if (mode == A20_GPIO_OUT && is_real_pwm_pin(pin))
		mode = A20_GPIO_PWM;
	name = name_from_pin(pin->idx);

	reg_addr = ((char *)map[name].start + pin->cfg_reg_off);

	ret = get_register_bit_range_value(reg_addr, name, pin->low_bit,
			3, &value);
	if (ret < 0) {
		fprintf(stderr, "failed to read value of register 0x%x\n",
				pin->cfg_reg_off);
		return ret;
	}

	if (value == mode)
		return 0;

	return set_register_bit_range_value(reg_addr, name, pin->low_bit, 3,
			mode);
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

	return set_register_bit_range_value(reg_addr, name, pin->dat_bit, 1,
			value);
}

static int pin_digitalRead(const struct pin *pin, uint32_t *value)
{
	void *reg_addr;
	enum mapping_name name;

	if (pin == NULL || value == NULL)
		return -EINVAL;
	name = name_from_pin(pin->idx);

	reg_addr = ((char *)map[name].start + pin->dat_reg_off);

	return get_register_bit_range_value(reg_addr, name, pin->dat_bit, 1,
			value);
}

static void __attribute__ ((destructor)) libarduino_clean(void)
{
	enum mapping_name n;

	for (n = MAPPING_FIRST; n <= MAPPING_LAST; n++)
		munmap(map[n].base, map[n].size);
}

static void init_mapping(enum mapping_name n, int fd)
{
	long page_size_mask;
	long min_size;
	long page_size;
	/* absolute address to which corresponds the /dev/mem mapping start */
	long mapping_absolute_start;

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
}

/*
 * frequency for pwm should be ~ 490 Hz (near to that of a real arduino)
 * in microseconds : 1000000 / 490 = 2041
 * each period is divided in 256 slices of ~8us
 * which gives a frequency of 1000000 / (256 * 8) =~ 488Hz
 */

/* in us */
#define PWM_SLICE 8

static void *simulated_pwm_update_routine(void *data)
{
	struct pin *pin = data;
	int value, complement;

	while (true) {
		value = pin->pwm.simulated.value;
		complement = 255 - value;
		if (value != 0) {
			pin_digitalWrite(pin, HIGH);
			usleep(PWM_SLICE * value);
		}
		if (complement != 0) {
			pin_digitalWrite(pin, LOW);
			usleep(PWM_SLICE * complement);
		}
	}

	return NULL;
}

static void init_simulated_pwm(struct pin *pin)
{
	pthread_create(&pin->pwm.simulated.thread, NULL,
			simulated_pwm_update_routine, pin);
}

static void init_real_pwm(struct pin *pin)
{
	void *reg_addr;
	struct real_pwm *pwm = &pin->pwm.real;

	reg_addr = (char *)map[MAPPING_PWM].start + A20_REG_PWM_CTRL_OFF;
	/* don't bypass the pwm */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->bypass, 1, 0);
	/* don't send a pulse */
	set_register_bit_range_value(reg_addr, MAPPING_PWM,
			pwm->pulse_out_start, 1, 0);
	/* put in cycle mode (not pulse mode) */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->mode, 1, 0);
	/* set clock divider to / 1 */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->prescal, 4,
			0xF);
	/* disable the clock gating */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->clk_gating, 1,
			1);
	/* set the active state to high */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->act_state, 1,
			1);
	/* enable the pwm */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->en, 1, 1);

	/*
	 * define the duty cycle, divide in 256 slices, so that the number of
	 * active cycles is the value passed to analogWrite
	 */
	reg_addr = (char *)map[MAPPING_PWM].start + pwm->period_register;
	/* set number of the entire cycles to 255 (0xFE + 1) */
	set_register_bit_range_value(reg_addr, MAPPING_PWM,
			A20_LOW_BIT_PWM_ENTIRE_CYS, 0x10, 0xFE);
	/* set number of the active cycles to 0 */
	set_register_bit_range_value(reg_addr, MAPPING_PWM,
			A20_LOW_BIT_PWM_ACTIVE_CYS, 0x10, 0);
}

void init_servo(uint8_t p)
{
	void *reg_addr;
	struct pin *pin = pins + p;
	struct real_pwm *pwm = &pin->pwm.real;

	reg_addr = (char *)map[MAPPING_PWM].start + A20_REG_PWM_CTRL_OFF;
	/* don't bypass the pwm */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->bypass, 1, 0);
	/* don't send a pulse */
	set_register_bit_range_value(reg_addr, MAPPING_PWM,
			pwm->pulse_out_start, 1, 0);
	/* put in cycle mode (not pulse mode) */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->mode, 1, 0);
	/* set clock divider to / 120 */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->prescal, 4,
			0x0);
	/* disable the clock gating */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->clk_gating, 1,
			1);
	/* set the active state to high */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->act_state, 1,
			1);
	/* enable the pwm */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->en, 1, 1);

	/*
	 * at 24MHz / 120, 4000 is 20ms, 1 ms is 200 and 2ms is 400
	 */
	reg_addr = (char *)map[MAPPING_PWM].start + pwm->period_register;
	/* set number of the entire cycles to 4000 (3999 + 1) */
	set_register_bit_range_value(reg_addr, MAPPING_PWM,
			A20_LOW_BIT_PWM_ENTIRE_CYS, 0x10, 3999);
	/* set number of the active cycles to 0 */
	set_register_bit_range_value(reg_addr, MAPPING_PWM,
			A20_LOW_BIT_PWM_ACTIVE_CYS, 0x10, 0);
}

static void init_simulated_pwms()
{
	int i;
	struct pin *pin = pins;

	for (i = 0; i <= A5; i++, pin++)
		if (is_simulated_pwm_pin(pin))
			init_simulated_pwm(pin);
}

static void init_real_pwms()
{
	int i;
	struct pin *pin = pins;

	for (i = 0; i <= A5; i++, pin++)
		if (is_real_pwm_pin(pin))
			init_real_pwm(pin);
}

static void init_mappings()
{
	int fd;
	enum mapping_name n;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1)
		error(EXIT_FAILURE, errno, "open");

	/* mmap dev/mem for the register ranges we wan't to gain access to */
	for (n = MAPPING_FIRST; n <= MAPPING_LAST; n++)
		init_mapping(n, fd);

	close(fd);
}

static void init_gpios(void)
{
	/* TODO init the gpios to a known state, e.g. OUTPUT / LOW */
}

static void init_pwms(void)
{
	init_simulated_pwms();
	init_real_pwms();
}

static void __attribute__ ((constructor)) libarduino_init(void)
{
	init_mappings();
	init_gpios();
	init_pwms();
}

void pinMode(uint8_t pin, uint8_t mode)
{
	const struct pin *p;

	if (pin > A5 || (mode != A20_GPIO_IN && mode != A20_GPIO_OUT))
		return;

	p = pins + pin;
	pin_pinMode(p, mode);
}

void analogWrite(uint8_t pin, int value)
{
	struct pin *ppin;

	if (pin > A5)
		return;
	ppin = pins + pin;
	if (ppin->pwm.analogWrite == NULL)
		return;
	value = constrain(value, 0, 255);

	ppin->pwm.analogWrite(ppin, value);
}

void digitalWrite(uint8_t pin, uint8_t value)
{
	struct pin *ppin;

	if (pin > A5)
		return;

	ppin = pins + pin;

	if (is_pwm_pin(ppin))
		ppin->pwm.analogWrite(ppin, value ? 255 : 0);
	else
		pin_digitalWrite(ppin, !!value);
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

int analogRead(__attribute__((unused)) uint8_t pin)
{
	// TODO

	return 0;
}

void setServoValue(uint8_t p, int value)
{
	void *reg_addr;
	struct pin *pin = pins + p;
	struct real_pwm *pwm = &pin->pwm.real;
	uint32_t busy;

	reg_addr = (char *)map[MAPPING_PWM].start + A20_REG_PWM_CTRL_OFF;
	/* disable the pwm */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->en, 1, 0);

	reg_addr = (char *)map[MAPPING_PWM].start + A20_REG_PWM_CTRL_OFF;
	do {
		get_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->rdy, 1,
				&busy);
		if (busy)
			fprintf(stderr, "busy\n");
	} while (busy);

	reg_addr = (char *)map[MAPPING_PWM].start + pwm->period_register;
	/* set number of the active cycles to 0 */
	set_register_bit_range_value(reg_addr, MAPPING_PWM,
			A20_LOW_BIT_PWM_ACTIVE_CYS, 0x10, 120 + 22 * value / 10);

	reg_addr = (char *)map[MAPPING_PWM].start + A20_REG_PWM_CTRL_OFF;
	/* enable the pwm */
	set_register_bit_range_value(reg_addr, MAPPING_PWM, pwm->en, 1, 1);
}

void tone(__attribute__((unused)) uint8_t pin,
		__attribute__((unused)) unsigned int frequency,
		__attribute__((unused)) unsigned long duration)
{
	// TODO
	printf("STUB %s: pin %"PRIu32", frequency %d, duration %lu\n", __func__,
			pin, frequency, duration);
}
