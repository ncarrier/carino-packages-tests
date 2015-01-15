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
#define A20_PIO_REG_SIZE sizeof(uint32_t)

#define A20_PIO_BASE_ADDR 0x01C20800
#define A20_REG_PIO_LAST_OFF A20_REG_PIO_INT_DEB_OFF

/* base + last register offset + last register size */
#define A20_PIO_UPPER_ADDR (A20_PIO_BASE_ADDR + A20_REG_PIO_LAST_OFF + A20_PIO_REG_SIZE)

#define A20_REG_PI_CFG1_OFF 0x124
#define A20_REG_PI_DAT_OFF 0x130

#define A20_REG_PIO_INT_DEB_OFF 0x218

#define A20_GPIO_IN 0
#define A20_GPIO_OUT 1

#define LOW 0
#define HIGH 1

/* address at which the mmapping of /dev/mem starts */
static void *map_base;

static long mapping_size;

/* where is mapped the first register of PIO */
static void *pio_start;

struct pin {
	/* configuration informations */
	uint32_t cfg_reg_off;
	uint8_t upp_bit;
	uint8_t low_bit;
	uint8_t index;

	/* data informations */
	uint32_t dat_reg_off;
	uint8_t dat_bit;
};

const struct pin pin13 = {
	.cfg_reg_off = A20_REG_PI_CFG1_OFF,
	.upp_bit = 14,
	.low_bit = 12,
	.index = 11,

	.dat_reg_off = A20_REG_PI_DAT_OFF,
	.dat_bit = 11,
};

static bool register_bit_range_value_base_prm_valid(void *reg_addr,
		uint8_t low_bit, uint8_t upp_bit)
{
	/* check reg_addr is in range */
	if (reg_addr < pio_start || reg_addr > pio_start + A20_REG_PIO_LAST_OFF)
		return false;
	/* check bounds are strictly ordered and less than 32 */
	if (upp_bit <= low_bit || upp_bit >= 32 || low_bit >= 32)
		return false;

	return true;
}

static int get_register_bit_range_value(void *reg_addr, uint8_t low_bit,
		uint8_t upp_bit, uint32_t *value)
{
	uint8_t bit_span;
	uint32_t bit_mask;

	if (value == NULL)
		return -EINVAL;
	if (!register_bit_range_value_base_prm_valid(reg_addr, low_bit, upp_bit))
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
static int set_register_bit_range_value(void *reg_addr, uint8_t low_bit,
		uint8_t upp_bit, uint32_t value)
{
	uint32_t reg_value;
	uint8_t bit_span;
	uint32_t bit_mask;

	if (!register_bit_range_value_base_prm_valid(reg_addr, low_bit, upp_bit))
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

	if (pin == NULL || (mode != A20_GPIO_IN && mode != A20_GPIO_OUT))
		return -EINVAL;

	reg_addr = ((char *)pio_start + pin->cfg_reg_off);

	ret = get_register_bit_range_value(reg_addr, pin->low_bit, pin->upp_bit,
		&value);
	if (ret < 0) {
		fprintf(stderr, "failed to read value of register 0x%x\n",
				pin->cfg_reg_off);
		return ret;
	}

	if (value == mode)
		return 0;

	return set_register_bit_range_value(reg_addr, pin->low_bit,
			pin->upp_bit, mode);
}

static int pin_digitalWrite(const struct pin *pin, uint8_t value)
{
	void *reg_addr;

	if (pin == NULL)
		return -EINVAL;
	value = !!value;

	reg_addr = ((char *)pio_start + pin->dat_reg_off);

	return set_register_bit_range_value(reg_addr, pin->dat_bit,
			pin->dat_bit, value);
}

static void __attribute__ ((destructor)) clean(void)
{
	munmap(map_base, mapping_size);
}

static void __attribute__ ((constructor)) init(void)
{
	int fd;
	long page_size_mask;
	long min_size;
	long page_size;
	/* absolute address to which corresponds the /dev/mem mapping start */
	long mapping_absolute_start;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1)
		error(EXIT_FAILURE, errno, "open");

	page_size = sysconf(_SC_PAGESIZE);
	/* assumes page_size is a power of two */
	page_size_mask = page_size - 1;

	/* start mapping at a page boundary */
	mapping_absolute_start = A20_PIO_BASE_ADDR & ~page_size_mask;

	/* size necessary to access all the PIO registers */
	min_size = A20_PIO_UPPER_ADDR - mapping_absolute_start;

	/* size rounded to the above page_size multiple */
	mapping_size = min_size & ~page_size_mask;
	if (mapping_size != min_size)
		mapping_size += page_size;

	map_base = mmap(NULL, mapping_size, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, mapping_absolute_start);
	if (map_base == MAP_FAILED)
		error(EXIT_FAILURE, errno, "mmap");
	pio_start = (void *)((int)map_base +
			(A20_PIO_BASE_ADDR & page_size_mask));

	close(fd);
}

int main(int argc, char *argv[])
{
	int ret;

	ret = pin_pinMode(&pin13, A20_GPIO_OUT);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "pin_pinMode");

	/* let it blink, let it blink, let it blink, oh let it blink ! */
	ret = pin_digitalWrite(&pin13, HIGH);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "pin_digitalWrite");
	sleep(1);
	ret = pin_digitalWrite(&pin13, LOW);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "pin_digitalWrite");
	sleep(1);
	ret = pin_digitalWrite(&pin13, HIGH);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "pin_digitalWrite");
	sleep(1);
	ret = pin_digitalWrite(&pin13, LOW);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "pin_digitalWrite");
	sleep(1);
	ret = pin_digitalWrite(&pin13, HIGH);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "pin_digitalWrite");
	sleep(1);
	ret = pin_digitalWrite(&pin13, LOW);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "pin_digitalWrite");

	return EXIT_SUCCESS;
}
