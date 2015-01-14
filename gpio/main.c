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

#include <error.h>

/* documentation used is A20 user manual V1.2 20131210.pdf */

#define A20_PIO_BASE_ADDR 0x01C20800
/* base + last register offset + last register size */
#define A20_PIO_UPPER_ADDR (A20_PIO_BASE_ADDR + A20_REG_PIO_INT_DEB_OFF + 4)

#define A20_REG_PI_CFG1_OFF 0x124
#define A20_REG_PI_DAT_OFF 0x130

#define A20_REG_PIO_INT_DEB_OFF 0x218

#define A20_GPIO_IN 0
#define A20_GPIO_OUT 1

/* TODO remove some unneeded globals */
static long page_size;

static void *map_base;

static long mapping_size;

/* absolute address to which corresponds the start of the /dev/mem mapping */
static long mapping_absolute_start;

/* where is mapped the first register of PIO */
static void *pio_start;

struct gpio {
	uint32_t cfg_reg_off;
	uint8_t upp_bit;
	uint8_t low_bit;
	uint8_t index;
};

const struct gpio gpio13 = {
	.cfg_reg_off = A20_REG_PI_CFG1_OFF,
	.upp_bit = 14,
	.low_bit = 12,
	.index = 11,
};

/* TODO make this function generic, write a value in a bit sequence */
static int gpio_pinMode(const struct gpio *g, int mode)
{
	volatile uint32_t reg_value;
	volatile uint32_t reg_new_value;
	void *reg_addr;

	if (g == NULL || (mode != A20_GPIO_IN && mode != A20_GPIO_OUT))
		return -EINVAL;

	reg_addr = ((char *)pio_start + g->cfg_reg_off);
	reg_value = *(volatile uint32_t*)reg_addr;

	printf("reg_value = 0x%x\n", reg_value);
	reg_new_value = reg_value & ~(7 << g->low_bit);
	reg_new_value |= mode << g->low_bit;
	*(volatile uint32_t*)reg_addr = reg_new_value;

	return 0;
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

	ret = gpio_pinMode(&gpio13, A20_GPIO_OUT);
	if (ret < 0)
		error(EXIT_FAILURE, -ret, "gpio_pinMode");

	return EXIT_SUCCESS;
}
