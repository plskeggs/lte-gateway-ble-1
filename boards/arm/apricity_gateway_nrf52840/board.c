/*
 * Copyright (c) 2020 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_control, CONFIG_LOG_OVERRIDE_LEVEL);

/* Spare Pins
 *
 * | nRF52840 |
 * | P0.06   |
 * | P0.05   |
 * | P0.26   |
 * | P0.27   |
 * | P0.28   |
 * | P0.30   |
 * | P0.03   |
 * | P1.11   |
 */

__packed struct pin_config {
	uint8_t pin;
	uint8_t val;
};

//#define RESET_PORT CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET_PORT
//#define RESET_PIN CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET_PIN
//#define BOOT_SELECT_PORT CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_BOOT_SELECT_PORT
//#define BOOT_SELECT_PIN CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_BOOT_SELECT_PIN

#define RESET_NODE DT_NODELABEL(nrf52840_reset)
#define BOOT_NODE DT_NODELABEL(nrf52840_boot)

#if DT_NODE_HAS_STATUS(RESET_NODE, okay) && DT_NODE_HAS_STATUS(BOOT_NODE, okay)

#define RESET_GPIO_CTRL  DT_GPIO_CTLR(RESET_NODE, gpios)
#define RESET_GPIO_PIN   DT_GPIO_PIN(RESET_NODE, gpios)
#define RESET_GPIO_FLAGS DT_GPIO_FLAGS(RESET_NODE, gpios)

#define BOOT_GPIO_CTRL  DT_GPIO_CTLR(BOOT_NODE, gpios)
#define BOOT_GPIO_PIN   DT_GPIO_PIN(BOOT_NODE, gpios)
#define BOOT_GPIO_FLAGS DT_GPIO_FLAGS(BOOT_NODE, gpios)

bool ignore_reset = true;

static void chip_reset(const struct device *gpio,
		       struct gpio_callback *cb, uint32_t pins)
{
	const uint32_t stamp = k_cycle_get_32();

	printk("GPIO reset line asserted, device reset.\n");
	printk("Bye @ cycle32 %u\n", stamp);

	if (!ignore_reset) {
		NVIC_SystemReset();
	}
}

static void reset_pin_wait_low(const struct device *port, uint32_t pin)
{
	int val;

	/* Wait until the pin is pulled low */
	do {
		val = gpio_pin_get_raw(port, pin);
	} while (val > 0);
}

static int reset_pin_configure(const struct device *port, uint32_t pin)
{
	int err;

	static struct gpio_callback gpio_ctx;

	__ASSERT_NO_MSG(port != NULL);

	err = gpio_pin_configure(port, pin, GPIO_INPUT | GPIO_PULL_DOWN);
	if (err) {
		return err;
	}

	gpio_init_callback(&gpio_ctx, chip_reset, BIT(pin));

	err = gpio_add_callback(port, &gpio_ctx);
	if (err) {
		return err;
	}

	err = gpio_pin_interrupt_configure(port, pin, GPIO_INT_EDGE_RISING);
	if (err) {
		return err;
	}

	/* Wait until the pin is pulled low before continuing.
	 * This lets the other side ensure that they are ready.
	 */
	LOG_INF("GPIO reset line enabled");

	reset_pin_wait_low(port, pin);

	return 0;
}

static int init(void)
{
	/* Make sure to configure the switches before initializing
	 * the GPIO reset pin, so that we are connected to
	 * the nRF9160 before enabling our interrupt.
	 */
	if (IS_ENABLED(CONFIG_BOARD_APRICITY_GATEWAY_NRF52840_RESET)) {
		int rc;
		const struct device *reset_port = DEVICE_DT_GET(RESET_GPIO_CTRL);
		const struct device *boot_port = DEVICE_DT_GET(BOOT_BPIO_CTRL);
		const char *name;

		LOG_INF("Enabling GPIO reset line..",

		rc = reset_pin_configure(reset_port, RESET_GPIO_PIN);
		if (rc) {
			LOG_ERR("Unable to configure reset pin, err %d", rc);
			return -EIO;
		}

		/* let nrf9160 know we are running, by setting shared
		 * boot pin low briefly
		 */
		rc = gpio_pin_configure(boot_port, BOOT_GPIO_PIN, GPIO_OUTPUT |
					GPIO_OPEN_DRAIN | GPIO_PULL_UP);
		if (!rc) {
			LOG_ERR("Unable to configure boot pin, err %d", rc);
			return -EIO;
		}

		rc = gpio_pin_set(boot_port, BOOT_GPIO_PIN, 0);
		if (!rc) {
			LOG_ERR("Unable to set boot pin = 0, err %d", rc);
			return -EIO;
		}

		k_sleep(K_MSEC(200));

		/* let it get pulled up again, so user can request mcuboot
		 * later on if needed
		 */
		rc = gpio_pin_configure(boot_port, BOOT_GPIO_PIN,
					GPIO_INPUT | GPIO_PULL_UP);
	}

	LOG_INF("Board configured.");

	return 0;
}

SYS_INIT(init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

#else
#warning "Reset and/or boot node is missing"
#endif /* DT_NODE_HAS_STATUS(RESET_NODE, okay) && DT_NODE_HAS_STATUS(BOOT_NODE, okay) */
