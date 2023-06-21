/*
 * Copyright (c) 2023 Felix Baral-Weber
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_VCNL4035_VCNL4035_H_
#define ZEPHYR_DRIVERS_SENSOR_VCNL4035_VCNL4035_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

/* Registers all 16 bits */
#define VCNL4035_REG_ALS_CONF	0x00
#define VCNL4035_REG_ALS_THDH	0x01
#define VCNL4035_REG_ALS_THDL	0x02
#define VCNL4035_REG_PS_CONF_1_2	0x03
#define VCNL4035_REG_PS_MS_CONF3		0x04
#define VCNL4035_REG_PS_CANC	0x05
#define VCNL4035_REG_PS_THDL	0x06
#define VCNL4035_REG_PS_THDH	0x07
#define VCNL4035_REG_PS1_DATA	0x08
#define VCNL4035_REG_PS2_DATA	0x09
#define VCNL4035_REG_PS3_DATA	0x0A
#define VCNL4035_REG_ALS_DATA	0x0B
#define VCNL4035_REG_WHITE_DATA	0x0C
#define VCNL4035_REG_INT_FLAG	0x0D
#define VCNL4035_REG_DEVICE_ID	0x0E

#define VCNL4035_0x60_ID	0x00
// TODO: actually use those other ids (p4)
#define VCNL4035_0x51_ID	0x10
#define VCNL4035_0x40_ID	0x20
#define VCNL4035_0x41_ID	0x30

#define VCNL4035_DEFAULT_ID	(0x0080 << VCNL4035_0x60_ID)

#define VCNL4035_LED_I_POS	8 // checked
#define VCNL4035_IRED_SELECT_POS	5 // checked
#define VCNL4035_PS_HD_POS	11
#define VCNL4035_PS_HD_ENABLED	(0x01 << VCNL4035_PS_HD_POS)
#define VCNL4035_PS_DUTY_POS	6
#define VCNL4035_PS_PERS_POS	4
#define VCNL4035_PS_IT_POS	1
#define VCNL4035_PS_SD_POS	0
//#define VCNL4035_PS_SD_MASK	(0x01 << VCNL4035_PS_SD_POS)
#define VCNL4035_ALS_IT_POS	6
#define VCNL4035_ALS_SD_POS	0
#define VCNL4035_ALS_SD_MASK	(0x01 << VCNL4035_ALS_SD_POS)
#define VCNL4035_PS_GAIN_POS 12

#define VCNL4035_PS_TRIG_POS	2
#define VCNL4035_PS_TRIG_ENABLED	(0x1 << VCNL4035_PS_TRIG_POS)
#define VCNL4035_PS_TRIG_DISABLED	(0x0 << VCNL4035_PS_TRIG_POS)
#define VCNL4035_PS_MS_POS	1
#define VCNL4035_PS_MS_ENABLED	(0x1 << VCNL4035_PS_MS_POS)
#define VCNL4035_PS_MS_DISABLED	(0x0 << VCNL4035_PS_MS_POS)
#define VCNL4035_PS_AF_POS	3
#define VCNL4035_PS_AF_ENABLED	(0x1 << VCNL4035_PS_AF_POS)
#define VCNL4035_PS_AF_DISABLED	(0x0 << VCNL4035_PS_AF_POS)

enum led_current {
	VCNL4035_LED_CURRENT_50MA,
	VCNL4035_LED_CURRENT_75MA,
	VCNL4035_LED_CURRENT_100MA,
	VCNL4035_LED_CURRENT_120MA,
	VCNL4035_LED_CURRENT_140MA,
	VCNL4035_LED_CURRENT_160MA,
	VCNL4035_LED_CURRENT_180MA,
	VCNL4035_LED_CURRENT_200MA,
};

enum ired_select {
	IRED1,
	IRED2,
	IRED3
};

enum led_duty_cycle {
	VCNL4035_LED_DUTY_1_40,
	VCNL4035_LED_DUTY_1_80,
	VCNL4035_LED_DUTY_1_160,
	VCNL4035_LED_DUTY_1_320,
};

enum ambient_integration_time {
	VCNL4035_AMBIENT_INTEGRATION_TIME_80MS,
	VCNL4035_AMBIENT_INTEGRATION_TIME_160MS,
	VCNL4035_AMBIENT_INTEGRATION_TIME_320MS,
	VCNL4035_AMBIENT_INTEGRATION_TIME_640MS,
};

enum proximity_integration_time {
	VCNL4035_PROXIMITY_INTEGRATION_TIME_1T,
	VCNL4035_PROXIMITY_INTEGRATION_TIME_1_5T,
	VCNL4035_PROXIMITY_INTEGRATION_TIME_2T,
	VCNL4035_PROXIMITY_INTEGRATION_TIME_2_5T,
	VCNL4035_PROXIMITY_INTEGRATION_TIME_3T,
	VCNL4035_PROXIMITY_INTEGRATION_TIME_3_5T,
	VCNL4035_PROXIMITY_INTEGRATION_TIME_4T,
	VCNL4035_PROXIMITY_INTEGRATION_TIME_8T,
};

enum proximity_type {
	VCNL4035_PROXIMITY_INT_DISABLE,
	VCNL4035_PROXIMITY_INT_CLOSE,
	VCNL4035_PROXIMITY_INT_AWAY,
	VCNL4035_PROXIMITY_INT_CLOSE_AWAY,
};

enum interrupt_type {
	VCNL4035_PROXIMITY_AWAY = 1,
	VCNL4035_PROXIMITY_CLOSE,
	VCNL4035_AMBIENT_HIGH = 4,
	VCNL4035_AMBIENT_LOW,
};

struct vcnl4035_config {
	struct i2c_dt_spec i2c;
#ifdef CONFIG_VCNL4035_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif
	enum led_current led_i;
	enum ired_select ired_s;
	enum led_duty_cycle led_dc;
	enum ambient_integration_time als_it;
	enum proximity_integration_time proxy_it;
	enum proximity_type proxy_type;
};

struct vcnl4035_data {
	struct k_sem sem;
#ifdef CONFIG_VCNL4035_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	enum interrupt_type int_type;
	sensor_trigger_handler_t proxy_handler;
	sensor_trigger_handler_t als_handler;
#endif
#ifdef CONFIG_VCNL4035_TRIGGER_OWN_THREAD
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_VCNL4035_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem trig_sem;
#endif
#ifdef CONFIG_VCNL4035_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
	uint16_t proximity;
	uint16_t light;
	float sensitivity;
};

int vcnl4035_read(const struct device *dev, uint8_t reg, uint16_t *out);
int vcnl4035_write(const struct device *dev, uint8_t reg, uint16_t value);

#ifdef CONFIG_VCNL4035_TRIGGER
int vcnl4035_trigger_init(const struct device *dev);

int vcnl4035_attr_set(const struct device *dev,
		      enum sensor_channel chan,
		      enum sensor_attribute attr,
		      const struct sensor_value *val);

int vcnl4035_trigger_set(const struct device *dev,
			 const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler);
#endif

#endif
