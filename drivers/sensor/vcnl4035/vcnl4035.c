/*
 * Copyright (c) 2023 Felix Baral-Weber
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT vishay_vcnl4035

#include "vcnl4035.h"
#include <zephyr/pm/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(vcnl4035, CONFIG_SENSOR_LOG_LEVEL);

int vcnl4035_read(const struct device *dev, uint8_t reg, uint16_t *out)
{
	const struct vcnl4035_config *config = dev->config;
	uint8_t buff[2] = { 0 };
	int ret = 0;

	ret = i2c_write_read_dt(&config->i2c, &reg, sizeof(reg), buff, sizeof(buff));

	if (!ret) {
		*out = sys_get_le16(buff);
		LOG_DBG("read value %x", sys_get_le16(buff));
	}

	if (ret) {
		LOG_ERR("some error");
	}

	return ret;
}

int vcnl4035_write(const struct device *dev, uint8_t reg, uint16_t value)
{
	const struct vcnl4035_config *config = dev->config;
	struct i2c_msg msg;
	int ret;
	uint8_t buff[3];

	sys_put_le16(value, &buff[1]);

	buff[0] = reg;

	msg.buf = buff;
	msg.flags = 0;
	msg.len = sizeof(buff);

	// this new write function does work. it is tested
	// the old one ade the bus stuck
	// TODO: this this i2c_msg stuff can be deleted now
	//ret = i2c_transfer_dt(&config->i2c, &msg, 1);
	ret = i2c_write_dt(&config->i2c, buff, 3);

	if (ret < 0) {
		LOG_ERR("write block failed %d", ret);
		return ret;
	}

	return 0;
}

static int vcnl4035_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct vcnl4035_data *data = dev->data;
	int ret = 0;

#ifdef CONFIG_VCNL4035_ENABLE_ALS
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_PROX ||
			chan == SENSOR_CHAN_LIGHT);
#else
	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_PROX);
#endif
	k_sem_take(&data->sem, K_FOREVER);

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_PROX) {
		ret = vcnl4035_read(dev, VCNL4035_REG_PS1_DATA,
				    &data->proximity);
		if (ret < 0) {
			LOG_ERR("Could not fetch proximity");
			goto exit;
		}
	}
#ifdef CONFIG_VCNL4035_ENABLE_ALS
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_LIGHT) {
		ret = vcnl4035_read(dev, VCNL4035_REG_ALS_DATA,
				    &data->light);
		if (ret < 0) {
			LOG_ERR("Could not fetch ambient light");
		}
	}
#endif
exit:
	k_sem_give(&data->sem);

	return ret;
}

static int vcnl4035_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	struct vcnl4035_data *data = dev->data;
	int ret = 0;

	k_sem_take(&data->sem, K_FOREVER);

	switch (chan) {
	case SENSOR_CHAN_PROX:
		val->val1 = data->proximity;
		val->val2 = 0;
		break;

#ifdef CONFIG_VCNL4035_ENABLE_ALS
	case SENSOR_CHAN_LIGHT:
		val->val1 = data->light * data->sensitivity;
		val->val2 = 0;
		break;
#endif

	default:
		ret = -ENOTSUP;
	}

	k_sem_give(&data->sem);

	return ret;
}

static int vcnl4035_configure_PS_CONF_1_PS_CONF_2(const struct device *dev, const struct vcnl4035_config *config) {

	uint16_t conf = 0;
	// if (vcnl4035_read(dev, VCNL4035_REG_PS_CONF_1_2, &conf)) {
	// 	LOG_ERR("Could not read proximity config 2 :(");
	// 	return -EIO;
	// }

	// CONF 1
	// ired duty cycle (PS_DUTY)
	conf |= config->led_dc << VCNL4035_PS_DUTY_POS;
	// interrupt persistence (PS_PERS)
	conf |= 0x00 << VCNL4035_PS_PERS_POS;
	// Set integration time  (PS_IT)
	conf |= config->proxy_it << VCNL4035_PS_IT_POS;
	// disable proximity sensor shutdown (PS_SD) 
	conf |= 0 << VCNL4035_PS_SD_POS;

	// CONF 2
	// disable GESTURE_INT_EN
	// - 
	// disable GESTURE_MODE
	// - 
	// gain (PS_Gain)
	// TODO: make configurable
	conf |= 0 << VCNL4035_PS_GAIN_POS; // two step mode
	// 12 or 16 bit outpu (PS_HD)
	conf |= VCNL4035_PS_HD_ENABLED; // 16 bit
	// PS_NS
	// - 
	// PS_INT
	// - 


	LOG_DBG("PS_CONF_1 PS_CONF_2 0x%x", conf);


	if (vcnl4035_write(dev, VCNL4035_REG_PS_CONF_1_2, conf)) {
		LOG_ERR("Could not write proximity config");
		return -EIO;
	}
	return 0;
}

static int vcnl4035_configure_PS_CONF_3_PS_MS(const struct device *dev, const struct vcnl4035_config *config) {
	uint16_t conf = 0;

	// if (vcnl4035_read(dev, VCNL4035_REG_PS_MS_CONF3, &conf)) {
	// 	LOG_ERR("Could not read proximity config");
	// 	return -EIO;
	// }

	// PS_MS
	// PS_SC_CUR
	// -
	// PS_SP
	// -
	// sunlight protect mode (PS_SPO)
	// -
	// led current selection (LED_I)
	conf |= config->led_i << VCNL4035_LED_I_POS;
	
	
	// PS_CONF_3
	// LED_I_LOW
	// -
	// Set used led (IRED select)
	//conf |= config->ired_s << VCNL4035_IRED_SELECT_POS;
	conf |= IRED1 << VCNL4035_IRED_SELECT_POS;
	// SMART_PERS
	// - 
	// enable active force mode enabled (PS_AF)
	conf |= VCNL4035_PS_AF_DISABLED;
	// active force mode (PS_TRIG)
	conf |= VCNL4035_PS_TRIG_ENABLED;
	// normal operation with interrupt (PS_MS)
	conf |= VCNL4035_PS_MS_DISABLED;
	// sun cancel (PS_SC_EN)
	// -
	
	LOG_DBG("PS_CONF_3 PS_MS 0x%x", conf);

	if (vcnl4035_write(dev, VCNL4035_REG_PS_MS_CONF3, conf)) {
		LOG_ERR("Could not write proximity config");
		return -EIO;
	}
	return 0;
}

static int vcnl4035_proxy_setup(const struct device *dev)
{
	const struct vcnl4035_config *config = dev->config;
	

	LOG_DBG("vcnl4035_proxy_setup");

	// configure CONF 1 and 2
	vcnl4035_configure_PS_CONF_1_PS_CONF_2(dev, config);

	// configure PS_MS and CONF 3
	vcnl4035_configure_PS_CONF_3_PS_MS(dev, config);


	return 0;
}

#ifdef CONFIG_VCNL4035_ENABLE_ALS
static int vcnl4035_ambient_setup(const struct device *dev)
{
	const struct vcnl4035_config *config = dev->config;
	struct vcnl4035_data *data = dev->data;
	uint16_t conf = 0;

	if (vcnl4035_read(dev, VCNL4035_REG_ALS_CONF, &conf)) {
		LOG_ERR("Could not read proximity config ALS");
		return -EIO;
	}

	/* Set ALS integration time */
	conf |= config->als_it << VCNL4035_ALS_IT_POS;
	/* Clear ALS shutdown */
	conf &= ~VCNL4035_ALS_SD_MASK;

	if (vcnl4035_write(dev, VCNL4035_REG_ALS_CONF, conf)) {
		LOG_ERR("Could not write proximity config");
		return -EIO;
	}

	/*
	 * scale the lux depending on the value of the integration time
	 * see page 8 of the VCNL4035 application note:
	 * https://www.vishay.com/docs/84307/designingvcnl4035.pdf
	 */
	switch (config->als_it) {
	case VCNL4035_AMBIENT_INTEGRATION_TIME_80MS:
		data->sensitivity = 0.12;
		break;
	case VCNL4035_AMBIENT_INTEGRATION_TIME_160MS:
		data->sensitivity = 0.06;
		break;
	case VCNL4035_AMBIENT_INTEGRATION_TIME_320MS:
		data->sensitivity = 0.03;
		break;
	case VCNL4035_AMBIENT_INTEGRATION_TIME_640MS:
		data->sensitivity = 0.015;
		break;
	default:
		data->sensitivity = 1.0;
		LOG_WRN("Cannot set ALS sensitivity from ALS_IT=%d",
			config->als_it);
		break;
	}

	return 0;
}
#endif

#ifdef CONFIG_PM_DEVICE
static int vcnl4035_pm_action(const struct device *dev,
			      enum pm_device_action action)
{
	int ret = 0;
	uint16_t ps_conf;

	ret = vcnl4035_read(dev, VCNL4035_REG_PS_CONF_1_2, &ps_conf);
	if (ret < 0)
		return ret;
#ifdef CONFIG_VCNL4035_ENABLE_ALS
	uint16_t als_conf;

	ret = vcnl4035_read(dev, VCNL4035_REG_ALS_CONF, &als_conf);
	if (ret < 0)
		return ret;
#endif
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Clear proximity shutdown */
		ps_conf &= ~VCNL4035_PS_SD_MASK;

		ret = vcnl4035_write(dev, VCNL4035_REG_PS_CONF_1_2,
					ps_conf);
		if (ret < 0)
			return ret;
#ifdef CONFIG_VCNL4035_ENABLE_ALS
		/* Clear als shutdown */
		als_conf &= ~VCNL4035_ALS_SD_MASK;

		ret = vcnl4035_write(dev, VCNL4035_REG_ALS_CONF,
					als_conf);
		if (ret < 0)
			return ret;
#endif
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Set proximity shutdown bit 0 */
		ps_conf |= VCNL4035_PS_SD_MASK;

		ret = vcnl4035_write(dev, VCNL4035_REG_PS_CONF_1_2,
					ps_conf);
		if (ret < 0)
			return ret;
#ifdef CONFIG_VCNL4035_ENABLE_ALS
		/* Clear als shutdown bit 0 */
		als_conf |= VCNL4035_ALS_SD_MASK;

		ret = vcnl4035_write(dev, VCNL4035_REG_ALS_CONF,
					als_conf);
		if (ret < 0)
			return ret;
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif

static int vcnl4035_init(const struct device *dev)
{
	const struct vcnl4035_config *config = dev->config;
	struct vcnl4035_data *data = dev->data;
	uint16_t id;

	/* Get the I2C device */
	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	/* Check device id */
	if (vcnl4035_read(dev, VCNL4035_REG_DEVICE_ID, &id)) {
		LOG_ERR("Could not read device id");
		return -EIO;
	}

	if (id != VCNL4035_DEFAULT_ID) {
		LOG_ERR("Incorrect device id (%d)", id);
		return -EIO;
	}

	// ###### ###### ###### ######
	if (vcnl4035_proxy_setup(dev)) {
		LOG_ERR("Failed to setup proximity functionality");
		return -EIO;
	}

#ifdef CONFIG_VCNL4035_ENABLE_ALS
	if (vcnl4035_ambient_setup(dev)) {
		LOG_ERR("Failed to setup ambient light functionality");
		return -EIO;
	}
#endif

	k_sem_init(&data->sem, 0, K_SEM_MAX_LIMIT);

#if CONFIG_VCNL4035_TRIGGER
	if (config->int_gpio.port) {
		if (vcnl4035_trigger_init(dev)) {
			LOG_ERR("Could not initialise interrupts");
			return -EIO;
		}
	}
#endif

	k_sem_give(&data->sem);

	LOG_DBG("Init complete");

	return 0;
}

static const struct sensor_driver_api vcnl4035_driver_api = {
	.sample_fetch = vcnl4035_sample_fetch,
	.channel_get = vcnl4035_channel_get,
#ifdef CONFIG_VCNL4035_TRIGGER
	.attr_set = vcnl4035_attr_set,
	.trigger_set = vcnl4035_trigger_set,
#endif
};

#define VCNL4035_DEFINE(inst)									\
	static struct vcnl4035_data vcnl4035_data_##inst;					\
												\
	static const struct vcnl4035_config vcnl4035_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		IF_ENABLED(CONFIG_VCNL4035_TRIGGER,						\
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),))	\
		.led_i = DT_INST_ENUM_IDX(inst, led_current),					\
		.led_dc = DT_INST_ENUM_IDX(inst, led_duty_cycle),				\
		.als_it = DT_INST_ENUM_IDX(inst, als_it),					\
		.proxy_it = DT_INST_ENUM_IDX(inst, proximity_it),				\
		.proxy_type = DT_INST_ENUM_IDX(inst, proximity_trigger),			\
	};											\
												\
	PM_DEVICE_DT_INST_DEFINE(inst, vcnl4035_pm_action);					\
												\
	DEVICE_DT_INST_DEFINE(inst, vcnl4035_init, PM_DEVICE_DT_INST_GET(inst),			\
			      &vcnl4035_data_##inst, &vcnl4035_config_##inst, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &vcnl4035_driver_api);		\

DT_INST_FOREACH_STATUS_OKAY(VCNL4035_DEFINE)
