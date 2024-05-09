/**
 * @file drivers/sensor.h
 *
 * @brief Public APIs for the sensor driver.
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_INTROSPECT_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTROSPECT_H_

/**
 * @brief Sensor Interface
 * @defgroup sensor_interface Sensor Interface
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
//#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/dsp/types.h>
//#include <zephyr/rtio/rtio.h>
//#include <zephyr/sys/iterable_sections.h>
//#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @typedef set_data_t
 * @brief Callback API upon setting a sensor's attributes
 *
 * See i3c_set_data() for argument description
 */
typedef int (*set_data_t)(const struct device *dev,
				 uint8_t *val, uint32_t len);

/**
 * @typedef get_data_t
 * @brief Callback API upon getting a sensor's attributes
 *
 * See i3c_get_data() for argument description
 */
typedef int (*get_data_t)(const struct device *dev,
				 uint8_t *val, uint32_t len);

__subsystem struct introspect_driver_api {
	set_data_t set_data;
	get_data_t get_data;
};

/**
 * @brief Set an attribute for a sensor
 *
 * @param dev Pointer to the sensor device
 * @param chan The channel the attribute belongs to, if any.  Some
 * attributes may only be set for all channels of a device, depending on
 * device capabilities.
 * @param attr The attribute to set
 * @param val The value to set the attribute to
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int i3c_set_data(const struct device *dev,
				 uint8_t *val, uint32_t len);

static inline int z_impl_i3c_set_data(const struct device *dev,
				 uint8_t *val, uint32_t len)
{
	const struct introspect_driver_api *api =
		(const struct introspect_driver_api *)dev->api;

	if (api->set_data == NULL) {
		return -ENOSYS;
	}

	return api->set_data(dev, val, len);
}

/**
 * @brief Get an attribute for a sensor
 *
 * @param dev Pointer to the sensor device
 * @param chan The channel the attribute belongs to, if any.  Some
 * attributes may only be set for all channels of a device, depending on
 * device capabilities.
 * @param attr The attribute to get
 * @param val Pointer to where to store the attribute
 *
 * @return 0 if successful, negative errno code if failure.
 */
__syscall int i3c_get_data(const struct device *dev, uint8_t *val, uint32_t len);

static inline int z_impl_i3c_get_data(const struct device *dev,
				 uint8_t *val, uint32_t len)
{
	const struct introspect_driver_api *api =
		(const struct introspect_driver_api *)dev->api;

	if (api->get_data == NULL) {
		return -ENOSYS;
	}

	return api->get_data(dev, val, len);
}

/**
 * @typedef sensor_processing_callback_t
 * @brief Callback function used with the helper processing function.
 *
 * @see sensor_processing_with_callback
 *
 * @param[in] result The result code of the read (0 being success)
 * @param[in] buf The data buffer holding the sensor data
 * @param[in] buf_len The length (in bytes) of the @p buf
 * @param[in] userdata The optional userdata passed to sensor_read()
 */
//typedef void (*sensor_processing_callback_t)(int result, uint8_t *buf, uint32_t buf_len,
	//				     void *userdata);

/**
 * @brief Helper function for common processing of sensor data.
 *
 * This function can be called in a blocking manner after sensor_read() or in a standalone
 * thread dedicated to processing. It will wait for a cqe from the RTIO context, once received, it
 * will decode the userdata and call the @p cb. Once the @p cb returns, the buffer will be released
 * back into @p ctx's mempool if available.
 *
 * @param[in] ctx The RTIO context to wait on
 * @param[in] cb Callback to call when data is ready for processing
 */
//void sensor_processing_with_callback(struct rtio *ctx, sensor_processing_callback_t cb);
struct introspect_info {
	const struct device *dev;
	const char *vendor;
	const char *model;
	const char *friendly_name;
};

#define SENSOR_INFO_INITIALIZER(_dev, _vendor, _model, _friendly_name)	\
	{								\
		.dev = _dev,						\
		.vendor = _vendor,					\
		.model = _model,					\
		.friendly_name = _friendly_name,			\
	}

#define SENSOR_INFO_DEFINE(name, ...)					\
	static const STRUCT_SECTION_ITERABLE(introspect_info, name) =	\
		SENSOR_INFO_INITIALIZER(__VA_ARGS__)

#define SENSOR_INFO_DT_NAME(node_id)					\
	_CONCAT(__sensor_info, DEVICE_DT_NAME_GET(node_id))

#define SENSOR_INFO_DT_DEFINE(node_id)					\
	SENSOR_INFO_DEFINE(SENSOR_INFO_DT_NAME(node_id),		\
			   DEVICE_DT_GET(node_id),			\
			   DT_NODE_VENDOR_OR(node_id, NULL),		\
			   DT_NODE_MODEL_OR(node_id, NULL),		\
			   DT_PROP_OR(node_id, friendly_name, NULL))	\


/**
 * @brief Like DEVICE_DT_DEFINE() with sensor specifics.
 *
 * @details Defines a device which implements the sensor API. May define an
 * element in the sensor info iterable section used to enumerate all sensor
 * devices.
 *
 * @param node_id The devicetree node identifier.
 *
 * @param init_fn Name of the init function of the driver.
 *
 * @param pm_device PM device resources reference (NULL if device does not use
 * PM).
 *
 * @param data_ptr Pointer to the device's private data.
 *
 * @param cfg_ptr The address to the structure containing the configuration
 * information for this instance of the driver.
 *
 * @param level The initialization level. See SYS_INIT() for details.
 *
 * @param prio Priority within the selected initialization level. See
 * SYS_INIT() for details.
 *
 * @param api_ptr Provides an initial pointer to the API function struct used
 * by the driver. Can be NULL.
 */
#define SENSOR_DEVICE_DT_DEFINE(node_id, init_fn, pm_device,		\
				data_ptr, cfg_ptr, level, prio,		\
				api_ptr, ...)				\
	DEVICE_DT_DEFINE(node_id, init_fn, pm_device,			\
			 data_ptr, cfg_ptr, level, prio,		\
			 api_ptr, __VA_ARGS__);				\
									\
	SENSOR_INFO_DT_DEFINE(node_id);

/**
 * @brief Like SENSOR_DEVICE_DT_DEFINE() for an instance of a DT_DRV_COMPAT
 * compatible
 *
 * @param inst instance number. This is replaced by
 * <tt>DT_DRV_COMPAT(inst)</tt> in the call to SENSOR_DEVICE_DT_DEFINE().
 *
 * @param ... other parameters as expected by SENSOR_DEVICE_DT_DEFINE().
 */
#define SENSOR_DEVICE_DT_INST_DEFINE(inst, ...)				\
	SENSOR_DEVICE_DT_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#include <syscalls/introspect_i3c.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_H_ */
