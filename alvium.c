// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 beh.digital GmbH
 * Author: Nicolai Behmann <behmann@beh.digital>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/crc32.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/lcm.h>
#include <linux/jiffies.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ctrls.h>

#include "alvium.h"

// forward declaration
static int alvium_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval);
static int alvium_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format);

#define alvium_dbg(dev, fmt, args...) \
	dev_info(dev, "%s:%d: " fmt "", __func__, __LINE__, ##args)

#define alvium_warn(dev, fmt, args...) \
	dev_warn(dev, "%s:%d: " fmt "", __func__, __LINE__, ##args)

#define I2C_IO_LIMIT 256 // jetson: 1024, imx8: 256
#define BCRM_WAIT_HANDSHAKE_TIMEOUT 3000

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define ALVIUM_DEFAULT_FPS 30
#define ALVIUM_DEFAULT_FMT MEDIA_BUS_FMT_YUYV8_2X8
// MEDIA_BUS_FMT_Y8_1X8
// MEDIA_BUS_FMT_YUYV8_2X8 // TODO format to alwas VYUY

#define ALVIUM_NUM_PADS 1

static int add_wait_time_ms = 2000;
module_param(add_wait_time_ms, int, 0600);

static const unsigned int alvium_formats[] = {
	MEDIA_BUS_FMT_Y8_1X8,
	MEDIA_BUS_FMT_Y10_1X10,
	MEDIA_BUS_FMT_Y12_1X12,
	// MEDIA_BUS_FMT_SGBRG8_1X8,
	// MEDIA_BUS_FMT_SBGGR8_1X8,
	// MEDIA_BUS_FMT_SRGGB8_1X8,
	// MEDIA_BUS_FMT_SGRBG8_1X8,
	// MEDIA_BUS_FMT_SGBRG10_1X10,
	// MEDIA_BUS_FMT_SBGGR10_1X10,
	// MEDIA_BUS_FMT_SRGGB10_1X10,
	// MEDIA_BUS_FMT_SGRBG10_1X10,
	// MEDIA_BUS_FMT_SGBRG12_1X12,
	// MEDIA_BUS_FMT_SBGGR12_1X12,
	// MEDIA_BUS_FMT_SRGGB12_1X12,
	// MEDIA_BUS_FMT_SGRBG12_1X12,
	// MEDIA_BUS_FMT_RGB444_1X12
	// MEDIA_BUS_FMT_RGB565_1X16
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_BGR888_1X24,
	MEDIA_BUS_FMT_UYVY8_2X8,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_YUYV8_2X8,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_VYUY8_2X8,
	MEDIA_BUS_FMT_VYUY8_1X16,
};

struct range32 {
	u32 min;
	u32 max;
	u32 step;
};

struct alvium_sensor_limits {
	struct range32 w; // width
	struct range32 h; // height
	struct range32 woff; // width offset
	struct range32 hoff; // height offset
};

struct range64 {
	u64 min;
	u64 max;
	u64 step;
};

struct alvium_mipi_businfo {
	u32 num_lanes;
	u32 clk_freq;
	u32 fmt_code;
};

struct alvium {
	struct v4l2_subdev subdev; // used
	struct device *dev; // used
	enum alvium_mode mode;

	struct media_pad pad[1]; // used
	struct gpio_desc *reset_gpio; // used

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq_ctrl;
	struct v4l2_ctrl *exposure_ctrl;
	struct v4l2_ctrl *exposure_absolute_ctrl;
	bool cross_update;
	struct v4l2_ctrl *auto_exposure_ctrl;
	struct v4l2_ctrl *auto_gain_ctrl;
	struct v4l2_ctrl *red_balance_ctrl;
	struct v4l2_ctrl *blue_balance_ctrl;

	// configuration
	struct v4l2_mbus_framefmt fmt;

	// frame sizes (current and supported)
	struct v4l2_rect crop;
	struct alvium_sensor_limits limits;

	// frame intervals (current and supported)
	struct v4l2_fract interval;
	struct range64 intervals;

	struct alvium_mipi_businfo minfo; // used

	struct cci_reg cci_reg; // used
	unsigned int *fmts; // used
	unsigned int num_fmts; // used

	struct mutex lock; // somehow used

	bool is_open; // used
	bool is_streaming; // used

	bool write_handshake_available; // used
};

static int alvium_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel);

static int alvium_create_ctrls(struct alvium *sensor);

static int alvium_write_param(struct alvium *sensor, u32 id, u32 val);

static inline struct alvium *to_alvium(struct v4l2_subdev *sd)
{
	return container_of(sd, struct alvium, subdev);
}

static void swapbytes(void *_object, size_t _size)
{
	switch (_size) {
	case 2:
		cpu_to_be16s((u16 *)_object);
		break;

	case 4:
		cpu_to_be32s((u32 *)_object);
		break;

	case 8:
		cpu_to_be64s((u64 *)_object);
		break;
	}
}

static bool common_range(u32 nMin1, u32 nMax1, u32 nInc1, u32 nMin2, u32 nMax2,
			 u32 nInc2, u32 *rMin, u32 *rMax, u32 *rInc)
{
	bool bResult = false;

	u32 nMin = max(nMin1, nMin2);
	u32 nMax = min(nMax1, nMax2);

	/* Check if it is overlapping at all */
	if (nMax >= nMin) {
		/* if both minima are equal,
		 * then the computation is a bit simpler
		 */
		if (nMin1 == nMin2) {
			u32 nLCM = lcm(nInc1, nInc2);
			*rMin = nMin;
			*rMax = nMax - ((nMax - nMin) % nLCM);

			if (*rMin == *rMax)
				*rInc = 1;
			else
				*rInc = nLCM;

			bResult = true;
		} else if (nMin1 > nMin2) {
			/* Find the first value that is ok for Host and BCRM */
			u32 nMin1Shifted = nMin1 - nMin2;
			u32 nMaxShifted = nMax - nMin2;
			u32 nValue = nMin1Shifted;

			for (; nValue <= nMaxShifted; nValue += nInc1) {
				if ((nValue % nInc2) == 0)
					break;
			}

			/* Compute common increment and maximum */
			if (nValue <= nMaxShifted) {
				u32 nLCM = lcm(nInc1, nInc2);
				*rMin = nValue + nMin2;
				*rMax = nMax - ((nMax - *rMin) % nLCM);

				if (*rMin == *rMax)
					*rInc = 1;
				else
					*rInc = nLCM;

				bResult = true;
			}
		} else {
			/* Find the first value that is ok for Host and BCRM */
			u32 nMin2Shifted = nMin2 - nMin1;
			u32 nMaxShifted = nMax - nMin1;
			u32 nValue = nMin2Shifted;

			for (; nValue <= nMaxShifted; nValue += nInc2) {
				if ((nValue % nInc1) == 0)
					break;
			}

			/* Compute common increment and maximum */
			if (nValue <= nMaxShifted) {
				u32 nLCM = lcm(nInc2, nInc1);
				*rMin = nValue + nMin1;
				*rMax = nMax - ((nMax - *rMin) % nLCM);
				if (*rMin == *rMax)
					*rInc = 1;
				else
					*rInc = nLCM;

				bResult = true;
			}
		}
	}

	return bResult;
}

static int convert_bcrm_to_v4l2(struct bcrm_to_v4l2 *bcrmv4l2, int conversion,
				bool abs)
{
	int64_t value = 0;
	int32_t min = 0;
	int32_t max = 0;
	int32_t step = 0;
	int32_t result = 0;
	int32_t valuedown = 0;
	int32_t valueup = 0;

	step = 1;
	max = S32_MAX;

	/* 1. convert to double */
	if (conversion == min_enum) {
		value = bcrmv4l2->min_bcrm;
		// min = 1;
	} else if (conversion == max_enum) {
		value = bcrmv4l2->max_bcrm;
		min = bcrmv4l2->min_bcrm;
	} else if (conversion == step_enum) {
		value = bcrmv4l2->step_bcrm;
		min = S32_MIN;
	}

	/* Clamp to limits of int32 representation */
	if (value > S32_MAX)
		value = S32_MAX;
	if (value < S32_MIN)
		value = S32_MIN;

	/* 2. convert the units */
	/*	value *= factor; */
	if (abs) {
		if (value != 0)
			do_div(value, 1UL);
	}
	/* V4L2_CID_EXPOSURE_ABSOLUTE */
	else {
		if (value != 0)
			do_div(value, EXP_ABS);
	}

	/* 3. Round value to next integer */
	if (value < S32_MIN)
		result = S32_MIN;
	else if (value > S32_MAX)
		result = S32_MAX;
	else
		result = value;

	/* 4. Clamp to limits */
	if (result > max)
		result = max;
	else if (result < min)
		result = min;

	/* 5. Use nearest increment */
	valuedown = result - ((result - min) % (step));
	valueup = valuedown + step;

	if (result >= 0) {
		if (((valueup - result) <= (result - valuedown)) &&
		    (valueup <= bcrmv4l2->max_bcrm))
			result = valueup;
		else
			result = valuedown;
	} else {
		if (((valueup - result) < (result - valuedown)) &&
		    (valueup <= bcrmv4l2->max_bcrm))
			result = valueup;
		else
			result = valuedown;
	}

	if (conversion == min_enum)
		bcrmv4l2->min_v4l2 = result;
	else if (conversion == max_enum)
		bcrmv4l2->max_v4l2 = result;
	else if (conversion == step_enum)
		bcrmv4l2->step_v4l2 = result;
	return 0;
}

static int32_t convert_bcrm_to_v4l2_gctrl(struct bcrm_to_v4l2 *bcrmv4l2,
					  int64_t val64, bool abs)
{
	int32_t value = 0;
	int32_t min = 0;
	int32_t max = 0;
	int32_t step = 0;
	int32_t result = 0;
	int32_t valuedown = 0;
	int32_t valueup = 0;

	/* 1. convert to double */
	step = bcrmv4l2->step_v4l2;
	max = bcrmv4l2->max_v4l2;
	min = bcrmv4l2->min_v4l2;
	value = (int32_t)val64;

	/* 2. convert the units */
	/*	value *= factor; */

	/* V4L2_CID_EXPOSURE_ABSOLUTE */
	if (abs) {
		if (value != 0)
			do_div(value, EXP_ABS);
	}

	/* 3. Round value to next integer */
	if (value < S32_MIN)
		result = S32_MIN;
	else if (value > S32_MAX)
		result = S32_MAX;
	else
		result = value;

	/* 4. Clamp to limits */
	if (result > max)
		result = max;
	else if (result < min)
		result = min;

	/* 5. Use nearest increment */
	valuedown = result - ((result - min) % (step));
	valueup = valuedown + step;

	if (result >= 0) {
		if (((valueup - result) <= (result - valuedown)) &&
		    (valueup <= bcrmv4l2->max_bcrm))
			result = valueup;
		else
			result = valuedown;
	} else {
		if (((valueup - result) < (result - valuedown)) &&
		    (valueup <= bcrmv4l2->max_bcrm))
			result = valueup;
		else
			result = valuedown;
	}

	return result;
}

static int i2c_read(struct i2c_client *i2c, u32 reg, u32 size, u32 count,
		    u8 *buf)
{
	struct i2c_msg msg[2];
	u8 *msgbuf;
	int ret = 0, i = 0, j = 0, reg_size_bkp = size;

	msgbuf = kzalloc(size, GFP_KERNEL);
	if (!msgbuf)
		return -ENOMEM;

	/* clearing i2c msg with 0's */
	memset(msg, 0, sizeof(msg));

	if (count > I2C_IO_LIMIT) {
		dev_err(&i2c->dev,
			"Limit excedded! i2c_reg->count > I2C_IO_LIMIT");
		count = I2C_IO_LIMIT;
	}

	/* find start address of buffer */
	for (i = --size; i >= 0; i--, j++)
		msgbuf[i] = ((reg >> (8 * j)) & 0xFF);

	msg[0].addr = i2c->addr;
	msg[0].flags = 0;
	msg[0].len = reg_size_bkp;
	msg[0].buf = msgbuf;
	msg[1].addr = i2c->addr; /* slave address */
	msg[1].flags = I2C_M_RD; /* read flag setting */
	msg[1].len = count;
	msg[1].buf = buf; /* dest buffer */

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));

	kfree(msgbuf);

	return ret;
}

static int alvium_read(struct alvium *sensor, u32 reg, u32 reg_size, u32 count,
		       u8 *buffer)
{
	int ret;
	struct i2c_client *i2c = v4l2_get_subdevdata(&sensor->subdev);

	ret = i2c_read(i2c, reg, reg_size, count, buffer);
	if (ret < 0)
		return ret;

	swapbytes(buffer, count);
	return ret;
}

static int i2c_write(struct i2c_client *i2c, u32 reg, u32 reg_size,
		     u32 buf_size, u8 *buf)
{
	int j = 0, i = 0;
	u8 *i2c_w_buf;
	int ret = 0;

	/* count exceeds writing I2C_IO_LIMIT characters */
	if (buf_size > I2C_IO_LIMIT) {
		dev_err(&i2c->dev,
			"limit excedded! i2c_reg->count > I2C_IO_LIMIT");
		buf_size = I2C_IO_LIMIT;
	}

	i2c_w_buf = kzalloc(buf_size + reg_size, GFP_KERNEL);
	if (!i2c_w_buf)
		return -ENOMEM;

	/* Fill the address in buffer upto size of address want to write */
	for (i = reg_size - 1, j = 0; i >= 0; i--, j++)
		i2c_w_buf[i] = ((reg >> (8 * j)) & 0xFF);

	/* Append the data value in the same buffer */
	memcpy(i2c_w_buf + reg_size, buf, buf_size);

	ret = i2c_master_send(i2c, i2c_w_buf, buf_size + reg_size);

	kfree(i2c_w_buf);

	if (ret < 0)
		return ret;

	return ret;
}

/**
 * @brief Since the camera needs a few ms to process written data, we need to poll
   the handshake register to make sure to continue not too early with the next write access.
 *
 * @param timeout_ms : Timeout value in ms
 * @return uint64_t : Duration in ms
 */
static u64 wait_for_bcrm_write_handshake(struct alvium *sensor, u64 timeout_ms)
{
	struct i2c_client *i2c = v4l2_get_subdevdata(&sensor->subdev);
	static const int poll_interval_ms = 10;
	static const int default_wait_time_ms = 50;
	int ret = 0;
	u8 buffer[3] = { 0 };
	u8 handshake_val = 0;
	bool handshake_valid = false;
	u64 duration_ms = 0;
	u64 start_jiffies = get_jiffies_64();
	unsigned long timeout_jiffies = jiffies + msecs_to_jiffies(timeout_ms);

	if (sensor->write_handshake_available) {
		/* We need to poll the handshake register and wait until the camera has processed the data */
		alvium_dbg(sensor->dev,
			   " Wait for 'write done' bit (0x81) ...");
		do {
			usleep_range(poll_interval_ms * 1000,
				     (poll_interval_ms * 1000) + 1);
			/* Read handshake register */
			ret = alvium_read(sensor,
					  sensor->cci_reg.bcrm_addr +
						  BCRM_WRITE_HANDSHAKE_8RW,
					  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8,
					  (char *)&handshake_val);

			dev_dbg(sensor->dev, " register read: %d, ret = %d",
				handshake_val, ret);

			if (ret >= 0) {
				/* Check, if handshake bit is set */
				if ((handshake_val & 0x01) == 1) {
					do {
						/* Handshake set by camera. We should to reset it */
						buffer[0] =
							(sensor->cci_reg
								 .bcrm_addr +
							 BCRM_WRITE_HANDSHAKE_8RW) >>
							8;
						buffer[1] =
							(sensor->cci_reg
								 .bcrm_addr +
							 BCRM_WRITE_HANDSHAKE_8RW) &
							0xff;
						buffer[2] =
							(handshake_val &
							 0xFE); /* Reset LSB (handshake bit)*/
						ret = i2c_master_send(
							i2c, buffer,
							sizeof(buffer));
						if (ret < 0) {
							dev_err(sensor->dev,
								" cannot reset handshake bit");
							return ret;
						}
						dev_dbg(sensor->dev,
							" handshake bit resetted");

						if (ret >= 0) {
							/* Since the camera needs a few ms for every write access to finish, we need to poll here too */
							dev_dbg(sensor->dev,
								" Wait for reset of 'write done' bit (0x80) ...");
							do {
								usleep_range(
									poll_interval_ms *
										1000,
									(poll_interval_ms *
									 1000) +
										1);
								/* We need to wait again until the bit is reset */
								ret = alvium_read(
									sensor,
									sensor->cci_reg.bcrm_addr +
										BCRM_WRITE_HANDSHAKE_8RW,
									AV_CAM_REG_SIZE,
									AV_CAM_DATA_SIZE_8,
									(char *)&handshake_val);

								if (ret >= 0) {
									if ((handshake_val &
									     0x1) ==
									    0) /* Verify write */
									{
										duration_ms = jiffies_to_msecs(
											get_jiffies_64() -
											start_jiffies);
										handshake_valid =
											true;
										break;
									}
									usleep_range(
										poll_interval_ms *
											1000,
										(poll_interval_ms *
										 1000) +
											1);
								} else {
									dev_err(sensor->dev,
										" Error while reading WRITE_HANDSHAKE_REG_8RW register.");
									break;
								}
							} while (time_before(
								jiffies,
								timeout_jiffies));
							if (!handshake_valid) {
								dev_warn(
									sensor->dev,
									"Verify handshake timeout");
							}
							break;
						} else {
							dev_err(sensor->dev,
								"Error while writing WRITE_HANDSHAKE_REG_8RW register.");
							break;
						}
					} while (!handshake_valid &&
						 time_before(jiffies,
							     timeout_jiffies));
				}
			} else {
				dev_err(sensor->dev,
					"Error while reading WRITE_HANDSHAKE_REG_8RW register.");
				break;
			}
		} while (!handshake_valid &&
			 time_before(jiffies, timeout_jiffies));

		if (!handshake_valid) {
			dev_err(sensor->dev,
				"Write handshake timeout! (Register 0x%02X)",
				ret);
		}
	} else {
		/* Handshake not supported. Use static sleep at least once as fallback */
		usleep_range(default_wait_time_ms * 1000,
			     (default_wait_time_ms * 1000) + 1);
		duration_ms =
			jiffies_to_msecs(get_jiffies_64() - start_jiffies);
	}

	return duration_ms;
}

static int alvium_write(struct alvium *sensor, u32 reg, u32 reg_size,
			u32 buf_size, u8 *buf)
{
	struct i2c_client *i2c = v4l2_get_subdevdata(&sensor->subdev);
	u64 duration = 0;
	int ret = 0;

	swapbytes(buf, buf_size);

	alvium_dbg(sensor->dev, "reg: 0x%x, length: %d, buf_length: %d", reg,
		   reg_size, buf_size);

	ret = i2c_write(i2c, reg, reg_size, buf_size, buf);
	if (ret < 0) {
		dev_err(sensor->dev, "i2c_write err: %d", ret);
		return ret;
	}

	/* Wait for write handshake register only for BCM registers */
	if ((reg >= sensor->cci_reg.bcrm_addr) &&
	    (reg <= sensor->cci_reg.bcrm_addr + _BCRM_LAST_ADDR)) {
		duration = wait_for_bcrm_write_handshake(
			sensor, BCRM_WAIT_HANDSHAKE_TIMEOUT);
		dev_dbg(sensor->dev,
			"i2c write success reg=0x%x, duration=%lldms, ret=%d",
			reg, duration, ret);
	}

	// TODO TODO, if ret == transfer size, than we are fine and return 0?

	// TODO TODO this is a cheap "hack" to have correct ordering again! Better copy value above instead of swap inline
	swapbytes(buf, buf_size);

	return ret;
}

static int alvium_read_cci_registers(struct alvium *sensor)
{
	struct i2c_client *i2c = v4l2_get_subdevdata(&sensor->subdev);
	int ret = 0;
	u32 crc = 0;
	u32 crc_byte_count = 0;

	u32 i2c_reg = cci_cmd_tbl[CCI_REGISTER_LAYOUT_VERSION].address;
	u32 i2c_reg_count;
	u32 i2c_reg_left;

	u8 *i2c_reg_buf;

	/*
	 * Avoid last bytes read as its WRITE only registers
	 * TODO check, what heartbeat is (not documented) and if we need to read
	 */
	i2c_reg_left = i2c_reg_count = sizeof(sensor->cci_reg) - 4;

	i2c_reg_buf = (u8 *)&sensor->cci_reg;
	/* Calculate CRC from each reg up to the CRC reg */
	crc_byte_count =
		(u32)((u8 *)&sensor->cci_reg.checksum - (u8 *)&sensor->cci_reg);

	alvium_dbg(&i2c->dev, "crc_byte_count = %d, i2c_reg.count = %d",
		   crc_byte_count, i2c_reg_count);

	/* read CCI registers */
	while (i2c_reg_left) {
		ret = i2c_read(i2c, i2c_reg + (i2c_reg_count - i2c_reg_left),
			       AV_CAM_REG_SIZE,
			       i2c_reg_left > I2C_IO_LIMIT ? I2C_IO_LIMIT :
							     i2c_reg_left,
			       i2c_reg_buf + (i2c_reg_count - i2c_reg_left));
		if (i2c_reg_left > I2C_IO_LIMIT)
			i2c_reg_left -= I2C_IO_LIMIT;
		else
			i2c_reg_left = 0;
	}

	if (ret < 0) {
		dev_err(&i2c->dev, "i2c read failed: %d", ret);
		return ret;
	}

	/* CRC calculation */
	crc = crc32(U32_MAX, &sensor->cci_reg, crc_byte_count);

	/* Swap bytes if neccessary */
	cpu_to_be32s(&sensor->cci_reg.layout_version);
	cpu_to_be64s(&sensor->cci_reg.device_capabilities);
	cpu_to_be16s(&sensor->cci_reg.gcprm_address);
	cpu_to_be16s(&sensor->cci_reg.bcrm_addr);
	cpu_to_be32s(&sensor->cci_reg.checksum);

	/* Check the checksum of received with calculated. */
	if (crc != sensor->cci_reg.checksum) {
		dev_err(&i2c->dev,
			"wrong CCI CRC value! calculated = 0x%x, received = 0x%x",
			crc, sensor->cci_reg.checksum);
		return -EINVAL;
	}

	alvium_dbg(&i2c->dev, "cci layout version: %x",
		   sensor->cci_reg.layout_version);
	alvium_dbg(&i2c->dev, "cci device capabilities: %llx",
		   sensor->cci_reg.device_capabilities);
	alvium_dbg(&i2c->dev, "cci gcprm_address: 0x%x",
		   sensor->cci_reg.gcprm_address);
	alvium_dbg(&i2c->dev, "cci bcrm_addr: 0x%x", sensor->cci_reg.bcrm_addr);
	alvium_dbg(&i2c->dev, "cci device guid: %s",
		   sensor->cci_reg.device_guid);
	alvium_dbg(&i2c->dev, "cci manufacturer name: %s",
		   sensor->cci_reg.manufacturer_name);
	alvium_dbg(&i2c->dev, "cci model name: %s", sensor->cci_reg.model_name);
	alvium_dbg(&i2c->dev, "cci family name: %s",
		   sensor->cci_reg.family_name);
	alvium_dbg(&i2c->dev, "cci device version: %s",
		   sensor->cci_reg.device_version);
	alvium_dbg(&i2c->dev, "cci manufacturer info: %s",
		   sensor->cci_reg.manufacturer_info);
	alvium_dbg(&i2c->dev, "cci serial number: %s",
		   sensor->cci_reg.serial_number);
	alvium_dbg(&i2c->dev, "cci user defined name: %s",
		   sensor->cci_reg.user_defined_name);

	return 0;
}

static int alvium_power_on(struct alvium *sensor)
{
	/* TODO Enable power, clocks, etc... */
	/* TODO Implement use counter */
	return 0;
}

static int alvium_power_off(struct alvium *sensor)
{
	/* TODO Disable power, clocks, etc... */
	return 0;
}

/* V4L2 subdev core ops */
static int alvium_s_power(struct v4l2_subdev *sd, int on)
{
	struct alvium *sensor = to_alvium(sd);
	int ret = 0;

	alvium_dbg(sensor->dev, "on: %d", on);

	mutex_lock(&sensor->lock);

	if (on) {
		ret = alvium_power_on(sensor);
		if (ret < 0) {
			goto out;
		}
	}

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int alvium_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct alvium *sensor = to_alvium(sd);
	int ret = -EINVAL;

	// TODO get reg size depending on register
	// ret = alvium_write(sensor, reg->reg, AV_CAM_REG_SIZE, reg->val);
	return ret;
}

static int alvium_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct alvium *sensor = to_alvium(sd);
	int ret = -EINVAL;

	// TODO get reg size depending on register
	// ret = alvium_read(sensor, reg->reg, (u16 *)&reg->val);
	return ret;
}
#endif

static s32 convert_s_ctrl(s32 val, s32 min, s32 max, s32 step)
{
	s32 valuedown = 0, valueup = 0;

	if (val > max)
		val = max;

	else if (val < min)
		val = min;

	valuedown = val - ((val - min) % step);
	valueup = valuedown + step;

	if (val >= 0) {
		if (((valueup - val) <= (val - valuedown)) && (valueup <= max))
			val = valueup;
		else
			val = valuedown;
	} else {
		if (((valueup - val) < (val - valuedown)) && (valueup <= max))
			val = valueup;
		else
			val = valuedown;
	}

	return val;
}

static int alvium_set_mode(struct alvium *sensor, enum alvium_mode mode)
{
	int ret = alvium_write(sensor, CCI_CHANGE_MODE_8W, AV_CAM_REG_SIZE,
			       AV_CAM_DATA_SIZE_8, (u8 *)&mode);
	if (ret < 0) {
		dev_err(sensor->dev, "Failed to set mode: %d (%d)", mode, ret);
		return ret;
	}
	sensor->mode = mode;
	return 0;
}

/* Read image format from camera,
 * should be only called once, during initialization
 * */
static int alvium_read_fmt(struct alvium *sensor, uint32_t *fmt)
{
	u32 avt_img_fmt = 0;
	u8 bayer_pattern;
	int ret = 0;

	ret = alvium_read(
		sensor, sensor->cci_reg.bcrm_addr + BCRM_IMG_BAYER_PATTERN_8RW,
		AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&bayer_pattern);
	if (ret < 0) {
		dev_err(sensor->dev,
			"BCRM_IMG_BAYER_PATTERN_8RW read failed: %d\n", ret);
		return ret;
	}
	// alvium_dbg(sensor->dev, "BCRM_IMG_BAYER_PATTERN_8RW=0x%X",
	// 	   bayer_pattern);

	ret = alvium_read(
		sensor,
		sensor->cci_reg.bcrm_addr + BCRM_IMG_MIPI_DATA_FORMAT_32RW,
		AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&avt_img_fmt);
	if (ret < 0) {
		dev_err(sensor->dev,
			"BCRM_IMG_MIPI_DATA_FORMAT_32RW read failed: %d\n",
			ret);
		return ret;
	}
	// alvium_dbg(sensor->dev, "BCRM_IMG_MIPI_DATA_FORMAT_32RW=0x%08X\n",
	// 	   avt_img_fmt);

	switch (avt_img_fmt) {
	case MIPI_DT_RGB888:
		avt_img_fmt = MEDIA_BUS_FMT_RGB888_1X24;
		break;
	case MIPI_DT_RGB565:
		avt_img_fmt = MEDIA_BUS_FMT_RGB565_1X16;
		break;
	case MIPI_DT_YUV422:
		avt_img_fmt = MEDIA_BUS_FMT_UYVY8_2X8;
		break;
	case MIPI_DT_CUSTOM:
		avt_img_fmt = MEDIA_BUS_FMT_CUSTOM;
		break;
	case MIPI_DT_RAW8:
		switch (bayer_pattern) {
		case monochrome:
			avt_img_fmt = MEDIA_BUS_FMT_Y8_1X8;
			break;
		case bayer_gr:
			avt_img_fmt = MEDIA_BUS_FMT_SGRBG8_1X8;
			break;
		case bayer_rg:
			avt_img_fmt = MEDIA_BUS_FMT_SRGGB8_1X8;
			break;
		case bayer_gb:
			avt_img_fmt = MEDIA_BUS_FMT_SGBRG8_1X8;
			break;
		case bayer_bg:
			avt_img_fmt = MEDIA_BUS_FMT_SBGGR8_1X8;
			break;
		default:
			dev_err(sensor->dev,
				"Unknown RAW8 pixelformat read, bayer_pattern %d\n",
				bayer_pattern);
			return -EINVAL;
		}
		break;
	case MIPI_DT_RAW10:
		switch (bayer_pattern) {
		case monochrome:
			avt_img_fmt = MEDIA_BUS_FMT_Y10_1X10;
			break;
		case bayer_gr:
			avt_img_fmt = MEDIA_BUS_FMT_SGRBG10_1X10;
			break;
		case bayer_rg:
			avt_img_fmt = MEDIA_BUS_FMT_SRGGB10_1X10;
			break;
		case bayer_gb:
			avt_img_fmt = MEDIA_BUS_FMT_SGBRG10_1X10;
			break;
		case bayer_bg:
			avt_img_fmt = MEDIA_BUS_FMT_SBGGR10_1X10;
			break;
		default:
			dev_err(sensor->dev,
				"Unknown RAW10 pixelformat read, bayer_pattern %d\n",
				bayer_pattern);
			return -EINVAL;
		}
		break;
	case MIPI_DT_RAW12:
		switch (bayer_pattern) {
		case monochrome:
			avt_img_fmt = MEDIA_BUS_FMT_Y12_1X12;
			break;
		case bayer_gr:
			avt_img_fmt = MEDIA_BUS_FMT_SGRBG12_1X12;
			break;
		case bayer_rg:
			avt_img_fmt = MEDIA_BUS_FMT_SRGGB12_1X12;
			break;
		case bayer_gb:
			avt_img_fmt = MEDIA_BUS_FMT_SGBRG12_1X12;
			break;
		case bayer_bg:
			avt_img_fmt = MEDIA_BUS_FMT_SBGGR12_1X12;
			break;
		default:
			dev_err(sensor->dev,
				"Unknown RAW12 pixelformat read, bayer_pattern %d\n",
				bayer_pattern);
			return -EINVAL;
		}
		break;

	case 0:
		/* Pixelformat 0 -> Probably fallback app running -> Emulate RAW888 */
		avt_img_fmt = MEDIA_BUS_FMT_RGB888_1X24;
		dev_warn(
			sensor->dev,
			"Invalid pixelformat detected (0). Fallback app running?");
		break;

	default:
		dev_err(sensor->dev,
			"Unknown pixelformat read, avt_img_fmt 0x%x\n",
			avt_img_fmt);
		return -EINVAL;
	}

	*fmt = avt_img_fmt;

	return 0;
}

static int alvium_init_mode(struct alvium *sensor)
{
	int ret = 0;
	u32 common_min_clk = 0;
	u32 common_max_clk = 0;
	u32 common_inc_clk = 0;
	u32 device_csi_clk_min = 0;
	u32 device_csi_clk_max = 0;
	u32 host_max_clk = 0;
	u8 device_supported_lane_counts = 0;
	u8 value8 = 0;

	/* Check if requested number of lanes is supported */
	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr +
				  BCRM_SUPPORTED_CSI2_LANE_COUNTS_8R,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8,
			  (char *)&device_supported_lane_counts);
	if (ret < 0) {
		dev_err(sensor->dev, "supported csi lane count read failed: %d",
			ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "supported lane count. camera: %d, soc: %d",
		   (u32)device_supported_lane_counts, sensor->minfo.num_lanes);

	if (!(test_bit(sensor->minfo.num_lanes - 1,
		       (const long *)(&device_supported_lane_counts)))) {
		dev_err(sensor->dev,
			"requested number of lanes (%u) not supported by this camera!",
			sensor->minfo.num_lanes);
		return -EINVAL;
	}

	/* Set number of lanes */
	value8 = (u8)sensor->minfo.num_lanes;
	ret = alvium_write(sensor,
			   sensor->cci_reg.bcrm_addr + BCRM_CSI2_LANE_COUNT_8RW,
			   AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, &value8);
	if (ret < 0) {
		dev_err(sensor->dev, "csi lane count write failed: %d", ret);
		return ret;
	}

	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr + BCRM_CSI2_CLOCK_MIN_32R,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32,
			  (char *)&device_csi_clk_min);
	if (ret < 0) {
		dev_err(sensor->dev, "csi clk min read failed: %d", ret);
		return ret;
	}
	// alvium_dbg(sensor->dev, "csi clk min: %u", device_csi_clk_min);

	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr + BCRM_CSI2_CLOCK_MAX_32R,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32,
			  (char *)&device_csi_clk_max);
	if (ret < 0) {
		dev_err(sensor->dev, "csi clk max read failed: %d", ret);
		return ret;
	}
	// alvium_dbg(sensor->dev, "csi clk max: %u", device_csi_clk_max);

	if (sensor->minfo.num_lanes == 4) {
		// TODO verify, if needed
		host_max_clk = CSI_HOST_CLK_MAX_FREQ_4L;
	} else {
		host_max_clk = CSI_HOST_CLK_MAX_FREQ;
	}

	alvium_dbg(sensor->dev,
		   "csi clock camera range: %d:%d Hz, host range: %d:%d Hz",
		   device_csi_clk_min, device_csi_clk_max,
		   CSI_HOST_CLK_MIN_FREQ, host_max_clk);

	if (common_range(device_csi_clk_min, device_csi_clk_max, 1,
			 CSI_HOST_CLK_MIN_FREQ, host_max_clk, 1,
			 &common_min_clk, &common_max_clk,
			 &common_inc_clk) == false) {
		dev_err(sensor->dev,
			"no common clock range for camera and host possible!");
		return -EINVAL;
	}

	alvium_dbg(sensor->dev, "camera/host common csi clock range: %d:%d Hz",
		   common_min_clk, common_max_clk);

	if (sensor->minfo.clk_freq == 0) {
		alvium_dbg(sensor->dev,
			   "no csi clock requested, using common max %d Hz",
			   common_max_clk);
		sensor->minfo.clk_freq = common_max_clk;
	} else {
		alvium_dbg(sensor->dev, "using csi clock from dts: %u Hz",
			   sensor->minfo.clk_freq);
	}

	if ((sensor->minfo.clk_freq < common_min_clk) ||
	    (sensor->minfo.clk_freq > common_max_clk)) {
		dev_err(sensor->dev,
			"unsupported csi clock frequency (%d Hz, range: %d:%d Hz)!",
			sensor->minfo.clk_freq, common_min_clk, common_max_clk);
		return -EINVAL;
	}

	/* write clk frequency */
	ret = alvium_write(sensor,
			   sensor->cci_reg.bcrm_addr + BCRM_CSI2_CLOCK_32RW,
			   AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32,
			   (u8 *)&sensor->minfo.clk_freq);
	if (ret < 0) {
		dev_err(sensor->dev, "csi clk write failed: %d", ret);
		return ret;
	}

	/* read clk frequency back */
	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr + BCRM_CSI2_CLOCK_32RW,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32,
			  (char *)&sensor->minfo.clk_freq);
	if (ret < 0) {
		dev_err(sensor->dev, "csi clk read failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "csi clock read from camera: %d Hz",
		   sensor->minfo.clk_freq);

	if (sensor->link_freq_ctrl != NULL) {
		ret = v4l2_ctrl_s_ctrl(sensor->link_freq_ctrl,
				       sensor->minfo.clk_freq);
		if (ret) {
			dev_err(sensor->dev, "Failed to set link freq ctrl");
		}
	}

	ret = alvium_read_fmt(sensor, &(sensor->minfo.fmt_code));
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_read_fmt failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "format set to: 0x%x", sensor->minfo.fmt_code);

	// set BCRM mode
	ret = alvium_set_mode(sensor, ALVIUM_BCRM_MODE);
	if (ret < 0) {
		dev_err(sensor->dev, "sensor mode set failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "sensor mode set to BCRM");

	return ret;
}

static bool device_present(struct alvium *sensor)
{
	int ret = 0;
	u64 data = 0;

	ret = alvium_read(sensor, CCI_DEVICE_CAP_64R, AV_CAM_REG_SIZE,
			  AV_CAM_DATA_SIZE_64, (char *)&data);
	if (ret < 0) {
		dev_err(sensor->dev, "i2c read failed: %d", ret);
		return ret;
	}

	return ((ret < 0) || (data == 0)) ? false : true;
}

static int soft_reset(struct alvium *sensor)
{
	int ret = 0;
	uint8_t reset_val = 1;
	static const uint8_t default_heartbeat_val = 0x80;
	uint8_t heartbeat_val = default_heartbeat_val;
	uint64_t duration_ms = 0;
	static const uint8_t heartbeat_low_limit = 0;
	static const uint32_t delay_ms = 400;
	static const uint32_t max_time_ms = 10000;
	uint64_t start_jiffies = get_jiffies_64();
	bool device_available = false;
	bool heartbeat_available = false;

	/* Check, if heartbeat register is available (write default value and read it back)*/
	ret = alvium_write(sensor, CCI_HEARTBEAT_8RW, AV_CAM_REG_SIZE,
			   AV_CAM_DATA_SIZE_8, (char *)&heartbeat_val);
	heartbeat_available =
		(alvium_read(sensor, CCI_HEARTBEAT_8RW, AV_CAM_REG_SIZE,
			     AV_CAM_DATA_SIZE_8, (char *)&heartbeat_val) < 0) ?
			false :
			true;
	/* If camera does not support heartbeat it delivers always 0 */
	heartbeat_available = ((heartbeat_val != 0) && (ret != 0)) ? true :
								     false;
	alvium_dbg(sensor->dev, "Heartbeat %ssupported",
		   (heartbeat_available) ? "" : "NOT ");

	/* Execute soft reset */
	ret = alvium_write(sensor, CCI_SOFT_RESET_8W, AV_CAM_REG_SIZE,
			   AV_CAM_DATA_SIZE_8, (char *)&reset_val);

	if (ret >= 0) {
		alvium_dbg(sensor->dev,
			   "Soft reset executed. Initializing camera...");
	} else {
		dev_err(sensor->dev, "Soft reset ERROR");
		return -EIO;
	}

	/* Poll camera register to check if camera is back again */
	do {
		usleep_range(delay_ms * 1000, (delay_ms * 1000) + 1);
		device_available = device_present(sensor);
		duration_ms =
			jiffies_to_msecs(get_jiffies_64() - start_jiffies);
	} while ((duration_ms < max_time_ms) && !device_available);

	if (!heartbeat_available) {
		/* Camera might need a few more seconds to be fully booted */
		usleep_range(add_wait_time_ms * 1000,
			     (add_wait_time_ms * 1000) + 1);
	} else {
		/* Heartbeat is supported. Poll heartbeat register until value is lower than the default value again */
		do {
			usleep_range(delay_ms * 1000, (delay_ms * 1000) + 1);
			ret = alvium_read(sensor, CCI_HEARTBEAT_8RW,
					  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8,
					  (char *)&heartbeat_val);
			// alvium_dbg(sensor->dev, "Heartbeat val=0x%02X", heartbeat_val);
			duration_ms = jiffies_to_msecs(get_jiffies_64() -
						       start_jiffies);
			if ((heartbeat_val > heartbeat_low_limit) &&
			    (heartbeat_val < default_heartbeat_val) &&
			    (ret >= 0)) {
				/* Heartbeat active -> Camera alive */
				alvium_dbg(sensor->dev, "Heartbeat active!");
				break;
			}
		} while (duration_ms < max_time_ms);
	}

	alvium_dbg(sensor->dev, "Camera boot time: %llums", duration_ms);
	if (!device_available)
		dev_err(sensor->dev, "Camera not reconnected");

	return 0;
}

// reset and reinit
static int alvium_reset(struct alvium *sensor)
{
	int ret = 0;
	struct v4l2_subdev_format format;
	struct v4l2_subdev_frame_interval interval;

	sensor->is_open = false;
	sensor->is_streaming = false;

	// reset camera
	ret = soft_reset(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "reset failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "resetted");

	// init MIPI clk
	ret = alvium_init_mode(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "set format failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "mode init");

	// sets frame format (and frame size internally)
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.code = ALVIUM_DEFAULT_FMT;
	format.format.width = sensor->crop.width;
	format.format.height = sensor->crop.height;
	ret = alvium_set_fmt(&sensor->subdev, NULL, &format);
	if (ret < 0) {
		dev_err(sensor->dev, "set format failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "format set");

	// set framerate
	interval.pad = 0;
	interval.interval.numerator = 1;
	interval.interval.denominator = ALVIUM_DEFAULT_FPS;
	ret = alvium_s_frame_interval(&sensor->subdev, &interval);
	if (ret < 0) {
		dev_err(sensor->dev, "set frame interval failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "framerate set");

	return 0;
}

static int alvium_stream_on(struct alvium *sensor)
{
	int ret = 0;
	// u8 trigger_mode;

	alvium_dbg(sensor->dev, "");

	// ret = alvium_write_param(sensor, V4L2_AV_CSI2_PHY_RESET_W, 1);
	// if (ret < 0) {
	// 	dev_err(sensor->dev, "phy reset failed: %d", ret);
	// 	return ret;
	// }

	// read trigger mode, however is this really needed?
	// ret = alvium_read(
	// 	sensor,
	// 	sensor->cci_reg.bcrm_addr + BCRM_FRAME_START_TRIGGER_MODE_8RW,
	// 	AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&trigger_mode);
	// if (ret < 0) {
	// 	dev_err(sensor->dev, "trigger mode read error: %d", ret);
	// 	return ret;
	// }
	// alvium_dbg(sensor->dev, "trigger mode: 0x%x", trigger_mode);

	// config MIPI again (which internally disables and enabled the PHY)
	// ret = alvium_init_mode(sensor);
	// if (ret < 0) {
	// 	dev_err(sensor->dev, "init mipi failed: %d", ret);
	// 	return ret;
	// }

	// if (sensor->mode != ALVIUM_BCRM_MODE) {
	// 	dev_err(sensor->dev, "sensor not in bcrm mode!!");
	// 	// TODO handle this
	// }

	// not sure, if we really need to wait here
	// msleep(5);

	// ret = alvium_write_param(sensor, V4L2_AV_CSI2_PHY_RESET_W, 0);
	// if (ret < 0) {
	// 	dev_err(sensor->dev, "phy reset failed: %d", ret);
	// 	return ret;
	// }

	// msleep(1);

	alvium_dbg(sensor->dev, "resetting");
	ret = alvium_reset(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium reset failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "reset done");

	ret = alvium_write_param(sensor, V4L2_AV_CSI2_STREAMON_W, 1);
	if (ret < 0) {
		dev_err(sensor->dev, "stream on failed: %d", ret);
		return ret;
	}

	// TODO TODO is this the right order with setting frame size etc before?

	sensor->is_streaming = true;

	return ret;
}

static int alvium_stream_off(struct alvium *sensor)
{
	int ret = 0;

	alvium_dbg(sensor->dev, "");

	if (sensor->mode == ALVIUM_BCRM_MODE) {
		ret = alvium_write_param(sensor, V4L2_AV_CSI2_STREAMOFF_W, 1);
		if (ret < 0) {
			dev_err(sensor->dev, "stream off failed: %d", ret);
			return ret;
		}

		ret = alvium_write_param(sensor, V4L2_AV_CSI2_PHY_RESET_W, 1);
		if (ret < 0) {
			dev_err(sensor->dev, "phy reset failed: %d", ret);
			return ret;
		}
	}

	sensor->is_streaming = false;

	// // reset and reinit camera
	// // with next stream on, clock lp11 change is expected and reached only from full reset
	// // TODO turn stream only off, when 0 listeners are registered?
	// ret = alvium_reset(sensor);
	// if (ret < 0) {
	// 	dev_err(sensor->dev, "alvium reset failed: %d", ret);
	// 	return ret;
	// }

	return ret;
}

/* V4L2 subdev video ops */
static int alvium_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct alvium *sensor = to_alvium(sd);
	int ret = 0;

	alvium_dbg(sensor->dev, "enable: %d", enable);

	mutex_lock(&sensor->lock);

	alvium_dbg(sensor->dev, "Mutex acquired");

	if (enable && sensor->is_streaming) {
		dev_err(sensor->dev, "enable and streaming");
		// ret = -EBUSY;
		// goto out;	// TODO check if ok
	}

	if (!enable && !sensor->is_streaming) {
		dev_err(sensor->dev, "not enable and not streaming");
		// goto out;	// TODO check if ok
	}

	if (enable)
		ret = alvium_stream_on(sensor);
	else
		ret = alvium_stream_off(sensor);

	// out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int alvium_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct alvium *sensor = to_alvium(sd);
	u64 framerate;
	int ret;

	alvium_dbg(sensor->dev, "pad: %d", interval->pad);

	if (interval->pad != 0)
		return -EINVAL;

	// mutex_lock(&sensor->lock);

	/* TODO: Calculate current frame interval here or in set_selection to
	 * always have the correct frame interval.
	 */
	interval->interval = sensor->interval;
	alvium_dbg(sensor->dev, "saved: %d / %d", interval->interval.numerator,
		   interval->interval.denominator);

	ret = alvium_read(
		sensor,
		sensor->cci_reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_64RW,
		AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&framerate);
	if (ret < 0) {
		dev_err(sensor->dev, "i2c read failed: %d", ret);
		return ret;
	}

	/* Translate frequency to timeperframe
	 * by inverting the fraction
	 */
	interval->interval.numerator = FRAQ_NUM;
	interval->interval.denominator = (framerate * FRAQ_NUM) / UHZ_TO_HZ;

	alvium_dbg(sensor->dev, "reported: %d / %d",
		   interval->interval.numerator,
		   interval->interval.denominator);

	// TODO TODO remove this!!!
	interval->interval.numerator = 1;
	interval->interval.denominator = ALVIUM_DEFAULT_FPS;
	// TODO TODO end remove

	// mutex_unlock(&sensor->lock);
	return 0;
}

// static struct v4l2_rect *alvium_get_pad_crop(struct alvium *sensor,
// 					     struct v4l2_subdev_pad_config *cfg,
// 					     unsigned int pad, u32 which)
// {
// 	alvium_dbg(sensor->dev, "%s: pad: %d, which: %d",  pad, which);

// 	switch (which) {
// 	case V4L2_SUBDEV_FORMAT_TRY:
// 		return v4l2_subdev_get_try_crop(&sensor->subdev, cfg, pad);
// 	case V4L2_SUBDEV_FORMAT_ACTIVE:
// 		return &sensor->crop;
// 	default:
// 		return NULL;
// 	}
// }

// static struct v4l2_mbus_framefmt *
// alvium_get_pad_fmt(struct alvium *sensor, struct v4l2_subdev_pad_config *cfg,
// 		   unsigned int pad, u32 which)
// {
// 	alvium_dbg(sensor->dev, "%s: pad: %d, which: %d",  pad, which);

// 	switch (which) {
// 	case V4L2_SUBDEV_FORMAT_TRY:
// 		return v4l2_subdev_get_try_format(&sensor->subdev, cfg, pad);
// 	case V4L2_SUBDEV_FORMAT_ACTIVE:
// 		return &sensor->fmt;
// 	default:
// 		return NULL;
// 	}
// }

static int alvium_check_fmt_available(struct alvium *sensor, u32 media_bus_fmt)
{
	union bcrm_avail_mipi_reg feature_inquiry_reg;
	union bcrm_bayer_inquiry_reg bayer_inquiry_reg;
	int ret;

	/* read the MIPI format register to check whether the camera
	 * really support the requested pixelformat format
	 */
	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr +
				  BCRM_IMG_AVAILABLE_MIPI_DATA_FORMATS_64R,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
			  (char *)&feature_inquiry_reg);
	if (ret < 0) {
		dev_err(sensor->dev, "i2c read failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "feature: 0x%016llx",
		   feature_inquiry_reg.value);

	/* read the Bayer Inquiry register to check whether the camera
	 * really support the requested RAW format
	 */
	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr +
				  BCRM_IMG_BAYER_PATTERN_INQUIRY_8R,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8,
			  (char *)&bayer_inquiry_reg);
	if (ret < 0) {
		dev_err(sensor->dev, "i2c read failed: %d", ret);
		return ret;
	}
	alvium_dbg(sensor->dev, "bayer: %02x", bayer_inquiry_reg.value);

	switch (media_bus_fmt) {
	case MEDIA_BUS_FMT_RGB444_1X12:
		return feature_inquiry_reg.avail_mipi.rgb444_avail;
	case MEDIA_BUS_FMT_RGB565_1X16:
		return feature_inquiry_reg.avail_mipi.rgb565_avail;
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_BGR888_1X24:
		return feature_inquiry_reg.avail_mipi.rgb888_avail;
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_2X8:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_VYUY8_2X8:
	case MEDIA_BUS_FMT_VYUY8_1X16:
		return feature_inquiry_reg.avail_mipi.yuv422_8_avail;
	/* RAW 8 */
	case MEDIA_BUS_FMT_Y8_1X8:
		return feature_inquiry_reg.avail_mipi.raw8_avail &&
		       bayer_inquiry_reg.bayer_pattern.monochrome_avail;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
		return feature_inquiry_reg.avail_mipi.raw8_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_BG_avail;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
		return feature_inquiry_reg.avail_mipi.raw8_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_GB_avail;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		return feature_inquiry_reg.avail_mipi.raw8_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_GR_avail;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return feature_inquiry_reg.avail_mipi.raw8_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_RG_avail;
	/* RAW 10 */
	case MEDIA_BUS_FMT_Y10_1X10:
		return feature_inquiry_reg.avail_mipi.raw10_avail &&
		       bayer_inquiry_reg.bayer_pattern.monochrome_avail;
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		return feature_inquiry_reg.avail_mipi.raw10_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_GB_avail;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		return feature_inquiry_reg.avail_mipi.raw10_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_GR_avail;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return feature_inquiry_reg.avail_mipi.raw10_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_RG_avail;
	/* RAW 12 */
	case MEDIA_BUS_FMT_Y12_1X12:
		return feature_inquiry_reg.avail_mipi.raw12_avail &&
		       bayer_inquiry_reg.bayer_pattern.monochrome_avail;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		return feature_inquiry_reg.avail_mipi.raw12_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_BG_avail;
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		return feature_inquiry_reg.avail_mipi.raw12_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_GB_avail;
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		return feature_inquiry_reg.avail_mipi.raw12_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_GR_avail;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return feature_inquiry_reg.avail_mipi.raw12_avail &&
		       bayer_inquiry_reg.bayer_pattern.bayer_RG_avail;
	default:
		dev_warn(sensor->dev, "unhandled mbus fmt: %x", media_bus_fmt);
	}

	return -EINVAL;
}

static int alvium_init_avail_formats(struct alvium *sensor)
{
	int fmt_iter = 0;
	int ret;
	int i;

	sensor->fmts = kmalloc(
		sizeof(unsigned int) * ARRAY_SIZE(alvium_formats), GFP_KERNEL);
	for (i = 0; i < ARRAY_SIZE(alvium_formats); i++) {
		ret = alvium_check_fmt_available(sensor, alvium_formats[i]);
		if (ret < 0) {
			dev_err(sensor->dev, "cannot check fmt 0x%x. ret: %d",
				alvium_formats[i], ret);
			return ret;
		}
		if (ret > 0) {
			sensor->fmts[fmt_iter++] = alvium_formats[i];
			alvium_dbg(sensor->dev, "fmt 0x%x available",
				   alvium_formats[i]);
		}
	}
	sensor->num_fmts = fmt_iter; // TODO TODO check, if we need to add  + 1;

	return 0;
}

static int alvium_read_param(struct alvium *sensor, u32 id, u32 *val)
{
	int ret = 0;
	u32 reg = 0, length = 0;
	int gencp_mode_local = 0; /* Default BCRM mode */

	switch (id) {
	case V4L2_AV_CSI2_WIDTH_R:
		reg = BCRM_IMG_WIDTH_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_WIDTH_MINVAL_R:
		reg = BCRM_IMG_WIDTH_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_WIDTH_MAXVAL_R:
		reg = BCRM_IMG_WIDTH_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_WIDTH_INCVAL_R:
		reg = BCRM_IMG_WIDTH_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HEIGHT_R:
		reg = BCRM_IMG_HEIGHT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HEIGHT_MINVAL_R:
		reg = BCRM_IMG_HEIGHT_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HEIGHT_MAXVAL_R:
		reg = BCRM_IMG_HEIGHT_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HEIGHT_INCVAL_R:
		reg = BCRM_IMG_HEIGHT_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_X_R:
		reg = BCRM_IMG_OFFSET_X_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_X_MIN_R:
		reg = BCRM_IMG_OFFSET_X_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_X_MAX_R:
		reg = BCRM_IMG_OFFSET_X_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_X_INC_R:
		reg = BCRM_IMG_OFFSET_X_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_R:
		reg = BCRM_IMG_OFFSET_Y_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_MIN_R:
		reg = BCRM_IMG_OFFSET_Y_MIN_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_MAX_R:
		reg = BCRM_IMG_OFFSET_Y_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_INC_R:
		reg = BCRM_IMG_OFFSET_Y_INC_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_SENSOR_WIDTH_R:
		reg = BCRM_SENSOR_WIDTH_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_SENSOR_HEIGHT_R:
		reg = BCRM_SENSOR_HEIGHT_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_MAX_WIDTH_R:
		reg = BCRM_WIDTH_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_MAX_HEIGHT_R:
		reg = BCRM_HEIGHT_MAX_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_PIXELFORMAT_R:
		reg = BCRM_IMG_MIPI_DATA_FORMAT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_PALYLOADSIZE_R:
		reg = BCRM_BUFFER_SIZE_32R;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_ACQ_STATUS_R:
		reg = BCRM_ACQUISITION_STATUS_8R;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_HFLIP_R:
		reg = BCRM_IMG_REVERSE_X_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_VFLIP_R:
		reg = BCRM_IMG_REVERSE_Y_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_CURRENTMODE_R:
		reg = CCI_CURRENT_MODE_8R;
		length = AV_CAM_DATA_SIZE_8;
		gencp_mode_local = 1;
		break;
	case V4L2_AV_CSI2_FRAMERATE_R:
		reg = BCRM_ACQUISITION_FRAME_RATE_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;
	case V4L2_AV_CSI2_FRAMERATE_MINVAL_R:
		reg = BCRM_ACQUISITION_FRAME_RATE_MIN_64R;
		length = AV_CAM_DATA_SIZE_64;
		break;
	case V4L2_AV_CSI2_FRAMERATE_MAXVAL_R:
		reg = BCRM_ACQUISITION_FRAME_RATE_MAX_64R;
		length = AV_CAM_DATA_SIZE_64;
		break;
	case V4L2_AV_CSI2_FRAMERATE_INCVAL_R:
		reg = BCRM_ACQUISITION_FRAME_RATE_INC_64R;
		length = AV_CAM_DATA_SIZE_64;
		break;
	default:
		dev_err(sensor->dev, "unknown ctrl 0x%x\n", id);
		return -EINVAL;
	}

	if (gencp_mode_local) {
		ret = alvium_read(sensor, reg, AV_CAM_REG_SIZE, length,
				  (char *)val);
		if (ret < 0) {
			dev_err(sensor->dev, "i2c read failed: %d", ret);
			return ret;
		}
		return 0;
	}

	ret = alvium_read(sensor, sensor->cci_reg.bcrm_addr + reg,
			  AV_CAM_REG_SIZE, length, (char *)val);
	if (ret < 0) {
		dev_err(sensor->dev, "i2c read failed: %d", ret);
		return ret;
	}
	// alvium_dbg(sensor->dev, "reg: 0x%x, value: 0x%x", reg, *val);

	if (id == V4L2_AV_CSI2_PIXELFORMAT_R) {
		// To avoid ambiguity, resulting from two MBUS formats linked with one camera image format, we return value stored in private data
		*val = sensor->minfo.fmt_code;
		// alvium_dbg(sensor->dev, "value: 0x%x", *val);
	}

	return 0;
}

static int alvium_write_param(struct alvium *sensor, u32 id, u32 val)
{
	int ret = 0;
	u32 reg = 0, length = 0;
	int gencp_mode_local = 0; /* Default BCRM mode */
	u8 bayer_temp = 0;

	switch (id) {
	case V4L2_AV_CSI2_STREAMON_W:
		reg = BCRM_ACQUISITION_START_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_STREAMOFF_W:
		reg = BCRM_ACQUISITION_STOP_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_PHY_RESET_W:
		reg = BCRM_PHY_RESET_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_ABORT_W:
		reg = BCRM_ACQUISITION_ABORT_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_WIDTH_W:
		reg = BCRM_IMG_WIDTH_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HEIGHT_W:
		reg = BCRM_IMG_HEIGHT_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_X_W:
		reg = BCRM_IMG_OFFSET_X_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_OFFSET_Y_W:
		reg = BCRM_IMG_OFFSET_Y_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;
	case V4L2_AV_CSI2_HFLIP_W:
		reg = BCRM_IMG_REVERSE_X_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_VFLIP_W:
		reg = BCRM_IMG_REVERSE_Y_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;
	case V4L2_AV_CSI2_PIXELFORMAT_W:
		reg = BCRM_IMG_MIPI_DATA_FORMAT_32RW;
		length = AV_CAM_DATA_SIZE_32;

		if (alvium_check_fmt_available(sensor, val) <= 0) {
			dev_err(sensor->dev, "format 0x%x not supported", val);
			return -EINVAL;
		}

		switch (val) {
		case MEDIA_BUS_FMT_RGB444_1X12:
			val = MIPI_DT_RGB444;
			break;
		case MEDIA_BUS_FMT_RGB565_1X16:
			val = MIPI_DT_RGB565;
			break;
		case MEDIA_BUS_FMT_RGB888_1X24:
		case MEDIA_BUS_FMT_BGR888_1X24:
			val = MIPI_DT_RGB888;
			break;
		case MEDIA_BUS_FMT_UYVY8_2X8:
		case MEDIA_BUS_FMT_UYVY8_1X16:
		case MEDIA_BUS_FMT_YUYV8_2X8:
		case MEDIA_BUS_FMT_YUYV8_1X16:
		case MEDIA_BUS_FMT_VYUY8_2X8:
		case MEDIA_BUS_FMT_VYUY8_1X16:
			val = MIPI_DT_YUV422;
			break;
		/* RAW 8 */
		case MEDIA_BUS_FMT_Y8_1X8:
			val = MIPI_DT_RAW8;
			bayer_temp = monochrome;
			break;
		case MEDIA_BUS_FMT_SBGGR8_1X8:
			val = MIPI_DT_RAW8;
			bayer_temp = bayer_bg;
			break;
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			val = MIPI_DT_RAW8;
			bayer_temp = bayer_gb;
			break;
		case MEDIA_BUS_FMT_SGRBG8_1X8:
			val = MIPI_DT_RAW8;
			bayer_temp = bayer_gr;
			break;
		case MEDIA_BUS_FMT_SRGGB8_1X8:
			val = MIPI_DT_RAW8;
			bayer_temp = bayer_rg;
			break;
		/* RAW 10 */
		case MEDIA_BUS_FMT_Y10_1X10:
			val = MIPI_DT_RAW10;
			bayer_temp = monochrome;
			break;
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			val = MIPI_DT_RAW10;
			bayer_temp = bayer_gb;
			break;
		case MEDIA_BUS_FMT_SGRBG10_1X10:
			val = MIPI_DT_RAW10;
			bayer_temp = bayer_gr;
			break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
			val = MIPI_DT_RAW10;
			bayer_temp = bayer_rg;
			break;
		/* RAW 12 */
		case MEDIA_BUS_FMT_Y12_1X12:
			val = MIPI_DT_RAW12;
			bayer_temp = monochrome;
			break;
		case MEDIA_BUS_FMT_SBGGR12_1X12:
			val = MIPI_DT_RAW12;
			bayer_temp = bayer_bg;
			break;
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			val = MIPI_DT_RAW12;
			bayer_temp = bayer_gb;
			break;
		case MEDIA_BUS_FMT_SGRBG12_1X12:
			val = MIPI_DT_RAW12;
			bayer_temp = bayer_gr;
			break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
			val = MIPI_DT_RAW12;
			bayer_temp = bayer_rg;
			break;

		default:
			dev_err(sensor->dev,
				"format 0x%x not supported by the host", val);
			return -EINVAL;
		}
		break;
	case V4L2_AV_CSI2_CHANGEMODE_W:
		reg = CCI_CHANGE_MODE_8W;
		length = AV_CAM_DATA_SIZE_8;
		gencp_mode_local = 1;
		if (val == 1)
			sensor->mode = ALVIUM_GENCP_MODE;
		else
			sensor->mode = ALVIUM_BCRM_MODE;
		break;
	default:
		dev_err(sensor->dev, "unknown ctrl 0x%x", id);
		return -EINVAL;
	}

	// alvium_dbg(sensor->dev, "reg %x, length %d, val 0x%x", reg, length,
	// 	   val);

	if (gencp_mode_local) {
		ret = alvium_write(sensor, reg, AV_CAM_REG_SIZE, length,
				   (char *)&val);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_write failed: %d", ret);
			return ret;
		} else {
			return 0;
		}
	}

	if (id == V4L2_AV_CSI2_PIXELFORMAT_W) {
		/* Set pixelformat then set bayer format, refer OCT-2417
		 *
		 * XXX implement these somehow, below imx6 code:
		 * mipi_csi2_info = mipi_csi2_get_info();
		 * mipi_csi2_set_datatype(mipi_csi2_info, val);
		 */
	}

	ret = alvium_write(sensor, sensor->cci_reg.bcrm_addr + reg,
			   AV_CAM_REG_SIZE, length, (char *)&val);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_write failed: %d", ret);
		return ret;
	}

	/* Set pixelformat then set bayer format, refer OCT-2417 */
	if (id == V4L2_AV_CSI2_PIXELFORMAT_W) {
		ret = alvium_write(sensor,
				   sensor->cci_reg.bcrm_addr +
					   BCRM_IMG_BAYER_PATTERN_8RW,
				   AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8,
				   (char *)&bayer_temp);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_write failed: %d", ret);
			return ret;
		}
	}

	return 0;
}

static void dump_frame_sizes(struct alvium *sensor)
{
	alvium_dbg(sensor->dev, "width = %d, (min/max/inc) = (%d, %d, %d)\n",
		   sensor->crop.width, sensor->limits.w.min,
		   sensor->limits.w.max, sensor->limits.w.step);
	alvium_dbg(sensor->dev, "height = %d, (min/max/inc) = (%d, %d, %d)\n",
		   sensor->crop.height, sensor->limits.h.min,
		   sensor->limits.h.max, sensor->limits.h.step);
	alvium_dbg(sensor->dev, "woff = %d, (min/max/inc) = (%d, %d, %d)\n",
		   sensor->crop.left, sensor->limits.woff.min,
		   sensor->limits.woff.max, sensor->limits.woff.step);
	alvium_dbg(sensor->dev, "hoff = %d, (min/max/inc) = (%d, %d, %d)\n",
		   sensor->crop.top, sensor->limits.hoff.min,
		   sensor->limits.hoff.max, sensor->limits.hoff.step);
}

static int alvium_init_frame_sizes(struct alvium *sensor)
{
	int ret = 0;

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_HEIGHT_MINVAL_R,
				&(sensor->limits.h.min));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_HEIGHT_MINVAL_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_HEIGHT_MAXVAL_R,
				&(sensor->limits.h.max));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_HEIGHT_MAXVAL_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_HEIGHT_INCVAL_R,
				&(sensor->limits.h.step));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_HEIGHT_INCVAL_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_WIDTH_MINVAL_R,
				&(sensor->limits.w.min));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_WIDTH_MINVAL_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_WIDTH_MAXVAL_R,
				&(sensor->limits.w.max));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_WIDTH_MAXVAL_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_WIDTH_INCVAL_R,
				&(sensor->limits.w.step));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_WIDTH_INCVAL_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_OFFSET_Y_MIN_R,
				&(sensor->limits.hoff.min));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_OFFSET_Y_MIN_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_OFFSET_Y_MAX_R,
				&(sensor->limits.hoff.max));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_OFFSET_Y_MAX_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_OFFSET_Y_INC_R,
				&(sensor->limits.hoff.step));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_OFFSET_Y_INC_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_OFFSET_X_MIN_R,
				&(sensor->limits.woff.min));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_OFFSET_X_MIN_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_OFFSET_X_MAX_R,
				&(sensor->limits.woff.max));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_OFFSET_X_MAX_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_OFFSET_X_INC_R,
				&(sensor->limits.woff.step));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_OFFSET_X_INC_R err: %d",
			ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_WIDTH_R,
				&(sensor->crop.width));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_WIDTH_R err: %d", ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_HEIGHT_R,
				&(sensor->crop.height));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_HEIGHT_R err: %d", ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_OFFSET_X_R,
				&(sensor->crop.left));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_OFFSET_X_R err: %d", ret);
		return ret;
	}

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_OFFSET_Y_R,
				&(sensor->crop.top));
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_OFFSET_Y_R err: %d", ret);
		return ret;
	}

	/* We might need to correct some values */
	// /* Tegra doesn't seem to accept offsets that are not divisible by 8. */
	// roundup(sensor->limits.swoff, OFFSET_INC_W);
	// roundup(sensor->limits.shoff, OFFSET_INC_H);
	// /* Tegra doesn't allow image resolutions smaller than 64x32 */
	// sensor->limits.minw = max_t(uint32_t, sensor->limits.minw, FRAMESIZE_MIN_W);
	// sensor->limits.minh = max_t(uint32_t, sensor->limits.minh, FRAMESIZE_MIN_H);
	// sensor->limits.maxw = min_t(uint32_t, sensor->limits.maxw, FRAMESIZE_MAX_W);
	// sensor->limits.maxh = min_t(uint32_t, sensor->limits.maxh, FRAMESIZE_MAX_H);

	// /* Take care of image width alignment*/
	// if (priv->crop_align_enabled) {
	// 	sensor->limits.maxwoff = avt_align_width(sd, sensor->limits.maxwoff);
	// 	sensor->limits.maxw = avt_align_width(sd, sensor->limits.maxw);
	// }

	dump_frame_sizes(sensor);

	return 0;
}

/* V4L2 subdev pad ops */
static int alvium_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct alvium *sensor = to_alvium(sd);

	alvium_dbg(sensor->dev, "index: %d", code->index);

	if (code->index >= 0 && code->index < sensor->num_fmts) {
		code->code = sensor->fmts[code->index];
		return 0;
	}
	return -EINVAL;

	alvium_dbg(sensor->dev, "fmt[%d]: 0x%x", code->index,
		   sensor->fmts[code->index]);

	return 0;
}

static int alvium_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct alvium *sensor = to_alvium(sd);
	int ret = 0;
	bool format_present = false;
	int i;

	alvium_dbg(sensor->dev, "index: %u, code: 0x%x", fse->index, fse->code);

	// mutex_lock(&sensor->lock);

	fse->min_width = fse->max_width = sensor->crop.width;
	fse->min_height = fse->max_height = sensor->crop.height;

	for (i = 0; i < sensor->num_fmts; i++) {
		if (sensor->fmts[i] == fse->code) {
			format_present = true;
			break;
		}
	}

	if (fse->index != 0 || format_present == false)
		return -EINVAL;

	if (fse->min_width > 1 && fse->min_height > 1)
		ret = 0;
	else
		ret = -EINVAL;

	alvium_dbg(
		sensor->dev,
		"width (min/max): (%d/%d), height (min/max): (%d/%d), format: 0x%x",
		fse->min_width, fse->max_width, fse->min_height,
		fse->max_height, fse->code);

	// out:
	// 	mutex_unlock(&sensor->lock);
	return ret;
}

static int
alvium_enum_frame_interval(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct alvium *sensor = to_alvium(sd);
	struct v4l2_fract tpf;
	int ret;
	u64 framerate;

	alvium_dbg(sensor->dev, "index: %d, code: 0x%x", fie->index, fie->code);

	/* Only one frame rate should be returned
	 * - the current frame rate
	 * TODO TODO change this to report all possible frame intervals (from min to max in step)
	 */
	if (fie->index != 0)
		return -EINVAL;

	ret = alvium_read(
		sensor,
		sensor->cci_reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_64RW,
		AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&framerate);
	if (ret < 0) {
		dev_err(sensor->dev, "read frameinterval failed");
		return ret;
	}

	/* Translate frequency to timeperframe
	 * by inverting the fraction
	 */
	tpf.numerator = FRAQ_NUM;
	tpf.denominator = (framerate * FRAQ_NUM) / UHZ_TO_HZ;

	alvium_dbg(sensor->dev, "framerate.numerator = %d, .denominator = %d",
		   tpf.numerator, tpf.denominator);

	// TODO TODO remove this!!!
	tpf.numerator = 1;
	tpf.denominator = ALVIUM_DEFAULT_FPS;
	// TODO TODO end remove

	fie->interval = tpf;

	return 0;
}

static int alvium_frm_supported(int wmin, int wmax, int ws, int hmin, int hmax,
				int hs, int w, int h)
{
	if (w > wmax || w < wmin || h > hmax || h < hmin ||
	    (h - hmin) % hs != 0 || (w - wmin) % ws != 0)
		return -EINVAL;

	return 0;
}

static int alvium_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct alvium *sensor = to_alvium(sd);
	int ret = 0;

	ret = alvium_frm_supported(sensor->limits.w.min, sensor->limits.w.max,
				   sensor->limits.w.step, sensor->limits.h.min,
				   sensor->limits.h.max, sensor->limits.h.step,
				   format->format.width, format->format.height);

	if (ret < 0) {
		dev_err(sensor->dev, "format not supported");
		return ret;
	}

	return ret;
}

static int alvium_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct alvium *sensor = to_alvium(sd);
	struct v4l2_subdev_selection sel;
	int ret = 0;

	alvium_dbg(sensor->dev, "requested: %dx%d, set: %dx%d",
		   format->format.width, format->format.height,
		   sensor->crop.width, sensor->crop.height);

	// changing the resolution is not allowed with VIDIOC_S_FMT
	if (sensor->mode == ALVIUM_BCRM_MODE &&
	    (format->format.width != sensor->crop.width ||
	     format->format.height != sensor->crop.height)) {
		// TODO TODO should be error?
		alvium_warn(
			sensor->dev,
			"Changing the resolution is not allowed with VIDIOC_S_FMT! current: [%dx%d], wanted: [%dx%d]",
			sensor->crop.width, sensor->crop.height,
			format->format.width, format->format.height);
		// return -EINVAL;
	}

	// JPEG or SRGB colorspace?
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_SRGB;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return alvium_try_fmt(sd, cfg, format);

	sel.target = V4L2_SEL_TGT_CROP;
	sel.r = sensor->crop;
	ret = alvium_set_selection(sd, NULL, &sel);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_set_selection failed: %d", ret);
		return ret;
	}

	ret = alvium_write_param(sensor, V4L2_AV_CSI2_PIXELFORMAT_W,
				 format->format.code);
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_PIXELFORMAT_W write err: %d",
			ret);
		return ret;
	}
	// Save format to private data if set_param succeded
	sensor->minfo.fmt_code = format->format.code;

	return 0;
}

static int alvium_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct alvium *sensor = to_alvium(sd);
	int ret = 0;
	u32 val = 0;

	alvium_dbg(sensor->dev, "pad: %d, which: %d", format->pad,
		   format->which);

	// mutex_lock(&sensor->lock);
	// fmt = alvium_get_pad_fmt(sensor, cfg, format->pad, format->which);
	// format->format = *fmt;
	// mutex_unlock(&sensor->lock);

	if (format->pad != 0)
		return -EINVAL;

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_WIDTH_R, &val);
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_WIDTH_R read error: %d",
			ret);
		return ret;
	}
	format->format.width = val;

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_HEIGHT_R, &val);
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_HEIGHT_R read error: %d",
			ret);
		return ret;
	}
	format->format.height = val;

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_PIXELFORMAT_R, &val);
	if (ret < 0) {
		dev_err(sensor->dev,
			"V4L2_AV_CSI2_PIXELFORMAT_R read error: %d", ret);
		return ret;
	}
	format->format.code = val;

	/* Hardcoded default format */
	format->format.field = V4L2_FIELD_NONE;
	format->format.colorspace = V4L2_COLORSPACE_SRGB;
	format->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	format->format.xfer_func = V4L2_XFER_FUNC_DEFAULT;

	alvium_dbg(sensor->dev, "width: %d, height: %d, format: 0x%08x",
		   format->format.width, format->format.height,
		   format->format.code);

	return 0;
}

static int alvium_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct alvium *sensor = to_alvium(sd);
	int ret = 0;

	alvium_dbg(sensor->dev, "");

	// update width, height, offset x/y restrictions from camera
	ret = alvium_init_frame_sizes(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_init_frame_sizes err: %d", ret);
		return ret;
	}

	// mutex_lock(&sensor->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		// TODO do we need this?
		// if (priv->crop_align_enabled) {
		// 	sel->r.width = avt_align_width(sd, sel->r.width);
		// }

		/*
		 *       As per the crop concept document, the following steps should be followed before setting crop to the sensor.
		 *
		 * i)    If new width is less or equal than current width, write the width register first then write offset X (left) register,
		 *       both values should be within the range (min, max and inc).
		 * ii)   If new width is higher than current width, write the offset X (left) register first then write the width register,
		 *       both values should be within the range (min, max, and inc)
		 * iii)  If new height is less or equal than current height, write the height register first then write offset Y (top) register,
		 *       both values should be within the range (min, max and inc).
		 * iv)   If new height is higher than current height, write the offset Y (top) register first then write the height register,
		 *       both values should be within the range (min, max, and inc)
		 */

		if (sel->r.width <= sensor->crop.width) {
			/* case i) New width is lesser or equal than current */

			// write width
			sel->r.width = clamp(
				roundup(sel->r.width, sensor->limits.w.step),
				sensor->limits.w.min, sensor->limits.w.max);
			ret = alvium_write_param(sensor, V4L2_AV_CSI2_WIDTH_W,
						 sel->r.width);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_write_param err: %d", ret);
				return ret;
			}

			// update width, height, offset x/y restrictions from camera
			ret = alvium_init_frame_sizes(sensor);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_init_frame_sizes err: %d", ret);
				return ret;
			}

			// write offset x
			sel->r.left = clamp(roundup(sel->r.left,
						    sensor->limits.woff.step),
					    sensor->limits.woff.min,
					    sensor->limits.woff.max);
			ret = alvium_write_param(
				sensor, V4L2_AV_CSI2_OFFSET_X_W, sel->r.left);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_write_param err: %d", ret);
				return ret;
			}
		} else { /* case ii) New width is higher than current */

			// write offset x
			sel->r.left = clamp(roundup(sel->r.left,
						    sensor->limits.woff.step),
					    sensor->limits.woff.min,
					    sensor->limits.woff.max);
			ret = alvium_write_param(
				sensor, V4L2_AV_CSI2_OFFSET_X_W, sel->r.left);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_write_param err: %d", ret);
				return ret;
			}

			// update width, height, offset x/y restrictions from camera
			ret = alvium_init_frame_sizes(sensor);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_init_frame_sizes err: %d", ret);
				return ret;
			}

			// write width
			sel->r.width = clamp(
				roundup(sel->r.width, sensor->limits.w.step),
				sensor->limits.w.min, sensor->limits.w.max);
			ret = alvium_write_param(sensor, V4L2_AV_CSI2_WIDTH_W,
						 sel->r.width);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_write_param err: %d", ret);
				return ret;
			}
		}

		if (sel->r.height <= sensor->crop.height) {
			/* case iii) New height is lesser or equal than current */
			// write height
			sel->r.height = clamp(
				roundup(sel->r.height, sensor->limits.h.step),
				sensor->limits.h.min, sensor->limits.h.max);
			ret = alvium_write_param(sensor, V4L2_AV_CSI2_HEIGHT_W,
						 sel->r.height);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_write_param err: %d", ret);
				return ret;
			}

			// update width, height, offset x/y restrictions from camera
			ret = alvium_init_frame_sizes(sensor);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_init_frame_sizes err: %d", ret);
				return ret;
			}

			// write offset y
			sel->r.top = clamp(roundup(sel->r.top,
						   sensor->limits.hoff.step),
					   sensor->limits.hoff.min,
					   sensor->limits.hoff.max);
			ret = alvium_write_param(
				sensor, V4L2_AV_CSI2_OFFSET_Y_W, sel->r.top);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_write_param err: %d", ret);
				return ret;
			}
		} else {
			/* case iv) New height is higher than current */

			// write offset y
			sel->r.top = clamp(roundup(sel->r.top,
						   sensor->limits.hoff.step),
					   sensor->limits.hoff.min,
					   sensor->limits.hoff.max);
			ret = alvium_write_param(
				sensor, V4L2_AV_CSI2_OFFSET_Y_W, sel->r.top);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_write_param err: %d", ret);
				return ret;
			}

			// update width, height, offset x/y restrictions from camera
			ret = alvium_init_frame_sizes(sensor);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_init_frame_sizes err: %d", ret);
				return ret;
			}

			// write height
			sel->r.height = clamp(
				roundup(sel->r.height, sensor->limits.h.step),
				sensor->limits.h.min, sensor->limits.h.max);
			ret = alvium_write_param(sensor, V4L2_AV_CSI2_HEIGHT_W,
						 sel->r.height);
			if (ret < 0) {
				dev_err(sensor->dev,
					"alvium_write_param err: %d", ret);
				return ret;
			}
		}

		// update width, height, offset x/y restrictions from camera
		ret = alvium_init_frame_sizes(sensor);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_init_frame_sizes err: %d",
				ret);
			return ret;
		}

		break;
	default:
		return -EINVAL;
	}

	// mutex_unlock(&sensor->lock);

	return 0;
}

static int alvium_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct alvium *sensor = to_alvium(sd);

	alvium_dbg(sensor->dev, "target: %d", sel->target);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: // current cropping area
		sel->r = sensor->crop;

		// TODO or should we read these values again from the device?
		// mutex_lock(&sensor->lock);
		// _crop = alvium_get_pad_crop(sensor, cfg, sel->pad, sel->which);
		// sel->r = *_crop;
		// mutex_unlock(&sensor->lock);

		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS: // cropping area bounds
		sel->r.left = sensor->limits.woff.min;
		sel->r.top = sensor->limits.hoff.min;
		sel->r.width = sensor->limits.w.max;
		sel->r.height = sensor->limits.h.max;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops alvium_subdev_core_ops = {
	.s_power = alvium_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.s_register = alvium_s_register,
	.g_register = alvium_g_register,
#endif
	// .subscribe_event = alvium_csi2_subscribe_event,	// TODO from avt_csi
	// .unsubscribe_event = v4l2_event_subdev_unsubscribe,	// TODO from avt_csi
	// .ioctl = alvium_ioctl,	// TODO from avt_csi
};

static int alvium_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct alvium *sensor = to_alvium(sd);
	// TODO filter pad (reutrn error on wrong pad)
	alvium_dbg(sensor->dev, "called");

	cfg->type = V4L2_MBUS_CSI2_DPHY;

	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	// TODO get this from supported lane count register
	cfg->flags |= V4L2_MBUS_CSI2_4_LANE;

	return 0;
}

static int alvium_g_pixelaspect(struct v4l2_subdev *sd,
				struct v4l2_fract *aspect)
{
	struct alvium *sensor = to_alvium(sd);

	alvium_dbg(sensor->dev, "called");

	aspect->numerator = 1;
	aspect->denominator = 1;

	return 0;
}

// static int alvium_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
// {
// 	struct v4l2_subdev_selection sel;

// 	sel.target = V4L2_SEL_TGT_CROP;
// 	alvium_get_selection(sd, NULL, &sel);
// 	memcpy(&crop->c, &sel.r, sizeof(struct v4l2_rect));

// 	return 0;
// }

static const struct v4l2_subdev_video_ops alvium_subdev_video_ops = {
	.s_stream = alvium_s_stream,
	// .g_mbus_config = alvium_g_mbus_config,
	.g_pixelaspect = alvium_g_pixelaspect,
	.g_frame_interval = alvium_g_frame_interval,
	// .s_frame_interval = alvium_s_frame_interval,
	// .s_parm = alvium_s_parm,	// TODO from avt_csi
	// .g_parm = alvium_g_parm,	// TODO from avt_csi
};

static const struct v4l2_subdev_pad_ops alvium_subdev_pad_ops = {
	.enum_mbus_code = alvium_enum_mbus_code,
	.enum_frame_size = alvium_enum_frame_size,
	.enum_frame_interval = alvium_enum_frame_interval,
	.get_fmt = alvium_get_fmt,
	.set_fmt = alvium_set_fmt,
	.get_selection = alvium_get_selection,
	.set_selection = alvium_set_selection,
	.get_mbus_config = alvium_get_mbus_config,
};

static const struct v4l2_subdev_ops alvium_subdev_ops = {
	.core = &alvium_subdev_core_ops,
	.video = &alvium_subdev_video_ops,
	.pad = &alvium_subdev_pad_ops,
};

static int alvium_link_setup(struct media_entity *entity,
			     struct media_pad const *local,
			     struct media_pad const *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct alvium *sensor = to_alvium(sd);
	int ret = 0;

	if (sensor->is_streaming)
		return -EBUSY;

	if (!(flags & MEDIA_LNK_FL_ENABLED))
		return 0;

	return ret;
}

static const struct media_entity_operations alvium_entity_ops = {
	.link_setup = alvium_link_setup,
	// .link_validate = alvium_link_validate,	// TODO from avt_csi
};

static int alvium_subdev_registered(struct v4l2_subdev *sd)
{
	struct alvium *sensor = to_alvium(sd);
	// int ret = 0;

	alvium_dbg(sensor->dev, "");

	// ret = alvium_create_ctrls(sensor);
	// if (ret < 0) {
	// 	dev_err(sensor->dev, "alvium_create_ctrls failed: %d", ret);
	// 	return ret;
	// }
	// alvium_dbg(sensor->dev, "created ctrls");

	// v4l2_ctrl_handler_setup(&sensor->ctrl_handler);
	return 0;
}

static const struct v4l2_subdev_internal_ops alvium_subdev_internal_ops = {
	// .open = alvium_subdev_open, // TODO from avt_csi
	// .close = alvium_subdev_close, // TODO from avt_csi
	.registered = alvium_subdev_registered,
};

static int alvium_query_ctrl(struct alvium *sensor,
			     struct v4l2_queryctrl *qctrl)
{
	u32 value = 0;
	u64 value64 = 0;
	int ret;
	union bcrm_feature_reg feature_inquiry_reg;
	struct bcrm_to_v4l2 bcrm_v4l2;

	/* reading the Feature inquiry register */
	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr + BCRM_FEATURE_INQUIRY_64R,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
			  (char *)&value64);
	if (ret < 0) {
		dev_err(sensor->dev, "i2c read failed: %d", ret);
		return ret;
	}
	feature_inquiry_reg.value = value64;

	switch (qctrl->id) {
	/* BLACK LEVEL is deprecated and thus we use Brightness */
	case V4L2_CID_BRIGHTNESS:
		alvium_dbg(sensor->dev, "V4L2_CID_BRIGHTNESS");
		strcpy(qctrl->name, "Brightness");

		if (!feature_inquiry_reg.feature_inq.black_level_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'Brightness' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the current Black Level value */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_BLACK_LEVEL_32RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLACK_LEVEL_32RW: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->default_value = value;

		/* reading the Minimum Black Level */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_BLACK_LEVEL_MIN_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLACK_LEVEL_MIN_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->minimum = value;

		/* reading the Maximum Black Level */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_BLACK_LEVEL_MAX_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLACK_LEVEL_MAX_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->maximum = value;

		/* reading the Black Level step increment */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_BLACK_LEVEL_INC_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLACK_LEVEL_INC_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->step = value;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev, "Brightness: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Brightness: non-positive step value (%x)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_CONTRAST:
		alvium_dbg(sensor->dev, "V4L2_CID_CONTRAST");
		strcpy(qctrl->name, "Contrast");

		if (!feature_inquiry_reg.feature_inq.contrast_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'Contrast' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Contrast value */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_CONTRAST_VALUE_32RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"CONTRAST_VALUE_32RW: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->default_value = value;

		/* reading the Minimum Contrast */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_CONTRAST_VALUE_MIN_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"CONTRAST_VALUE_MIN_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->minimum = value;

		/* reading the Maximum Contrast */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_CONTRAST_VALUE_MAX_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"CONTRAST_VALUE_MAX_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->maximum = value;

		/* reading the Contrast step increment */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_CONTRAST_VALUE_INC_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"CONTRAST_VALUE_INC_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->step = value;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev, "Contrast: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Contrast: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_SATURATION:
		alvium_dbg(sensor->dev, "V4L2_CID_SATURATION");
		strcpy(qctrl->name, "Saturation");

		if (!feature_inquiry_reg.feature_inq.saturation_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'Saturation' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Saturation value */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_SATURATION_32RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"SATURATION_32RW: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->default_value = value;

		/* reading the Minimum Saturation */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_SATURATION_MIN_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"SATURATION_MIN_32R: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->minimum = value;

		/* reading the Maximum Saturation */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_SATURATION_MAX_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"SATURATION_MAX_32R: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->maximum = value;

		/* reading the Saturation step increment */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_SATURATION_INC_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"SATURATION_INC_32R: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->step = value;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev, "Saturation: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Saturation: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_HUE:
		alvium_dbg(sensor->dev, "V4L2_CID_HUE");
		strcpy(qctrl->name, "Hue");

		if (!feature_inquiry_reg.feature_inq.hue_avail) {
			alvium_dbg(sensor->dev,
				   "control 'Hue' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Hue value */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_HUE_32RW,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32,
				  (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev, "HUE_32RW: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->default_value = value;

		/* reading the Minimum HUE */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_HUE_MIN_32R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32,
				  (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev, "HUE_MIN_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->minimum = value;

		/* reading the Maximum HUE */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_HUE_MAX_32R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32,
				  (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev, "HUE_MAX_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->maximum = value;

		/* reading the HUE step increment */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_HUE_INC_32R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32,
				  (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev, "HUE_INC_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->step = value;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev, "Hue: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Hue: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		alvium_dbg(sensor->dev, "V4L2_CID_AUTO_WHITE_BALANCE");
		strcpy(qctrl->name, "White Balance Auto");

		if (!feature_inquiry_reg.feature_inq.white_balance_auto_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'White balance Auto' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the White balance auto value */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_WHITE_BALANCE_AUTO_8RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"WHITE_BALANCE_AUTO_8RW: i2c read failed: %d",
				ret);
			return ret;
		}

		if (value == 2)
			/* true (ON) */
			qctrl->default_value = true;
		else
			/* false (OFF) */
			qctrl->default_value = false;

		qctrl->minimum = 0;
		qctrl->maximum = 1;
		qctrl->step = 1;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;

		break;

	case V4L2_CID_DO_WHITE_BALANCE:
		alvium_dbg(sensor->dev, "V4L2_CID_DO_WHITE_BALANCE");
		strcpy(qctrl->name, "White Balance");

		if (!feature_inquiry_reg.feature_inq.white_balance_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'White balance' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the White balance auto reg */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_WHITE_BALANCE_AUTO_8RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"WHITE_BALANCE_AUTO_8RW: i2c read failed: %d",
				ret);
			return ret;
		}

		qctrl->default_value = 0;
		qctrl->minimum = 0;
		qctrl->maximum = 0;
		qctrl->step = 0;
		qctrl->type = V4L2_CTRL_TYPE_BUTTON;

		break;

	case V4L2_CID_RED_BALANCE:
		alvium_dbg(sensor->dev, "V4L2_CID_RED_BALANCE");
		strcpy(qctrl->name, "Red Balance");

		if (!feature_inquiry_reg.feature_inq.white_balance_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'Red balance' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Red balance value */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_RED_BALANCE_RATIO_64RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"RED_BALANCE_RATIO_64RW: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->default_value = (s32)value64;

		/* reading the Minimum Red balance */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr +
					  BCRM_RED_BALANCE_RATIO_MIN_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"RED_BALANCE_RATIO_MIN_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.min_bcrm = value64;

		/* reading the Maximum Red balance */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr +
					  BCRM_RED_BALANCE_RATIO_MAX_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"RED_BALANCE_RATIO_MAX_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.max_bcrm = value64;

		/* reading the Red balance step increment */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr +
					  BCRM_RED_BALANCE_RATIO_INC_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"RED_BALANCE_RATIO_INC_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev,
				"Red Balance: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Red Balance: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		// TODO check for type int or type int64
		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_BLUE_BALANCE:
		alvium_dbg(sensor->dev, "V4L2_CID_BLUE_BALANCE");
		strcpy(qctrl->name, "Blue Balance");

		if (!feature_inquiry_reg.feature_inq.white_balance_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'Blue balance' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Blue balance value */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr +
					  BCRM_BLUE_BALANCE_RATIO_64RW,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLUE_BALANCE_RATIO_64RW: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->default_value = (s32)value64;

		/* reading the Minimum Blue balance */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr +
					  BCRM_BLUE_BALANCE_RATIO_MIN_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLUE_BALANCE_RATIO_MIN_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.min_bcrm = value64;

		/* reading the Maximum Blue balance */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr +
					  BCRM_BLUE_BALANCE_RATIO_MAX_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLUE_BALANCE_RATIO_MAX_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.max_bcrm = value64;

		/* reading the Blue balance step increment */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr +
					  BCRM_BLUE_BALANCE_RATIO_INC_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLUE_BALANCE_RATIO_INC_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev,
				"Blue Balance: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Blue Balance: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_GAMMA:
		alvium_dbg(sensor->dev, "V4L2_CID_GAMMA");
		strcpy(qctrl->name, "Gamma");

		if (!feature_inquiry_reg.feature_inq.gamma_avail) {
			alvium_dbg(sensor->dev,
				   "control 'Gamma' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Gamma value */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_GAMMA_64RW,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev, "GAMMA_64RW: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->default_value = (s32)value64;

		/* reading the Minimum Gamma */
		ret = alvium_read(
			sensor, sensor->cci_reg.bcrm_addr + BCRM_GAMMA_MIN_64R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"GAMMA_GAIN_MIN_64R: i2c read failed: %d", ret);
			return ret;
		}
		bcrm_v4l2.min_bcrm = value64;

		/* reading the Maximum Gamma */
		ret = alvium_read(
			sensor, sensor->cci_reg.bcrm_addr + BCRM_GAMMA_MAX_64R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"GAMMA_GAIN_MAX_64R: i2c read failed: %d", ret);
			return ret;
		}
		bcrm_v4l2.max_bcrm = value64;

		/* reading the Gamma step increment */
		ret = alvium_read(
			sensor, sensor->cci_reg.bcrm_addr + BCRM_GAMMA_INC_64R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"GAMMA_GAIN_INC_64R: i2c read failed: %d", ret);
			return ret;
		}

		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev, "Gamma: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Gamma: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_EXPOSURE:
		alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE");
		strcpy(qctrl->name, "Exposure");

		/* reading the Exposure time */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_EXPOSURE_TIME_64RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"EXPOSURE_TIME_64RW: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->default_value = (s32)value64;

		/* reading the Minimum Exposure time */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_EXPOSURE_TIME_MIN_64R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"EXPOSURE_TIME_MIN_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.min_bcrm = value64;

		/* reading the Maximum Exposure time */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_EXPOSURE_TIME_MAX_64R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"EXPOSURE_TIME_MAX_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.max_bcrm = value64;

		/* reading the Exposure time step increment */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_EXPOSURE_TIME_INC_64R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"EXPOSURE_TIME_INC_64R: i2c read failed: %d",
				ret);
			return ret;
		}
		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev, "Exposure: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Exposure: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_AUTOGAIN:
		alvium_dbg(sensor->dev, "V4L2_CID_AUTOGAIN");
		strcpy(qctrl->name, "Auto Gain");

		if (!feature_inquiry_reg.feature_inq.gain_auto) {
			alvium_dbg(
				sensor->dev,
				"control 'Gain Auto' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Auto Gain value */
		ret = alvium_read(
			sensor, sensor->cci_reg.bcrm_addr + BCRM_GAIN_AUTO_8RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&value);

		if (ret < 0) {
			dev_err(sensor->dev,
				"GAIN_AUTO_8RW: i2c read failed: %d", ret);
			return ret;
		}

		if (value == 2)
			/* true (ON) for continous mode, Refer BCRM doc */
			qctrl->default_value = true;
		else
			/* false (OFF) */
			qctrl->default_value = false;

		qctrl->minimum = 0;
		qctrl->maximum = 1;
		qctrl->step = 1;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;

		break;

	case V4L2_CID_GAIN:
		alvium_dbg(sensor->dev, "V4L2_CID_GAIN");
		strcpy(qctrl->name, "Gain");

		if (!feature_inquiry_reg.feature_inq.gain_avail) {
			alvium_dbg(sensor->dev,
				   "control 'Gain' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Gain value */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_GAIN_64RW,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev, "GAIN_64RW: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->default_value = (s32)value64;

		/* reading the Minimum Gain value */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_GAIN_MIN_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"GAIN_MIN_64R: i2c read failed: %d", ret);
			return ret;
		}
		bcrm_v4l2.min_bcrm = value64;

		/* reading the Maximum Gain value */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_GAIN_MAX_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"GAIN_MAX_64R: i2c read failed: %d", ret);
			return ret;
		}
		bcrm_v4l2.max_bcrm = value64;

		/* reading the Gain step increment */
		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr + BCRM_GAIN_INC_64R,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"GAIN_INC_64R: i2c read failed: %d", ret);
			return ret;
		}

		bcrm_v4l2.step_bcrm = value64;

		convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
		convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);

		qctrl->minimum = bcrm_v4l2.min_v4l2;
		qctrl->maximum = bcrm_v4l2.max_v4l2;
		qctrl->step = bcrm_v4l2.step_v4l2;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev, "Gain: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Gain: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_HFLIP:
		alvium_dbg(sensor->dev, "V4L2_CID_HFLIP");
		strcpy(qctrl->name, "Reverse X");

		if (!feature_inquiry_reg.feature_inq.reverse_x_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'Reversing X (Horizantal Flip)' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Reverse X value */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_IMG_REVERSE_X_8RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"IMG_REVERSE_X_8RW: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->default_value = value;

		qctrl->minimum = 0;
		qctrl->maximum = 1;
		qctrl->step = qctrl->maximum = 1;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;
		break;

	case V4L2_CID_VFLIP:
		alvium_dbg(sensor->dev, "V4L2_CID_VFLIP");
		strcpy(qctrl->name, "Reverse Y");

		if (!feature_inquiry_reg.feature_inq.reverse_y_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'Reversing Y (Vertical Flip)' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Reverse Y value */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_IMG_REVERSE_Y_8RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"IMG_REVERSE_Y_8RW: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->default_value = value;

		qctrl->minimum = 0;
		qctrl->maximum = 1;
		qctrl->step = 1;
		qctrl->type = V4L2_CTRL_TYPE_BOOLEAN;

		break;

	case V4L2_CID_SHARPNESS:
		alvium_dbg(sensor->dev, "V4L2_CID_SHARPNESS");
		strcpy(qctrl->name, "Sharpness");

		if (!feature_inquiry_reg.feature_inq.sharpness_avail) {
			alvium_dbg(
				sensor->dev,
				"control 'Sharpness' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the Sharpness value */
		ret = alvium_read(
			sensor, sensor->cci_reg.bcrm_addr + BCRM_SHARPNESS_32RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"SHARPNESS_32RW: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->default_value = value;

		/* reading the Minimum sharpness */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_SHARPNESS_MIN_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"SHARPNESS_MIN_32R: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->minimum = value;

		/* reading the Maximum sharpness */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_SHARPNESS_MAX_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"SHARPNESS_MAX_32R: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->maximum = value;

		/* reading the sharpness step increment */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_SHARPNESS_INC_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"SHARPNESS_INC_32R: i2c read failed: %d", ret);
			return ret;
		}
		qctrl->step = value;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev, "Sharpness: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}
		if (qctrl->step <= 0) {
			dev_err(sensor->dev,
				"Sharpness: non-positive step value (%d)!",
				qctrl->step);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_EXPOSURE_AUTO:
		alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE_AUTO");
		strcpy(qctrl->name, "Exposure Auto");

		if (!feature_inquiry_reg.feature_inq.exposure_auto) {
			alvium_dbg(
				sensor->dev,
				"control 'Exposure Auto' not supported by firmware");
			qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
			return 0;
		}

		/* reading the current exposure auto value */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_EXPOSURE_AUTO_8RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"EXPOSURE_AUTO_8RW: i2c read failed: %d", ret);
			return ret;
		}

		if (value == 2)
			/* continous mode, Refer BCRM doc */
			qctrl->default_value = V4L2_EXPOSURE_AUTO;
		else
			/* false (OFF) */
			qctrl->default_value = V4L2_EXPOSURE_MANUAL;

		qctrl->minimum = V4L2_EXPOSURE_AUTO;
		qctrl->maximum = V4L2_EXPOSURE_MANUAL;
		qctrl->step = 0;
		qctrl->type = V4L2_CTRL_TYPE_MENU;

		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE_ABSOLUTE");
		strcpy(qctrl->name, "Exposure Absolute");

		/* reading the Exposure time */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_EXPOSURE_TIME_64RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"EXPOSURE_TIME_64RW: i2c read failed: %d", ret);
			return ret;
		}

		/* convert unit [ns] to [100*us] */
		value64 = value64 / EXP_ABS;

		/* clamp to s32 max */
		qctrl->default_value = clamp(value64, (u64)0, (u64)S32_MAX);

		/* reading the Maximum Exposure time */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_EXPOSURE_TIME_MAX_64R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&value64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"EXPOSURE_TIME_MAX_64R: i2c read failed: %d",
				ret);
			return ret;
		}

		/* convert unit [ns] to [100*us] */
		value64 = value64 / EXP_ABS;

		/* clamp to s32 max */
		qctrl->maximum = clamp(value64, (u64)0, (u64)S32_MAX);

		qctrl->minimum = 1;
		qctrl->step = 1;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev,
				"Exposure Absolute: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;

		break;

	case V4L2_CID_LINK_FREQ:
		alvium_dbg(sensor->dev, "V4L2_CID_LINK_FREQ");
		strcpy(qctrl->name, "Link Frequency");

		/* reading the link freq */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_CSI2_CLOCK_32RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BCRM_CSI2_CLOCK_32RW: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->default_value = value;

		/* reading the minimum link frequency */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_CSI2_CLOCK_MIN_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BCRM_CSI2_CLOCK_MIN_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->minimum = value;

		/* reading the maximum link frequency */
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_CSI2_CLOCK_MAX_32R,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BCRM_CSI2_CLOCK_MAX_32R: i2c read failed: %d",
				ret);
			return ret;
		}
		qctrl->maximum = value;

		qctrl->step = 1;

		if (qctrl->minimum > qctrl->maximum) {
			dev_err(sensor->dev,
				"Link Frequency: min > max! (%d > %d)",
				qctrl->minimum, qctrl->maximum);
			return -EINVAL;
		}

		qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		qctrl->flags = V4L2_CTRL_FLAG_READ_ONLY;

		break;

		// case V4L2_CID_TEMPERATURE:
		// 	alvium_dbg(sensor->dev, "V4L2_CID_TEMPERATURE");
		// 	strcpy(qctrl->name, "Temperature");

		// 	if (!feature_inquiry_reg.feature_inq.device_temperature_avail) {
		// 		alvium_dbg(
		// 			sensor->dev,
		// 			"control 'Temperature' not supported by firmware");
		// 		qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
		// 		return 0;
		// 	}

		// 	/* reading the temperature */
		// 	ret = alvium_read(
		// 		sensor,
		// 		sensor->cci_reg.bcrm_addr + BCRM_DEVICE_TEMPERATURE_32R,
		// 		AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (char *)&value);
		// 	if (ret < 0) {
		// 		dev_err(sensor->dev,
		// 			"BCRM_DEVICE_TEMPERATURE_32R: i2c read failed: %d",
		// 			ret);
		// 		return ret;
		// 	}
		// 	qctrl->default_value = value;
		// 	qctrl->minimum = S32_MIN;
		// 	qctrl->maximum = S32_MAX;
		// 	qctrl->step = 1;

		// 	qctrl->type = V4L2_CTRL_TYPE_INTEGER;
		// 	qctrl->flags =
		// 		V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE;
		// 	break;

	default:
		alvium_dbg(sensor->dev,
			   "default or not supported qctrl->id 0x%x",
			   qctrl->id);
		qctrl->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}

	if (ret < 0) {
		dev_err(sensor->dev, "i2c read failed: %d", ret);
		return ret;
	}

	return 0;
}

static int alvium_s_ctrl(struct v4l2_ctrl *ctrl)
{
	// struct alvium *sensor = ctrl->priv;
	struct alvium *sensor =
		container_of(ctrl->handler, struct alvium, ctrl_handler);
	int ret = 0;
	u32 reg = 0, length = 0;
	u8 val8 = 0;
	s32 val32 = 0;
	u64 val64 = 0;
	struct v4l2_queryctrl qctrl;
	struct v4l2_subdev_frame_interval interval;
	CLEAR(qctrl);

	alvium_dbg(sensor->dev, "ctrl id: 0x%x. val: 0x%x (%d)", ctrl->id,
		   ctrl->val, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		reg = BCRM_BLACK_LEVEL_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_BRIGHTNESS;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		val32 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_CONTRAST:
		reg = BCRM_CONTRAST_VALUE_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_CONTRAST;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		val32 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_SATURATION:
		reg = BCRM_SATURATION_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_SATURATION;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		val32 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_HUE:
		reg = BCRM_HUE_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_HUE;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		val32 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;

		/* BCRM Auto White balance changes	*/
		if (ctrl->val == true) {
			val8 = 2; /* Continouous mode */
		} else {
			val8 = 0; /* 1; OFF/once mode */
		}

		break;

	case V4L2_CID_DO_WHITE_BALANCE:
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;

		/* Set 'once' in White Balance Auto Register. */
		val8 = 1;

		break;

	case V4L2_CID_RED_BALANCE:
		reg = BCRM_RED_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_RED_BALANCE;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}

		val64 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_BLUE_BALANCE:
		reg = BCRM_BLUE_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_BLUE_BALANCE;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}

		val64 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_GAMMA:
		reg = BCRM_GAMMA_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_GAMMA;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		val64 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_EXPOSURE:
		alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE: %d", ctrl->val);

		/* i) Setting 'Manual' in Exposure Auto reg*/
		alvium_dbg(sensor->dev, "disabling automatic exposure");
		val8 = 0;
		ret = alvium_write(sensor,
				   sensor->cci_reg.bcrm_addr +
					   BCRM_EXPOSURE_AUTO_8RW,
				   AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, &val8);
		if (ret < 0) {
			dev_err(sensor->dev,
				"Failed to disable automatic exposure");
			return ret;
		}
		if (sensor->auto_exposure_ctrl != NULL) {
			ret = __v4l2_ctrl_s_ctrl(sensor->auto_exposure_ctrl,
						 V4L2_EXPOSURE_MANUAL);
			if (ret < 0) {
				dev_err(sensor->dev,
					"Failed to disable automatic exposure");
				return ret;
			}
		}

		/* ii) Setting value in Exposure reg. */
		reg = BCRM_EXPOSURE_TIME_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_EXPOSURE;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		alvium_dbg(sensor->dev, "exposure: min=%d, max=%d, step=%d",
			   qctrl.minimum, qctrl.maximum, qctrl.step);

		val64 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);
		alvium_dbg(sensor->dev, "should set exposure: %d", ctrl->val);

		/* Setting the absolute exposure control */
		if (sensor->exposure_absolute_ctrl != NULL) {
			if (!sensor->cross_update) {
				sensor->cross_update = true;
				ret = __v4l2_ctrl_s_ctrl(
					sensor->exposure_absolute_ctrl,
					ctrl->val / EXP_ABS);
			} else {
				sensor->cross_update = false;
			}
		}

		break;

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE_ABSOLUTE: %d",
			   ctrl->val);

		/* i) Setting 'Manual' in Exposure Auto reg*/
		alvium_dbg(sensor->dev, "disabling automatic exposure");
		val8 = 0;
		ret = alvium_write(sensor,
				   sensor->cci_reg.bcrm_addr +
					   BCRM_EXPOSURE_AUTO_8RW,
				   AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, &val8);
		if (ret < 0) {
			dev_err(sensor->dev,
				"Failed to disable automatic exposure");
			return ret;
		}
		if (sensor->auto_exposure_ctrl != NULL) {
			ret = __v4l2_ctrl_s_ctrl(sensor->auto_exposure_ctrl,
						 V4L2_EXPOSURE_MANUAL);
			if (ret < 0) {
				dev_err(sensor->dev,
					"Failed to disable automatic exposure");
				return ret;
			}
		}

		/* ii) Setting value in Exposure reg. */
		reg = BCRM_EXPOSURE_TIME_64RW;
		length = AV_CAM_DATA_SIZE_64;

		qctrl.id = V4L2_CID_EXPOSURE;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		alvium_dbg(sensor->dev, "exposure: min=%d, max=%d, step=%d",
			   qctrl.minimum, qctrl.maximum, qctrl.step);

		val64 = clamp(((u64)(ctrl->val)) * EXP_ABS, (u64)0,
			      (u64)S32_MAX);
		val64 = convert_s_ctrl(val64, qctrl.minimum, qctrl.maximum,
				       qctrl.step);
		alvium_dbg(sensor->dev, "should set exposure: %ld", val64);

		/* Setting the exposure control */
		if (sensor->exposure_ctrl != NULL) {
			if (!sensor->cross_update) {
				sensor->cross_update = true;
				__v4l2_ctrl_s_ctrl(sensor->exposure_ctrl,
						   ctrl->val * EXP_ABS);
			} else {
				sensor->cross_update = false;
			}
		}

		break;

	case V4L2_CID_EXPOSURE_AUTO:
		alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE_AUTO: %d",
			   ctrl->val);

		reg = BCRM_EXPOSURE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;

		/* BCRM Auto Gain changes */
		if (ctrl->val == V4L2_EXPOSURE_AUTO) {
			val8 = 2; /* Continouous mode */
		} else {
			val8 = 0; /* 1; OFF/once mode */
		}

		break;

		// TODO implement these variables?
		// V4L2_CID_EXPOSURE_AUTO_MIN
		// V4L2_CID_EXPOSURE_AUTO_MAX
		// V4L2_CID_AUTOGAIN_MIN
		// V4L2_CID_AUTOGAIN_MAX
		// V4L2_CID_LINK_FREQ

	case V4L2_CID_AUTOGAIN:
		reg = BCRM_GAIN_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;

		/* BCRM Auto Gain changes */
		if (ctrl->val == true)
			val8 = 2; /* Continouous mode */
		else
			val8 = 0; /* 1; OFF/once mode */

		break;

	case V4L2_CID_GAIN:
		reg = BCRM_GAIN_64RW;
		length = AV_CAM_DATA_SIZE_64;

		// TODO first disable automatic gain?

		qctrl.id = V4L2_CID_GAIN;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		val64 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_HFLIP:
		reg = BCRM_IMG_REVERSE_X_8RW;
		length = AV_CAM_DATA_SIZE_8;

		if (ctrl->val == true)
			val8 = 1; /* FLIP */
		else
			val8 = 0; /* UNCHANGED */

		break;

	case V4L2_CID_VFLIP:
		reg = BCRM_IMG_REVERSE_Y_8RW;
		length = AV_CAM_DATA_SIZE_8;

		if (ctrl->val == true)
			val8 = 1; /* FLIP */
		else
			val8 = 0; /* UNCHANGED */

		break;

	case V4L2_CID_SHARPNESS:
		reg = BCRM_SHARPNESS_32RW;
		length = AV_CAM_DATA_SIZE_32;

		qctrl.id = V4L2_CID_SHARPNESS;
		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0) {
			dev_err(sensor->dev, "alvium_query_ctrl failed: ret %d",
				ret);
			return ret;
		}
		val32 = ctrl->val = convert_s_ctrl(ctrl->val, qctrl.minimum,
						   qctrl.maximum, qctrl.step);

		break;

	case V4L2_CID_LINK_FREQ:
		// intentionally do nothing, as it is read-only
		return 0;

	default:
		dev_err(sensor->dev, "default or not supported");
		ret = -EINVAL;
		return ret;
	}

	alvium_dbg(sensor->dev, "ctrl->val = %d, reg = %x, length = %d",
		   ctrl->val, reg, length);

	switch (length) {
	case AV_CAM_DATA_SIZE_8:
		ret = alvium_write(sensor, sensor->cci_reg.bcrm_addr + reg,
				   AV_CAM_REG_SIZE, length, (u8 *)&(val8));
		break;
	case AV_CAM_DATA_SIZE_32:
		ret = alvium_write(sensor, sensor->cci_reg.bcrm_addr + reg,
				   AV_CAM_REG_SIZE, length, (u8 *)&(val32));
		break;
	case AV_CAM_DATA_SIZE_64:
		ret = alvium_write(sensor, sensor->cci_reg.bcrm_addr + reg,
				   AV_CAM_REG_SIZE, length, (u8 *)&(val64));
		break;
	default:
		dev_err(sensor->dev, "unsupported reg size: %d", length);
		return -1;
	}
	if (ret < 0) {
		dev_err(sensor->dev, "write error: %d", ret);
		return ret;
	}

	// maximize framerate, according to exposure time!?
	if ((ctrl->id == V4L2_CID_EXPOSURE ||
	     ctrl->id == V4L2_CID_EXPOSURE_ABSOLUTE)) {
		interval.pad = 0;
		interval.interval.numerator = UHZ_TO_HZ;
		interval.interval.denominator =
			max((u64)(ALVIUM_DEFAULT_FPS * UHZ_TO_HZ), val64);
		alvium_dbg(sensor->dev, "maximizing framerate: %d / %d",
			   interval.interval.numerator,
			   interval.interval.denominator);
		ret = alvium_s_frame_interval(&sensor->subdev, &interval);
		if (ret < 0) {
			dev_err(sensor->dev, "set frame interval failed: %d",
				ret);
			return ret;
		}
		alvium_dbg(sensor->dev, "framerate successfully set");
	} else if (ctrl->id == V4L2_CID_AUTO_WHITE_BALANCE) {
		// read red and blue balance values back and store them
		ret = alvium_read(
			sensor,
			sensor->cci_reg.bcrm_addr + BCRM_RED_BALANCE_RATIO_64RW,
			AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&val64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"RED_BALANCE_RATIO_64RW: i2c read failed: %d",
				ret);
			return ret;
		}
		alvium_dbg(sensor->dev, "RED_BALANCE_RATIO_64RW: %ld", val64);
		if (sensor->red_balance_ctrl != NULL) {
			ret = __v4l2_ctrl_s_ctrl(sensor->red_balance_ctrl,
						 val64);
			if (ret < 0) {
				dev_err(sensor->dev,
					"Failed to set red_balance_ctrl");
				return ret;
			}
		}

		ret = alvium_read(sensor,
				  sensor->cci_reg.bcrm_addr +
					  BCRM_BLUE_BALANCE_RATIO_64RW,
				  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64,
				  (char *)&val64);
		if (ret < 0) {
			dev_err(sensor->dev,
				"BLUE_BALANCE_RATIO_64RW: i2c read failed: %d",
				ret);
			return ret;
		}
		alvium_dbg(sensor->dev, "BLUE_BALANCE_RATIO_64RW: %ld", val64);
		if (sensor->blue_balance_ctrl != NULL) {
			ret = __v4l2_ctrl_s_ctrl(sensor->blue_balance_ctrl,
						 val64);
			if (ret < 0) {
				dev_err(sensor->dev,
					"Failed to set blue_balance_ctrl");
				return ret;
			}
		}
	}

	return 0;
}

static int alvium_g_ctrl(struct v4l2_ctrl *ctrl)
{
	// struct alvium *sensor = ctrl->priv;
	struct alvium *sensor =
		container_of(ctrl->handler, struct alvium, ctrl_handler);
	int ret = 0;
	u32 reg = 0, length = 0;
	struct bcrm_to_v4l2 bcrm_v4l2;
	struct v4l2_queryctrl qctrl;
	u64 val64 = 0;

	alvium_dbg(sensor->dev, "ctrl id: %d.", ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		/* BLACK LEVEL is deprecated and thus we use Brightness */
		alvium_dbg(sensor->dev, "V4L2_CID_BRIGHTNESS");
		reg = BCRM_BLACK_LEVEL_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	case V4L2_CID_GAMMA:
		alvium_dbg(sensor->dev, "V4L2_CID_GAMMA");
		reg = BCRM_GAIN_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_CONTRAST:
		alvium_dbg(sensor->dev, "V4L2_CID_CONTRAST");
		reg = BCRM_CONTRAST_VALUE_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	case V4L2_CID_DO_WHITE_BALANCE:
		alvium_dbg(sensor->dev, "V4L2_CID_DO_WHITE_BALANCE");
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_CID_AUTO_WHITE_BALANCE:
		alvium_dbg(sensor->dev, "V4L2_CID_AUTO_WHITE_BALANCE");
		reg = BCRM_WHITE_BALANCE_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_CID_SATURATION:
		alvium_dbg(sensor->dev, "V4L2_CID_SATURATION");
		reg = BCRM_SATURATION_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	case V4L2_CID_HUE:
		alvium_dbg(sensor->dev, "V4L2_CID_HUE");
		reg = BCRM_HUE_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	case V4L2_CID_RED_BALANCE:
		alvium_dbg(sensor->dev, "V4L2_CID_RED_BALANCE");
		reg = BCRM_RED_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_BLUE_BALANCE:
		alvium_dbg(sensor->dev, "V4L2_CID_BLUE_BALANCE");
		reg = BCRM_BLUE_BALANCE_RATIO_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_EXPOSURE:
		reg = BCRM_EXPOSURE_TIME_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_GAIN:
		alvium_dbg(sensor->dev, "V4L2_CID_GAIN");
		reg = BCRM_GAIN_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_AUTOGAIN:
		alvium_dbg(sensor->dev, "V4L2_CID_AUTOGAIN");
		reg = BCRM_GAIN_AUTO_8RW;
		length = AV_CAM_DATA_SIZE_8;
		break;

	case V4L2_CID_SHARPNESS:
		alvium_dbg(sensor->dev, "V4L2_CID_SHARPNESS");
		reg = BCRM_SHARPNESS_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

	case V4L2_CID_EXPOSURE_AUTO_MIN:
		alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE_AUTO_MIN");
		reg = BCRM_EXPOSURE_AUTO_MIN_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_EXPOSURE_AUTO_MAX:
		alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE_AUTO_MAX");
		reg = BCRM_EXPOSURE_AUTO_MAX_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_AUTOGAIN_MIN:
		alvium_dbg(sensor->dev, "V4L2_CID_AUTOGAIN_MIN");
		reg = BCRM_GAIN_AUTO_MIN_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

	case V4L2_CID_AUTOGAIN_MAX:
		alvium_dbg(sensor->dev, "V4L2_CID_AUTOGAIN_MAX");
		reg = BCRM_GAIN_AUTO_MAX_64RW;
		length = AV_CAM_DATA_SIZE_64;
		break;

		// case V4L2_CID_EXPOSURE_ACTIVE_LINE_MODE:
		// 	alvium_dbg(sensor->dev, "V4L2_CID_EXPOSURE_ACTIVE_LINE_MODE");
		// 	reg = BCRM_EXPOSURE_ACTIVE_LINE_MODE_8RW;
		// 	length = AV_CAM_DATA_SIZE_8;
		// 	break;

		// case V4L2_CID_EXPOSURE_ACTIVE_LINE_SELECTOR:
		// 	alvium_dbg(sensor->dev,
		// 		   "V4L2_CID_EXPOSURE_ACTIVE_LINE_SELECTOR");
		// 	reg = BCRM_EXPOSURE_ACTIVE_LINE_SELECTOR_8RW;
		// 	length = AV_CAM_DATA_SIZE_8;
		// 	break;

		// case V4L2_CID_EXPOSURE_ACTIVE_INVERT:
		// TODO implement
		// 	ctrl->val = sensor->acquisition_active_invert;
		// 	return 0;

	case V4L2_CID_LINK_FREQ:
		alvium_dbg(sensor->dev, "V4L2_CID_LINK_FREQ");
		reg = BCRM_CSI2_CLOCK_32RW;
		length = AV_CAM_DATA_SIZE_32;
		break;

		// case V4L2_CID_TEMPERATURE:
		// 	alvium_dbg(sensor->dev, "V4L2_CID_TEMPERATURE");
		// 	reg = BCRM_DEVICE_TEMPERATURE_32R;
		// 	length = AV_CAM_DATA_SIZE_32;
		// 	break;

	default:
		dev_err(sensor->dev, "default or not supported");
		return -EINVAL;
	}

	CLEAR(bcrm_v4l2);
	CLEAR(qctrl);

	qctrl.id = ctrl->id;
	ret = alvium_query_ctrl(sensor, &qctrl);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_query_ctrl failed: %d", ret);
		return ret;
	}

	bcrm_v4l2.min_v4l2 = qctrl.minimum;
	bcrm_v4l2.max_v4l2 = qctrl.maximum;
	bcrm_v4l2.step_v4l2 = qctrl.step;

	/* Overwrite the queryctrl maximum value for auto features since value
	 * 2 is 'true' (1)
	 */
	if (ctrl->id == V4L2_CID_AUTOGAIN ||
	    ctrl->id == V4L2_CID_AUTO_WHITE_BALANCE ||
	    ctrl->id == V4L2_CID_HFLIP || ctrl->id == V4L2_CID_VFLIP)
		bcrm_v4l2.max_v4l2 = 2;

	/* Check values from BCRM */
	if ((bcrm_v4l2.min_v4l2 > bcrm_v4l2.max_v4l2) ||
	    (bcrm_v4l2.step_v4l2 <= 0)) {
		dev_err(sensor->dev, "invalid BCRM values found. ctrl->id %d",
			ctrl->id);
		return -EINVAL;
	}

	ret = alvium_read(sensor, sensor->cci_reg.bcrm_addr + reg,
			  AV_CAM_REG_SIZE, length, (char *)&val64);

	ctrl->val = convert_bcrm_to_v4l2_gctrl(
		&bcrm_v4l2, val64, ctrl->id == V4L2_CID_EXPOSURE_ABSOLUTE);

	/* BCRM Auto Exposure changes -> Refer to BCRM document */
	if (ctrl->id == V4L2_CID_EXPOSURE_AUTO) {
		if (ctrl->val == 2)
			/* continous mode, Refer BCRM doc */
			ctrl->val = V4L2_EXPOSURE_AUTO;
		else
			/* OFF for off & once mode, Refer BCRM doc */
			ctrl->val = V4L2_EXPOSURE_MANUAL;
	}

	/* BCRM Auto Gain/WB changes -> Refer to BCRM document */
	if (ctrl->id == V4L2_CID_AUTOGAIN ||
	    ctrl->id == V4L2_CID_AUTO_WHITE_BALANCE ||
	    ctrl->id == V4L2_CID_HFLIP || ctrl->id == V4L2_CID_VFLIP) {
		if (ctrl->val == 2)
			/* continous mode, Refer BCRM doc */
			ctrl->val = true;
		else
			/* OFF for off & once mode, Refer BCRM doc */
			ctrl->val = false;
	}

	alvium_dbg(sensor->dev, "ctrl val: %d.", ctrl->val);

	return 0;
}

static const struct v4l2_ctrl_ops alvium_ctrl_ops = {
	.s_ctrl = alvium_s_ctrl,
	.g_volatile_ctrl = alvium_g_ctrl,
};

static int alvium_create_ctrls(struct alvium *sensor)
{
	struct v4l2_ctrl_config ctrl_cfg;
	struct v4l2_ctrl *ctrl;
	struct v4l2_queryctrl qctrl;
	int i;
	int ret;

	ret = v4l2_ctrl_handler_init(&sensor->ctrl_handler,
				     ARRAY_SIZE(alvium_ctrl_mappings));
	if (ret < 0)
		return ret;

	sensor->subdev.ctrl_handler = &sensor->ctrl_handler;
	sensor->ctrl_handler.lock = &sensor->lock;

	// init controls with NULL
	sensor->link_freq_ctrl = NULL;
	sensor->exposure_ctrl = NULL;
	sensor->exposure_absolute_ctrl = NULL;
	sensor->auto_exposure_ctrl = NULL;
	sensor->auto_gain_ctrl = NULL;
	sensor->red_balance_ctrl = NULL;
	sensor->blue_balance_ctrl = NULL;
	sensor->cross_update = false;

	for (i = 0; i < ARRAY_SIZE(alvium_ctrl_mappings); ++i) {
		CLEAR(qctrl);
		CLEAR(ctrl_cfg);
		qctrl.id = alvium_ctrl_mappings[i].id;

		ret = alvium_query_ctrl(sensor, &qctrl);
		if (ret < 0)
			continue;

		alvium_dbg(
			sensor->dev,
			"Checking caps: %s - Range: %d-%d s: %d d: %d - %sabled",
			alvium_ctrl_mappings[i].attr.name, qctrl.minimum,
			qctrl.maximum, qctrl.step, qctrl.default_value,
			(qctrl.flags & V4L2_CTRL_FLAG_DISABLED) ? "dis" : "en");

		if (qctrl.flags & V4L2_CTRL_FLAG_DISABLED)
			continue;

		ctrl_cfg.type = qctrl.type;
		ctrl_cfg.flags = qctrl.flags;

		if (qctrl.type == V4L2_CTRL_TYPE_INTEGER)
			ctrl_cfg.flags |= V4L2_CTRL_FLAG_SLIDER;

		ctrl_cfg.ops = &alvium_ctrl_ops;
		ctrl_cfg.name = alvium_ctrl_mappings[i].attr.name;
		ctrl_cfg.id = alvium_ctrl_mappings[i].id;

		ctrl_cfg.min = qctrl.minimum;
		ctrl_cfg.max = qctrl.maximum;
		ctrl_cfg.def = qctrl.default_value;
		ctrl_cfg.step = qctrl.step;

		// if (qctrl.type == V4L2_CTRL_TYPE_MENU) {
		// 	alvium_dbg(sensor->dev, "Adding Menu: %d, %d, %d\n",
		// 		   qctrl.id, qctrl.maximum,
		// 		   qctrl.default_value);
		// 	ctrl = v4l2_ctrl_new_std_menu(&sensor->ctrl_handler,
		// 				      &alvium_ctrl_ops,
		// 				      qctrl.id, qctrl.maximum,
		// 				      0, qctrl.default_value);

		// } else {
		// 	alvium_dbg(sensor->dev,
		// 		   "Adding Standard: %d, %d, %d, %d, %d\n",
		// 		   qctrl.id, qctrl.minimum, qctrl.maximum,
		// 		   qctrl.step, qctrl.default_value);
		// 	ctrl = v4l2_ctrl_new_std(&sensor->ctrl_handler,
		// 				 &alvium_ctrl_ops, qctrl.id,
		// 				 qctrl.minimum, qctrl.maximum,
		// 				 qctrl.step,
		// 				 qctrl.default_value);
		// }
		ctrl = v4l2_ctrl_new_custom(&sensor->ctrl_handler, &ctrl_cfg,
					    sensor);
		if (ctrl == NULL) {
			dev_err(sensor->dev, "Failed to init %s ctrl",
				ctrl_cfg.name);
			continue;
		}

		ret = sensor->ctrl_handler.error;
		if (ret < 0) {
			dev_warn(sensor->dev,
				 "failed to register control "
				 "'%s'(0x%x): %d",
				 ctrl_cfg.name ?
					 ctrl_cfg.name :
					 v4l2_ctrl_get_name(ctrl_cfg.id),
				 ctrl_cfg.id, ret);
			continue;
		}

		switch (qctrl.id) {
		case V4L2_CID_EXPOSURE_AUTO:
			sensor->auto_exposure_ctrl = ctrl;
			break;
		case V4L2_CID_AUTOGAIN:
			sensor->auto_gain_ctrl = ctrl;
			break;
		case V4L2_CID_EXPOSURE:
			sensor->exposure_ctrl = ctrl;
			break;
		case V4L2_CID_EXPOSURE_ABSOLUTE:
			sensor->exposure_absolute_ctrl = ctrl;
			break;
		case V4L2_CID_RED_BALANCE:
			sensor->red_balance_ctrl = ctrl;
			break;
		case V4L2_CID_BLUE_BALANCE:
			sensor->blue_balance_ctrl = ctrl;
			break;
		case V4L2_CID_LINK_FREQ:
			sensor->link_freq_ctrl = ctrl;
			break;
		}
	}

	// init all controls once
	ret = v4l2_ctrl_handler_setup(sensor->subdev.ctrl_handler);

	return ret;
}

static int alvium_parse_mipi_ep(struct alvium *sensor, struct device_node *ep)
{
	struct device *dev = sensor->dev;
	struct fwnode_handle *fwnode;
	struct v4l2_fwnode_endpoint endpoint = { .bus_type =
							 V4L2_MBUS_CSI2_DPHY };
	int ret;
	int i;

	// Skip parsing if no endpoint was found
	if (!ep)
		return 0;

	fwnode = of_fwnode_handle(ep);

	ret = v4l2_fwnode_endpoint_alloc_parse(fwnode, &endpoint);
	if (ret < 0) {
		dev_err(dev, "Failed to parse MIPI endpoint (%d)", ret);
		return ret;
	}

	sensor->minfo.num_lanes = endpoint.bus.mipi_csi2.num_data_lanes;
	if (sensor->minfo.num_lanes < 1 || sensor->minfo.num_lanes > 4) {
		dev_err(dev, "Wrong number of lanes configured");
		return -EINVAL;
	}
	alvium_dbg(sensor->dev, "MIPI lanes configured: %d",
		   sensor->minfo.num_lanes);

	// if (of_property_read_u32(fwnode, "csi_clk_freq",
	// 			 &sensor->minfo.clk_freq))
	// 	sensor->minfo.clk_freq = 0;

	// alvium_dbg(sensor->dev, "csi_clk_freq: %d, %d", sensor->minfo.clk_freq,
	// 	endpoint.nr_of_link_frequencies);
	alvium_dbg(sensor->dev, "num freqs: %d",
		   endpoint.nr_of_link_frequencies);
	for (i = 0; i < endpoint.nr_of_link_frequencies; i++) {
		alvium_dbg(sensor->dev, "freq %d: %llu", i,
			   endpoint.link_frequencies[i]);
		sensor->minfo.clk_freq = endpoint.link_frequencies[i];
	}

	v4l2_fwnode_endpoint_free(&endpoint);

	return 0;
}

static int alvium_of_probe(struct alvium *sensor)
{
	struct device *dev = sensor->dev;
	struct device_node *ep;
	struct gpio_desc *gpio;
	int ret;

	// get clock - actually we dont need external clock ;)
	// clk = devm_clk_get(dev, "ext");
	// ret = PTR_ERR_OR_ZERO(clk);
	// if (ret == -EPROBE_DEFER)
	// 	return ret;
	// if (ret < 0) {
	// 	dev_err(dev, "Failed to get external clock (%d)", ret);
	// 	return ret;
	// }
	// sensor->extclk = clk;

	// get reset GPIO
	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	ret = PTR_ERR_OR_ZERO(gpio);
	if (ret < 0) {
		dev_err(dev, "Failed to get reset gpio (%d)", ret);
		return ret;
	}
	sensor->reset_gpio = gpio;

	// get MIPI endpoint
	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "No endpoint found");
		return -ENODEV;
	}

	ret = alvium_parse_mipi_ep(sensor, ep);
	of_node_put(ep);

	return ret;
}

static int alvium_cci_version_check(struct alvium *sensor)
{
	u32 cci_minver, cci_majver;

	cci_minver =
		(sensor->cci_reg.layout_version & CCI_REG_LAYOUT_MINVER_MASK) >>
		CCI_REG_LAYOUT_MINVER_SHIFT;

	if (cci_minver == CCI_REG_LAYOUT_MINVER) {
		alvium_dbg(sensor->dev,
			   "correct cci register minver: %d (0x%x)", cci_minver,
			   sensor->cci_reg.layout_version);
	} else {
		dev_err(sensor->dev,
			"cci reg minver mismatch! read: %d (0x%x) expected: %d. Ignoring.",
			cci_minver, sensor->cci_reg.layout_version,
			CCI_REG_LAYOUT_MINVER);
		// return -EINVAL;
	}

	cci_majver =
		(sensor->cci_reg.layout_version & CCI_REG_LAYOUT_MAJVER_MASK) >>
		CCI_REG_LAYOUT_MAJVER_SHIFT;

	if (cci_majver == CCI_REG_LAYOUT_MAJVER) {
		alvium_dbg(sensor->dev,
			   "correct cci register majver: %d (0x%x)", cci_majver,
			   sensor->cci_reg.layout_version);
	} else {
		dev_err(sensor->dev,
			"cci reg majver mismatch! read: %d (0x%x) expected: %d. Error.",
			cci_majver, sensor->cci_reg.layout_version,
			CCI_REG_LAYOUT_MAJVER);
		return -EINVAL;
	}

	return 0;
}

static int alvium_bcrm_version_check(struct alvium *sensor)
{
	u32 value = 0;
	int ret;

	/* reading the BCRM version */
	ret = alvium_read(sensor, sensor->cci_reg.bcrm_addr + BCRM_VERSION_32R,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_32, (u8 *)&value);
	if (ret < 0) {
		dev_err(sensor->dev, "i2c read failed: %d", ret);
		return ret;
	}

	alvium_dbg(sensor->dev,
		   "bcrm version (driver): 0x%x (maj: 0x%x min: 0x%x)",
		   BCRM_DEVICE_VERSION, BCRM_MAJOR_VERSION, BCRM_MINOR_VERSION);
	alvium_dbg(sensor->dev,
		   "bcrm version (camera): 0x%x (maj: 0x%x min: 0x%x)", value,
		   (value & 0xffff0000) >> 16, (value & 0x0000ffff));

	return (value >> 16) == BCRM_MAJOR_VERSION ? 1 : 0;
}

static bool alvium_bcrm_get_write_handshake_availibility(struct alvium *sensor)
{
	u8 value = 0;
	int ret;

	/* Reading the device firmware version from camera */
	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr + BCRM_WRITE_HANDSHAKE_8RW,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, (char *)&value);

	if ((ret >= 0) && (value & 0x80)) {
		alvium_dbg(sensor->dev, "BCRM write handshake supported!");
		return true;
	} else {
		alvium_warn(sensor->dev, "BCRM write handshake NOT supported!");
		return false;
	}
	return false;
}

static void dump_frame_intervals(struct alvium *sensor)
{
	alvium_dbg(sensor->dev,
		   "framerate = %d / %d, (min/max/inc) = (%llu, %llu, %llu)\n",
		   sensor->interval.denominator, sensor->interval.numerator,
		   sensor->intervals.min, sensor->intervals.max,
		   sensor->intervals.step);
}

static int alvium_init_frame_intervals(struct alvium *sensor)
{
	int ret = 0;
	u64 val64;
	struct bcrm_to_v4l2 bcrm_v4l2;

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_FRAMERATE_MINVAL_R,
				(u32 *)&val64);
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_FRAMERATE_MINVAL_R err: %d",
			ret);
		return ret;
	}
	bcrm_v4l2.min_bcrm = val64;

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_FRAMERATE_MAXVAL_R,
				(u32 *)&val64);
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_FRAMERATE_MAXVAL_R err: %d",
			ret);
		return ret;
	}
	bcrm_v4l2.max_bcrm = val64;

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_FRAMERATE_INCVAL_R,
				(u32 *)&val64);
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_FRAMERATE_INCVAL_R err: %d",
			ret);
		return ret;
	}
	bcrm_v4l2.step_bcrm = val64;

	convert_bcrm_to_v4l2(&bcrm_v4l2, min_enum, true);
	convert_bcrm_to_v4l2(&bcrm_v4l2, max_enum, true);
	convert_bcrm_to_v4l2(&bcrm_v4l2, step_enum, true);
	sensor->intervals.min = bcrm_v4l2.min_v4l2;
	sensor->intervals.max = bcrm_v4l2.max_v4l2;
	sensor->intervals.step = max(bcrm_v4l2.step_v4l2, 1); // 1 UHz

	ret = alvium_read_param(sensor, V4L2_AV_CSI2_FRAMERATE_R,
				(u32 *)&val64);
	if (ret < 0) {
		dev_err(sensor->dev, "V4L2_AV_CSI2_FRAMERATE_R err: %d", ret);
		return ret;
	}
	sensor->interval.numerator = FRAQ_NUM;
	sensor->interval.denominator = (val64 * FRAQ_NUM) / UHZ_TO_HZ;

	dump_frame_intervals(sensor);

	return 0;
}

static int alvium_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct alvium *sensor = to_alvium(sd);
	union bcrm_feature_reg feature_inquiry_reg;
	u8 val8;
	s64 val64;
	int ret;

	// init all available frame intervals
	ret = alvium_init_frame_intervals(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_init_frame_intervals failed: %d",
			ret);
		return ret;
	}

	/* reading the Feature inquiry register */
	ret = alvium_read(sensor,
			  sensor->cci_reg.bcrm_addr + BCRM_FEATURE_INQUIRY_64R,
			  AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&val64);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_read failed: %d", ret);
		return ret;
	}
	feature_inquiry_reg.value = val64;

	/* Check if setting acquisition frame rate is available */
	if (!feature_inquiry_reg.feature_inq.acquisition_frame_rate) {
		dev_err(sensor->dev, "Acquisition frame rate not supported");
		return -EPERM;
	}

	/* Translate timeperframe to frequency by inverting the fraction */
	alvium_dbg(sensor->dev, "%u / %u\n", interval->interval.numerator,
		   interval->interval.denominator);
	val64 = (interval->interval.denominator /
		 interval->interval.numerator) *
		UHZ_TO_HZ;
	val64 = convert_s_ctrl(val64, sensor->intervals.min,
			       sensor->intervals.max, sensor->intervals.step);
	alvium_dbg(sensor->dev,
		   "framerate = %llu, (min/max/inc) = (%llu, %llu, %llu)\n",
		   val64, sensor->intervals.min, sensor->intervals.max,
		   sensor->intervals.step);
	if (val64 < 0) {
		dev_err(sensor->dev, "Frame rate: non-positive value (%llu)!\n",
			val64);
		return -EINVAL;
	}

	/* Enable manual frame rate */
	val8 = 1;
	ret = alvium_write(sensor,
			   sensor->cci_reg.bcrm_addr +
				   BCRM_ACQUISITION_FRAME_RATE_ENABLE_8RW,
			   AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_8, &val8);
	if (ret < 0) {
		dev_err(sensor->dev, "frame rate enable failed: %d", ret);
		return ret;
	}

	/* Save new frame rate to camera register */
	ret = alvium_write(
		sensor,
		sensor->cci_reg.bcrm_addr + BCRM_ACQUISITION_FRAME_RATE_64RW,
		AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (char *)&val64);
	if (ret < 0) {
		dev_err(sensor->dev, "frame rate write failed: %d", ret);
		return ret;
	}

	// update internal framerate
	sensor->interval.numerator = FRAQ_NUM;
	sensor->interval.denominator = val64 / FRAQ_NUM;

	return 0;
}

static int alvium_probe(struct i2c_client *i2c, const struct i2c_device_id *did)
{
	struct alvium *sensor;
	struct v4l2_subdev *sd;
	union bcrm_device_firmware_version_reg firmware_version;
	struct v4l2_subdev_format format;
	struct v4l2_subdev_frame_interval interval;
	int ret;

	alvium_dbg(&i2c->dev, "alvium chip found @ 0x%x (%s)", i2c->addr << 1,
		   i2c->adapter->name);

	sensor = devm_kzalloc(&i2c->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	// sensor->model = did->driver_data;
	sensor->dev = &i2c->dev;

	alvium_dbg(sensor->dev, "Probing ALVIUM Driver");

	ret = alvium_of_probe(sensor);
	if (ret < 0)
		return ret;

	mutex_init(&sensor->lock);

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, i2c, &alvium_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->internal_ops = &alvium_subdev_internal_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &alvium_entity_ops;

	sensor->pad[0].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, ALVIUM_NUM_PADS, sensor->pad);
	if (ret < 0)
		goto out1;

	// TODO eventually soft_reset here: release GenCP mode

	ret = alvium_read_cci_registers(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_read_cci_registers failed: %d",
			ret);
		goto out1;
	}

	ret = alvium_cci_version_check(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "cci version mismatch!");
		goto out1;
	}
	alvium_dbg(sensor->dev, "correct cci version");

	ret = alvium_bcrm_version_check(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "bcrm version mismatch!");
		goto out1;
	}
	alvium_dbg(sensor->dev, "correct bcrm version");

	sensor->write_handshake_available =
		alvium_bcrm_get_write_handshake_availibility(sensor);

	// read all formats
	ret = alvium_init_avail_formats(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_init_avail_formats failed: %d",
			ret);
		goto out1;
	}

	// init frame sizes
	ret = alvium_init_frame_sizes(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_init_frame_sizes failed: %d", ret);
		goto out1;
	}

	// init frame intervals
	ret = alvium_init_frame_intervals(sensor);
	if (ret < 0) {
		dev_err(sensor->dev, "alvium_init_frame_intervals failed: %d",
			ret);
		goto out1;
	}

	// read firmware version
	ret = alvium_read(
		sensor,
		sensor->cci_reg.bcrm_addr + BCRM_DEVICE_FIRMWARE_VERSION_64R,
		AV_CAM_REG_SIZE, AV_CAM_DATA_SIZE_64, (u8 *)&firmware_version);
	if (ret < 0)
		goto out1;
	alvium_dbg(sensor->dev, "Firmware version: %d.%d.%d.%d ret = %d",
		   firmware_version.version.special,
		   firmware_version.version.major,
		   firmware_version.version.minor,
		   firmware_version.version.patch, ret);
	// device_caps.value = sensor->cci_reg.device_capabilities;

#if 0
	//TODO: GENCP
	// if (device_caps.caps.gencp) {
	// 	ret = read_gencp_registers(client);
	// 	if (ret < 0) {
	// 		dev_err(dev, "%s: read_gencp_registers failed: %d",
	// 				 ret);
	// 		return ret;
	// 	}

	// 	ret = gcprm_version_check(client);
	// 	if (ret < 0) {
	// 		dev_err(&client->dev, "gcprm version mismatch!");
	// 		return ret;
	// 	}

	// 	alvium_dbg(&client->dev, "correct gcprm version");
	// }
#endif

	ret = alvium_create_ctrls(sensor);
	if (ret < 0)
		goto out2;
	alvium_dbg(sensor->dev, "created ctrls");

	sensor->is_open = false;
	sensor->is_streaming = false;
	ret = alvium_init_mode(sensor);
	if (ret < 0)
		goto out2;
	alvium_dbg(sensor->dev, "mode init");

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0)
		goto out3;

	// sets frame format (and frame size internally)
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.code = ALVIUM_DEFAULT_FMT;
	format.format.width = sensor->crop.width;
	format.format.height = sensor->crop.height;
	ret = alvium_set_fmt(sd, NULL, &format);
	if (ret < 0) {
		dev_err(sensor->dev, "set format failed: %d", ret);
		goto out3;
	}
	alvium_dbg(sensor->dev, "format set");

	// set framerate
	interval.pad = 0;
	interval.interval.numerator = 1;
	interval.interval.denominator = ALVIUM_DEFAULT_FPS;
	ret = alvium_s_frame_interval(sd, &interval);
	if (ret < 0) {
		dev_err(sensor->dev, "set frame interval failed: %d", ret);
		goto out3;
	}
	alvium_dbg(sensor->dev, "framerate set");

	alvium_dbg(sensor->dev, "sensor %s registered", sensor->subdev.name);

	// we luckily had no errors
	return 0;

out3:
	v4l2_async_unregister_subdev(&sensor->subdev);
out2:
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
out1:
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&sensor->lock);
	return ret;
}

static int alvium_remove(struct i2c_client *i2c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(i2c);
	struct alvium *sensor = to_alvium(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&sensor->lock);

	return 0;
}

static const struct i2c_device_id alvium_id_table[] = { { "alvium", 0 },
							{ /* sentinel */ } };
MODULE_DEVICE_TABLE(i2c, alvium_id_table);

static const struct of_device_id alvium_of_match[] = {
	{ .compatible = "alliedvision,alvium" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, alvium_of_match);

static struct i2c_driver alvium_i2c_driver = {
	.driver = {
		.name = "alvium",
		.of_match_table = of_match_ptr(alvium_of_match),
	},
	.probe = alvium_probe,
	.remove = alvium_remove,
	.id_table = alvium_id_table,
};
module_i2c_driver(alvium_i2c_driver);

MODULE_AUTHOR("Nicolai Behmann <behmann@beh.digital>");
MODULE_DESCRIPTION("Allied Vision Alvium MIPI-CSI2 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1.0");
