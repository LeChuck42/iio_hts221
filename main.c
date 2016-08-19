/* 
 * Linux IIO driver for the ST HTS221 Capacitive digital sensor
 * for relative humidity and temperature
 * 
 * Copyright (C) 2016  Matthias Seidel
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
 * ===============================================================================
 * 
 * What works:
 * - Raw and converted output values via sysfs/iio
 * - Setting conversion and averaging
 * - Controlling heater
 * - Using single-shot as well as periodic measurements
 * 
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "hts221.h"

MODULE_AUTHOR("Matthias Seidel <matthias.seidel@ametek.com>");
MODULE_DESCRIPTION("ST HTS221");
MODULE_LICENSE("GPL v2");

#define FP_FACTOR 16

struct hts221_state {
	struct spi_device       *spi;
	u8                      ctrl;
	bool                  heater;
	u8                       avg;
	
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
			__be32 d32;
			u8 d8[4];
	} tx_buf ____cacheline_aligned;
	union {
			__be32 d32;
			u8 d8[4];
	} rx_buf ____cacheline_aligned;
};

enum hts221_type {
	ID_HTS221
};

static struct hts221_calib {
	int temp_offset;
	int temp_scale;
	int hum_offset;
	int hum_scale;
} hts221_calib_buf;
	

static struct iio_chan_spec hts221_channels[] = {
	{
		.type = IIO_TEMP,
		.indexed = 0,
		.output = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		                      BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 4,
		}
	},
	{
		.type = IIO_HUMIDITYRELATIVE,
		.indexed = 0,
		.output = 1,
		.channel = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		                      BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 4,
		}
	},
};


static int hts221_write_unlocked(struct iio_dev *indio_dev, unsigned int addr,
	unsigned int val)
{
	struct hts221_state *st = iio_priv(indio_dev);
	int ret;
	
	st->tx_buf.d8[0] = (addr & HTS221_ADDR_MASK) | HTS221_CMD_WRITE | HTS221_CMD_NOINCR;
	st->tx_buf.d8[1] = val;
	ret = spi_write(st->spi, &st->tx_buf, 2);

	return ret;
}

static int hts221_read8(struct iio_dev *indio_dev, unsigned int addr)
{
	struct hts221_state *st = iio_priv(indio_dev);
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->tx_buf.d8[0],
			.rx_buf = &st->rx_buf.d8[0],
			.len = 2
		}
	};
	
	mutex_lock(&indio_dev->mlock);
	st->tx_buf.d32 = 0;
	st->tx_buf.d8[0] = (addr & HTS221_ADDR_MASK) | HTS221_CMD_READ | HTS221_CMD_NOINCR;
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret >= 0)
		ret = st->rx_buf.d8[1];
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int hts221_read16(struct iio_dev *indio_dev, unsigned int addr)
{
	struct hts221_state *st = iio_priv(indio_dev);
	int ret;
	
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->tx_buf.d8[0],
			.rx_buf = &st->rx_buf.d8[0],
			.len = 3
		}
	};
	
	mutex_lock(&indio_dev->mlock);
	st->tx_buf.d32 = 0;
	st->tx_buf.d8[0] = (addr & HTS221_ADDR_MASK) | HTS221_CMD_READ | HTS221_CMD_INCR;
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret >= 0)
		ret = (st->rx_buf.d8[2] << 8) | st->rx_buf.d8[1];
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int hts221_update_ctrl(struct iio_dev *indio_dev, u8 set, u8 clr)
{
	struct hts221_state *st = iio_priv(indio_dev);
	int ret;
	mutex_lock(&indio_dev->mlock);
	
	st->ctrl &= ~clr;
	st->ctrl |= set;
	
	ret = hts221_write_unlocked(indio_dev, REG_CNTRL1_ADDR, st->ctrl);
	mutex_unlock(&indio_dev->mlock);
	
	return ret;
}

static int hts221_read_calib(struct iio_dev *indio_dev)
{
	int buf;
	
	u16 H[2];
	u16 T[2];
	s16 H_OUT[2];
	s16 T_OUT[2];
	
	buf = hts221_read16(indio_dev, REG_0RH_CAL_Y_H);
	if (buf < 0)
		return buf;
		
	H[0] = buf & 0xFF;
	H[1] = (buf >> 8) & 0xFF;
	
	buf = hts221_read16(indio_dev, REG_0T_CAL_Y_H);
	if (buf < 0)
		return buf;
		
	T[0] = buf & 0xFF;
	T[1] = (buf >> 8) & 0xFF;
	
	buf = hts221_read8(indio_dev, REG_T1_T0_CAL_Y_H);
	if (buf < 0)
		return buf;
		
	T[0] |= (buf << 8) & 0x300;
	T[1] |= (buf << 6) & 0x300;
	
	buf = hts221_read16(indio_dev, REG_0RH_CAL_X_H);
	if (buf < 0)
		return buf;
		
	H_OUT[0] = buf;
	
	buf = hts221_read16(indio_dev, REG_1RH_CAL_X_H);
	if (buf < 0)
		return buf;
		
	H_OUT[1] = buf;
	
	buf = hts221_read16(indio_dev, REG_0T_CAL_X_L);
	if (buf < 0)
		return buf;
		
	T_OUT[0] = buf;

	buf = hts221_read16(indio_dev, REG_1T_CAL_X_L);
	if (buf < 0)
		return buf;
		
	T_OUT[1] = buf;
	
	hts221_calib_buf.temp_scale = ((T[1]-T[0])<<(FP_FACTOR))/(T_OUT[1]-T_OUT[0]);
	hts221_calib_buf.temp_offset = (T[0]<<FP_FACTOR) - T_OUT[0]*hts221_calib_buf.temp_scale;
	
	hts221_calib_buf.hum_scale = ((H[1]-H[0])<<FP_FACTOR)/(H_OUT[1]-H_OUT[0]);
	hts221_calib_buf.hum_offset = (H[0]<<FP_FACTOR) - H_OUT[0]*hts221_calib_buf.hum_scale;
	
	return 0;
}

static ssize_t hts221_write_odr(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	int ret;

	if (strncmp("single", buf, 6) == 0)
		ret = hts221_update_ctrl(indio_dev, ODR_ONESH, ODR_MASK);
	else if (strncmp("1hz", buf, 3) == 0)
		ret = hts221_update_ctrl(indio_dev, ODR_1, ODR_MASK);
	else if (strncmp("7hz", buf, 3) == 0)
		ret = hts221_update_ctrl(indio_dev, ODR_7, ODR_MASK);
	else if (strncmp("12hz5", buf, 5) == 0)
		ret = hts221_update_ctrl(indio_dev, ODR_12, ODR_MASK);
	else
		return -EINVAL;

	if (ret)
		return ret;

	return len;
}

static ssize_t hts221_read_odr(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hts221_state *st = iio_priv(indio_dev);

	unsigned int odr = st->ctrl & ODR_MASK;
	if (buf) {
		*buf = 0;
		if (odr == ODR_ONESH)
			strcat(buf, "[single] ");
		else
			strcat(buf, " single  ");
		
		if (odr == ODR_1)
			strcat(buf, "[1hz] ");
		else
			strcat(buf, " 1hz  ");
		
		if (odr == ODR_7)
			strcat(buf, "[7hz] ");
		else
			strcat(buf, " 7hz  ");
		if (odr == ODR_12)
			strcat(buf, "[12hz5]\n");
		else
			strcat(buf, " 12hz5 \n");
		
		return 9+6+6+8;
	}
	return 0;
}

static ssize_t hts221_read_ctrl(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hts221_state *st = iio_priv(indio_dev);

	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	
	unsigned int val = (st->ctrl & this_attr->address)?1:0;
	
	return sprintf(buf, "%u\n", val);
}

static ssize_t hts221_write_ctrl(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	int ret;
	bool val;

	ret = strtobool(buf, &val);
	if (ret)
		return ret;

	if (val) 
		ret = hts221_update_ctrl(indio_dev, this_attr->address, 0);
	else
		ret = hts221_update_ctrl(indio_dev, 0, this_attr->address);

	return ret ? ret : len;
}

static ssize_t hts221_read_cntrl2(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	
	int ret;
	ret = hts221_read8(indio_dev, REG_CNTRL2_ADDR);
	
	if (ret < 0)
		return ret;
	
	return sprintf(buf, "%u\n", (ret & this_attr->address)?1:0);
}

static ssize_t hts221_write_cntrl2(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hts221_state *st = iio_priv(indio_dev);

	int ret;
	bool val;
	
	unsigned int set_bits;

	ret = strtobool(buf, &val);
	if (ret)
			return ret;

	mutex_lock(&indio_dev->mlock);
	
	if (this_attr->address == CNTRL2_BIT_HEATER)
		st->heater = val;
	
	set_bits = (st->heater)?CNTRL2_BIT_HEATER:0;
	
	if (val) 
		set_bits |= this_attr->address;
		
	ret = hts221_write_unlocked(indio_dev, REG_CNTRL2_ADDR, set_bits);
	
	mutex_unlock(&indio_dev->mlock);
	return ret ? ret : len;
}

static ssize_t hts221_read_avg(struct device *dev,
                               struct device_attribute *attr,
                               char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hts221_state *st = iio_priv(indio_dev);

	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	
	unsigned int bin;
	unsigned int val;
	
	if (this_attr->address == HTS221_H_AVG)
	{
		bin = st->avg & 0x7;
		val = 1<<(bin+2);
	}
	else
	{
		bin = (st->avg >> 3) & 0x7;
		val = 1<<(bin+1);
	}
	
	return sprintf(buf, "%u\n", val);
}

static ssize_t hts221_write_avg(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hts221_state *st = iio_priv(indio_dev);

	int ret;
	u32 val;
	
	unsigned int ld;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	// log2
	for (ld = 0; val > 2; val>>=1, ld++);
	
	mutex_lock(&indio_dev->mlock);

	if (this_attr->address == HTS221_H_AVG)
		st->avg = (st->avg & 0x38) | (ld - 1);
	else
		st->avg = (st->avg & 0x07) | (ld << 3);
		
	ret = hts221_write_unlocked(indio_dev, REG_AVG_ADDR, st->avg);
	
	mutex_unlock(&indio_dev->mlock);
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(data_rate,
			S_IRUGO | S_IWUSR,
			hts221_read_odr,
			hts221_write_odr, 0);

static IIO_DEVICE_ATTR(enabled,
			S_IRUGO | S_IWUSR,
			hts221_read_ctrl,
			hts221_write_ctrl,
			CNTRL1_BIT_PD);

static IIO_DEVICE_ATTR(heater,
			S_IRUGO | S_IWUSR,
			hts221_read_cntrl2,
			hts221_write_cntrl2,
			CNTRL2_BIT_HEATER);

static IIO_DEVICE_ATTR(start_single,
			S_IRUGO | S_IWUSR,
			hts221_read_cntrl2,
			hts221_write_cntrl2,
			CNTRL2_BIT_ONE_SHOT);

static IIO_DEVICE_ATTR(out_temp_avg,
			S_IRUGO | S_IWUSR,
			hts221_read_avg,
			hts221_write_avg,
			HTS221_T_AVG);

static IIO_DEVICE_ATTR(out_humidityrelative_avg,
			S_IRUGO | S_IWUSR,
			hts221_read_avg,
			hts221_write_avg,
			HTS221_H_AVG);


static struct attribute *hts221_attributes[] = {
	&iio_dev_attr_data_rate.dev_attr.attr,
	&iio_dev_attr_enabled.dev_attr.attr,
	&iio_dev_attr_heater.dev_attr.attr,
	&iio_dev_attr_start_single.dev_attr.attr,
	&iio_dev_attr_out_temp_avg.dev_attr.attr,
	&iio_dev_attr_out_humidityrelative_avg.dev_attr.attr,
	NULL
};


static int hts221_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	s16 raw;
	
	if (chan->type == IIO_TEMP)
	{
		raw = hts221_read16(indio_dev, REG_T_OUT_L);
		if (m == IIO_CHAN_INFO_RAW)
		{
			*val = raw;
			return IIO_VAL_INT;
		}
		if (m==IIO_CHAN_INFO_PROCESSED)
		{
			*val = hts221_calib_buf.temp_offset + (int)raw * hts221_calib_buf.temp_scale;
			*val2 = FP_FACTOR + HTS221_TEMP_FACTOR;
			return IIO_VAL_FRACTIONAL_LOG2;
		}
		return -EINVAL;
	}
	else if (chan->type == IIO_HUMIDITYRELATIVE)
	{
		raw = hts221_read16(indio_dev, REG_H_OUT_L);
		if (m == IIO_CHAN_INFO_RAW)
		{
			*val = raw;
			return IIO_VAL_INT;
		}
		if (m==IIO_CHAN_INFO_PROCESSED)
		{
			*val = hts221_calib_buf.hum_offset + (int)raw * hts221_calib_buf.hum_scale;
			*val2 = FP_FACTOR + HTS221_HUM_FACTOR;
			return IIO_VAL_FRACTIONAL_LOG2;
		}
		return -EINVAL;
	}
	return -EINVAL;
}

static int hts221_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	return -EINVAL;
}


static const struct attribute_group hts221_attribute_group = {
	.attrs = hts221_attributes,
};


static const struct iio_info hts221_info = {
	.read_raw = hts221_read_raw,
	.write_raw = hts221_write_raw,
	.attrs = &hts221_attribute_group,
	.driver_module = THIS_MODULE,
};

static int hts221_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct hts221_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL) {
		dev_err(&spi->dev, "Failed to allocate iio device\n");
		return  -ENOMEM;
	}

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &hts221_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = 2;
	indio_dev->channels = hts221_channels;

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register iio device: %d\n", ret);
		return ret;
	}
	
	spi->mode = SPI_MODE_3;
	
	ret = hts221_read8(indio_dev, REG_WHOAMI_ADDR);
	if (ret != WHOIAM_VALUE)
	{
		dev_err(&spi->dev, "Unknown WHO_AM_I value: %d\n", ret);
		iio_device_unregister(indio_dev);
		return -ENOSYS;
	}
	
	ret = hts221_read_calib(indio_dev);
	if (ret)
	{
		dev_err(&spi->dev, "Error while reading calibration values: %d\n", ret);
		iio_device_unregister(indio_dev);
		return -ENOSYS;
	}
	
	ret = hts221_update_ctrl(indio_dev, CNTRL1_BIT_BDU, 0xFF);
	if (ret)
	{
		dev_err(&spi->dev, "Error while initializing control register: %d\n", ret);
		iio_device_unregister(indio_dev);
		return -ENOSYS;
	}
	
	st->avg = hts221_read8(indio_dev, REG_AVG_ADDR) & 0x3F;
	
	return 0;
}

static int hts221_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	
	// disable measurement
	hts221_update_ctrl(indio_dev, 0, 0xFF);
	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id hts221_ids[] = {
	{ "hts221", ID_HTS221 },
	{}
};
MODULE_DEVICE_TABLE(spi, hts221_ids);

static struct spi_driver hts221_driver = {
	.driver = {
		   .name = "hts221",
	},
	.probe = hts221_probe,
	.remove = hts221_remove,
	.id_table = hts221_ids,
};

module_spi_driver(hts221_driver);
