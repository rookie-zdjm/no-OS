/***************************************************************************//**
 *   @file   iio_demo_dev.h
 *   @brief  iio description of demo_dev
 *   @author Mihail Chindris (mihail.chindris@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef IIO_DEMO_DEV
#define IIO_DEMO_DEV

#include <stdlib.h>
#include "demo_dev.h"

static struct iio_attribute iio_attr_demo_channel = {
	.name = "demo_channel_attr",
	.show = get_demo_channel_attr,
	.store = set_demo_channel_attr,
};

static struct iio_attribute iio_attr_demo_global = {
	.name = "demo_global_attr",
	.show = get_demo_global_attr,
	.store = set_demo_global_attr,
};

static struct iio_attribute *demo_channel_attributes[] = {
	&iio_attr_demo_channel,
	NULL,
};

static struct iio_channel iio_demo_channel_voltage0_in = {
	.name = "input_channel",
	.scan_index = 0,
	.scan_type = {
		.sign = 's',
		.realbits = 12,
		.storagebits = 16,
		.shift = 0,
		.is_big_endian = false
	},
	.attributes = demo_channel_attributes,
	.ch_out = false,
};

static struct iio_channel iio_demo_channel_voltage0_out = {
	.name = "output_channel",
	.scan_index = 0,
	.scan_type = {
		.sign = 's',
		.realbits = 12,
		.storagebits = 16,
		.shift = 0,
		.is_big_endian = false
	},
	.attributes = demo_channel_attributes,
	.ch_out = true,
};

static struct iio_channel *iio_demo_channels_in[] = {
	&iio_demo_channel_voltage0_in,
	NULL,
};

static struct iio_channel *iio_demo_channels_out[] = {
	&iio_demo_channel_voltage0_out,
	NULL,
};

static struct iio_attribute *iio_demo_global_attributes[] = {
	&iio_attr_demo_global,
	NULL,
};

static struct iio_device iio_demo_dev_in_descriptor = {
	.num_ch = DEMO_NUM_CHANNELS,
	.channels = iio_demo_channels_in,
	.attributes = iio_demo_global_attributes,
	.debug_attributes = NULL,
	.buffer_attributes = NULL,
	.transfer_dev_to_mem = iio_demo_transfer_dev_to_mem,
	.transfer_mem_to_dev = iio_demo_transfer_mem_to_dev,
	.read_data = iio_demo_read_dev,
	.write_data = iio_demo_write_dev
};


static struct iio_device iio_demo_dev_out_descriptor = {
	.num_ch = DEMO_NUM_CHANNELS,
	.channels = iio_demo_channels_out,
	.attributes = iio_demo_global_attributes,
	.debug_attributes = NULL,
	.buffer_attributes = NULL,
	.transfer_dev_to_mem = iio_demo_transfer_dev_to_mem,
	.transfer_mem_to_dev = iio_demo_transfer_mem_to_dev,
	.read_data = iio_demo_read_dev,
	.write_data = iio_demo_write_dev
};

#endif /* IIO_DEMO_DEV */
