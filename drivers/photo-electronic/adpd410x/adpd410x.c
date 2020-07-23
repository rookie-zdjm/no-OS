/***************************************************************************//**
 *   @file   adpd410x.c
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "error.h"

#include "adpd410x.h"

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Read device register.
 * @param dev - Device handler.
 * @param address - Register address.
 * @param data - Pointer to the register value container.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_reg_read(struct adpd410x_dev *dev, uint16_t address,
			  uint16_t *data)
{
	int32_t ret;
	uint8_t buff[] = {0, 0, 0, 0};

	if(dev->dev_type == ADPD4100) {
		buff[0] = (address & 0x7f80) >> 7;
		buff[1] = (address & 0x7f) << 1;
		buff[1] &= 0xfe;

		ret = spi_write_and_read(dev->dev_ops.spi_phy_dev, buff, 4);
		if(ret != SUCCESS)
			return FAILURE;
	} else {
		buff[0] = (address & 0x7f00) >> 8;
		buff[0] |= 0x80;
		buff[1] = address & 0xff;

		/* No stop bit */
		ret = i2c_write(dev->dev_ops.i2c_phy_dev, buff, 2, 0);
		if(ret != SUCCESS)
			return FAILURE;
		ret = i2c_read(dev->dev_ops.i2c_phy_dev, (buff + 2), 2, 1);
		if(ret != SUCCESS)
			return FAILURE;
	}

	*data = ((uint16_t)buff[2] << 8) & 0xff00;
	*data |= buff[3] & 0xff;

	return SUCCESS;
}

/**
 * @brief Write device register.
 * @param dev - Device handler.
 * @param address - Register address.
 * @param data - New register value.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_reg_write(struct adpd410x_dev *dev, uint16_t address,
			   uint16_t data)
{
	uint8_t buff[] = {0, 0, 0, 0};

	if(dev->dev_type == ADPD4100) {
		buff[0] = (address & 0x7f80) >> 7;
		buff[1] = (address & 0x7f) << 1;
		buff[1] &= 0xfe;
		buff[2] = (data & 0xff00) >> 8;
		buff[3] = data & 0xff;

		return spi_write_and_read(dev->dev_ops.spi_phy_dev, buff, 4);
	} else {
		buff[0] = (address & 0x7f00) >> 8;
		buff[0] |= 0x80;
		buff[1] = address & 0xff;
		buff[2] = (data & 0xff00) >> 8;
		buff[3] = data & 0xff;

		/* No stop bit */
		return i2c_write(dev->dev_ops.i2c_phy_dev, buff, 4, 1);
	}
}

/**
 * @brief Do a software reset.
 * @param dev - Device handler.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_reset(struct adpd410x_dev *dev)
{
	int32_t ret;
	uint16_t data;

	ret = adpd410x_reg_read(dev, ADPD410X_REG_SYS_CTL, &data);
	if(ret != SUCCESS)
		return ret;
	data |= BITM_SYS_CTL_SW_RESET;

	return adpd410x_reg_write(dev, ADPD410X_REG_SYS_CTL, data);
}

/**
 * @brief Set operation mode.
 * @param dev - Device handler.
 * @param mode - New operation mode.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_set_opmode(struct adpd410x_dev *dev,
			    enum adpd410x_opmode mode)
{
	int32_t ret;
	uint16_t data;

	ret = adpd410x_reg_read(dev, ADPD410X_REG_OPMODE, &data);
	if(ret != SUCCESS)
		return ret;
	data &= ~BITM_OPMODE_OP_MODE;
	data |= mode;

	return adpd410x_reg_write(dev, ADPD410X_REG_OPMODE, data);
}

/**
 * @brief Set number of active time slots.
 * @param dev - Device handler.
 * @param timeslot_no - Last time slot to be enabled.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_set_last_timeslot(struct adpd410x_dev *dev,
				   enum adpd410x_timeslots timeslot_no)
{
	int32_t ret;
	uint16_t data;

	ret = adpd410x_reg_read(dev, ADPD410X_REG_OPMODE, &data);
	if(ret != SUCCESS)
		return ret;
	data &= ~BITM_OPMODE_TIMESLOT_EN;
	data |= ((timeslot_no & 0x0f) << BITP_OPMODE_TIMESLOT_EN);

	return adpd410x_reg_write(dev, ADPD410X_REG_OPMODE, data);
}

/**
 * @brief Set device sampling frequency.
 * @param dev - Device handler.
 * @param sampling_freq - New sampling frequency.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_set_sampling_freq(struct adpd410x_dev *dev,
				   uint32_t sampling_freq)
{
	int32_t ret;
	uint32_t reg_load;
	uint16_t reg_temp;

	if((dev->clk_opt == ADPD410X_INTLFO_INTHFO) ||
	    (dev->clk_opt == ADPD410X_INTLFO_EXTHFO)) {
		ret = adpd410x_reg_read(dev, ADPD410X_REG_SYS_CTL, &reg_temp);
		if(ret != SUCCESS)
			return FAILURE;
		if(reg_temp & BITP_SYS_CTL_LFOSC_SEL)
			reg_load = ADPD410X_LOW_FREQ_OSCILLATOR_FREQ1;
		else
			reg_load = ADPD410X_LOW_FREQ_OSCILLATOR_FREQ2;
	} else {
		reg_load = dev->ext_lfo_freq;
	}

	reg_load /= sampling_freq;
	ret = adpd410x_reg_write(dev, ADPD410X_REG_TS_FREQ,
				 (reg_load & 0xFFFF));
	if(ret != SUCCESS)
		return FAILURE;
	return adpd410x_reg_write(dev, ADPD410X_REG_TS_FREQH,
				  ((reg_load & 0x7F0000) >> 16));
}

/**
 * @brief Setup an active time slot.
 * @param dev - Device handler.
 * @param timeslot_no - Time slot ID to setup.
 * @param init - Pointer to the time slot initialization structure.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_timeslot_setup(struct adpd410x_dev *dev,
				enum adpd410x_timeslots timeslot_no,
				struct adpd410x_timeslot_init *init)
{
	int32_t ret;
	uint16_t data;

	ret = adpd410x_reg_read(dev, ADPD410X_REG_OPMODE, &data);
	if(ret != SUCCESS)
		return ret;
	if(((data & BITM_OPMODE_TIMESLOT_EN) >> BITP_OPMODE_TIMESLOT_EN) <
	    timeslot_no)
		return FAILURE;

	/* Enable channel 2 */
	ret = adpd410x_reg_read(dev, (ADPD410X_REG_TS_CTRL_A + timeslot_no * 0x20),
				&data);
	if(ret != SUCCESS)
		return FAILURE;
	if(init->enable_ch2)
		data |= BITM_TS_CTRL_A_CH2_EN_A;
	else
		data &= ~BITM_TS_CTRL_A_CH2_EN_A;
	ret = adpd410x_reg_write(dev, (ADPD410X_REG_TS_CTRL_A + timeslot_no * 0x20),
				 data);
	if(ret != SUCCESS)
		return FAILURE;

	/* Setup inputs */
	data = init->ts_inputs.option << (init->ts_inputs.pair * 4);
	ret = adpd410x_reg_write(dev, (ADPD410X_REG_INPUTS_A + timeslot_no * 0x20),
				 data);
	if(ret != SUCCESS)
		return FAILURE;

	/* Set precondition PD */
	ret = adpd410x_reg_read(dev, (ADPD410X_REG_CATHODE_A + timeslot_no * 0x20),
				&data);
	if(ret != SUCCESS)
		return FAILURE;
	data &= ~BITM_CATHODE_A_PRECON_A;
	data |= (init->precon_option << BITP_CATHODE_A_PRECON_A) &
		BITM_CATHODE_A_PRECON_A;
	ret = adpd410x_reg_write(dev, (ADPD410X_REG_CATHODE_A + timeslot_no * 0x20),
				 data);
	if(ret != SUCCESS)
		return FAILURE;

	/**
	 *  Set TIA VREF and TRIM options. The 0xE000 is writing reserved bits
	 *  as specified in the datasheet.
	 */
	data = init->afe_trim_opt << BITP_AFE_TRIM_A_AFE_TRIM_VREF_A |
	       init->vref_pulse_opt << BITP_AFE_TRIM_A_VREF_PULSE_VAL_A |
	       init->chan2 << BITP_AFE_TRIM_A_TIA_GAIN_CH2_A |
	       init->chan1 << BITP_AFE_TRIM_A_TIA_GAIN_CH1_A;
	ret = adpd410x_reg_write(dev,
				 (ADPD410X_REG_AFE_TRIM_A + timeslot_no * 0x20), data);
	if(ret != SUCCESS)
		return FAILURE;

	/* Set LED pattern */
	data = init->pulse4_subtract << BITP_PATTERN_A_SUBTRACT_A |
	       init->pulse4_reverse << BITP_PATTERN_A_REVERSE_INTEG_A;
	ret = adpd410x_reg_write(dev, (ADPD410X_REG_PATTERN_A + timeslot_no * 0x20),
				 data);
	if(ret != SUCCESS)
		return FAILURE;

	/* Set bytes number for time slot */
	data = (init->byte_no << BITP_DATA1_A_SIGNAL_SIZE_A) &
	       BITM_DATA1_A_SIGNAL_SIZE_A;
	ret = adpd410x_reg_write(dev, (ADPD410X_REG_DATA1_A + timeslot_no * 0x20),
				 data);
	if(ret != SUCCESS)
		return FAILURE;

	/* Set decimate factor */
	data = (init->dec_factor << BITP_DECIMATE_A_DECIMATE_FACTOR_A) &
	       BITM_DECIMATE_A_DECIMATE_FACTOR_A;
	ret = adpd410x_reg_write(dev,
				 (ADPD410X_REG_DECIMATE_A + timeslot_no * 0x20), data);
	if(ret != SUCCESS)
		return FAILURE;

	/* Set LED power */
	data = init->led1.value |
	       (init->led2.value << BITP_LED_POW12_A_LED_CURRENT2_A);
	ret = adpd410x_reg_write(dev,
				 (ADPD410X_REG_LED_POW12_A + timeslot_no * 0x20), data);
	if(ret != SUCCESS)
		return FAILURE;
	data = init->led3.value |
	       (init->led4.value << BITP_LED_POW34_A_LED_CURRENT4_A);
	ret = adpd410x_reg_write(dev,
				 (ADPD410X_REG_LED_POW34_A + timeslot_no * 0x20), data);
	if(ret != SUCCESS)
		return FAILURE;

	/* Set ADC cycle and repeat */
	if(init->adc_cycles == 0)
		init->adc_cycles = 1;
	if(init->repeats_no == 0)
		init->repeats_no = 1;
	data = init->repeats_no | (init->adc_cycles << BITP_COUNTS_A_NUM_INT_A);
	return adpd410x_reg_write(dev, (ADPD410X_REG_COUNTS_A + timeslot_no * 0x20),
				  data);
}

/**
 * @brief Get number of bytes in the device FIFO.
 * @param dev - Device handler.
 * @param bytes - Pointer to the byte count container.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_get_fifo_bytecount(struct adpd410x_dev *dev, uint16_t *bytes)
{
	int32_t ret;

	ret = adpd410x_reg_read(dev, ADPD410X_REG_FIFO_STATUS, bytes);
	if(ret != SUCCESS)
		return FAILURE;

	*bytes &= BITM_INT_STATUS_FIFO_FIFO_BYTE_COUNT;

	return SUCCESS;
}

/**
 * @brief Get a data packet containing data from all active time slots and
 *        channels. adpd410x_get_data() helper function.
 * @param dev - Device handler.
 * @param data - Pointer to the data container.
 * @param no_slots - Number of active time slots.
 * @param dual_chan - Mask showing which of the active time slots are dual
 *                    channeled and which are single channeled.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
static int32_t adpd410x_get_data_packet(struct adpd410x_dev *dev,
					uint32_t *data, uint8_t no_slots, uint16_t dual_chan)
{
	int32_t ret;
	uint16_t data_byte_count, temp_data, expect_byte_count = 0;
	uint16_t *data_buff, sample_index = 0;
	int8_t i, got_one = 0;
	uint8_t *slot_bytecount_tab, *data_byte_buff;

	slot_bytecount_tab = (uint8_t *)calloc(no_slots,
					       sizeof (*slot_bytecount_tab));
	if (!slot_bytecount_tab)
		return FAILURE;
	for(i = 0; i < no_slots; i++) {
		ret = adpd410x_reg_read(dev, (ADPD410X_REG_DATA1_A + i * 0x20),
					&temp_data);
		if(ret != SUCCESS)
			goto error_slot;
		slot_bytecount_tab[i] = (temp_data &
					 BITM_DATA1_A_SIGNAL_SIZE_A);
		expect_byte_count += slot_bytecount_tab[i];
		if((dual_chan & (1 << i)) != 0)
			expect_byte_count += slot_bytecount_tab[i];
	}
	if((expect_byte_count % 2) != 0)
		expect_byte_count++;
	data_buff = (uint16_t *)calloc((expect_byte_count / 2),
				       sizeof (*data_buff));
	if (!data_buff)
		goto error_slot;
	data_byte_buff = (uint8_t *)calloc(expect_byte_count,
					   sizeof (*data_byte_buff));
	if (!data_byte_buff)
		goto error_data_buff;

	for(i = 0; i < (expect_byte_count / 2); i++) {
		ret = adpd410x_reg_read(dev, ADPD410X_REG_FIFO_DATA,
					(data_buff + i));
		if(ret != SUCCESS)
			goto error_data_byte;
	}
	for(i = 0; i < (expect_byte_count / 2); i++) {
		data_byte_buff[(i * 2)] = (data_buff[i] & 0xff00) >> 8;
		data_byte_buff[(i * 2 + 1)] = data_buff[i] & 0xff;
	}

	i = 0;
	data_byte_count = 0;
	do {
		switch(slot_bytecount_tab[i]) {
		case 0:
			continue;
		case 1:
			data[sample_index] = data_byte_buff[data_byte_count];
			data_byte_count++;
			break;
		case 2:
			data[sample_index] = data_byte_buff[data_byte_count] <<
					     8;
			data_byte_count++;
			data[sample_index] |= data_byte_buff[data_byte_count];
			data_byte_count++;
			break;
		case 3:
			data[sample_index] = data_byte_buff[data_byte_count] <<
					     8;
			data_byte_count++;
			data[sample_index] |= data_byte_buff[data_byte_count];
			data_byte_count++;
			data[sample_index] |= data_byte_buff[data_byte_count] <<
					      16;
			data_byte_count++;
			break;
		case 4:
			data[sample_index] = data_byte_buff[data_byte_count] <<
					     8;
			data_byte_count++;
			data[sample_index] |= data_byte_buff[data_byte_count];
			data_byte_count++;
			data[sample_index] |= data_byte_buff[data_byte_count] <<
					      24;
			data_byte_count++;
			data[sample_index] |= data_byte_buff[data_byte_count] <<
					      16;
			data_byte_count++;
			break;
		}
		sample_index++;
		if(((dual_chan & (1 << i)) != 0) && (got_one == 0)) {
			got_one = 1;
		} else {
			i++;
			got_one = 0;
		}
	} while(i < no_slots);

	free(data_byte_buff);
	free(data_buff);
	free(slot_bytecount_tab);

	return SUCCESS;

error_data_byte:
	free(data_byte_buff);
error_data_buff:
	free(data_buff);
error_slot:
	free(slot_bytecount_tab);

	return FAILURE;
}

/**
 * @brief Get a full data packet from the device containing data from all active
 *        time slots.
 * @param dev - Device handler.
 * @param data - Pointer to the data container.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_get_data(struct adpd410x_dev *dev, uint32_t *data)
{
	int32_t ret;
	int8_t i;
	uint16_t temp_data, dual_chan = 0;
	uint8_t ts_no;

	ret = adpd410x_reg_read(dev, ADPD410X_REG_OPMODE, &temp_data);
	if(ret != SUCCESS)
		return ret;
	ts_no = ((temp_data & BITM_OPMODE_TIMESLOT_EN) >>
		 BITP_OPMODE_TIMESLOT_EN) + 1;

	for(i = 0; i < ts_no; i++) {
		ret = adpd410x_reg_read(dev, (ADPD410X_REG_TS_CTRL_A + i * 0x20),
					&temp_data);
		if(ret != SUCCESS)
			return ret;
		if((temp_data & BITM_TS_CTRL_A_CH2_EN_A) != 0)
			dual_chan |= (1 << i);
	}

	return adpd410x_get_data_packet(dev, data, ts_no, dual_chan);
}

/**
 * @brief Setup the device and the driver.
 * @param device - Pointer to the device handler.
 * @param init_param - Pointer to the initialization structure.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_setup(struct adpd410x_dev **device,
		       struct adpd410x_init_param *init_param)
{
	int32_t ret;
	struct adpd410x_dev *dev;
	uint16_t reg_temp;

	dev = calloc(1, sizeof *dev);
	if(!dev)
		return FAILURE;

	dev->dev_type = init_param->dev_type;
	dev->clk_opt = init_param->clk_opt;
	dev->ext_lfo_freq = init_param->ext_lfo_freq;

	if(dev->dev_type == ADPD4100)
		ret = spi_init(&dev->dev_ops.spi_phy_dev,
			       &init_param->dev_ops_init.spi_phy_init);
	else
		ret = i2c_init(&dev->dev_ops.i2c_phy_dev,
			       &init_param->dev_ops_init.i2c_phy_init);
	if(ret != SUCCESS)
		goto error_dev;

	ret = gpio_get(&dev->gpio0, &init_param->gpio0);
	if(ret != SUCCESS)
		goto error_phy;
	ret = gpio_get(&dev->gpio1, &init_param->gpio1);
	if(ret != SUCCESS)
		goto error_gpio0;
	ret = gpio_get(&dev->gpio2, &init_param->gpio2);
	if(ret != SUCCESS)
		goto error_gpio1;
	ret = gpio_get(&dev->gpio3, &init_param->gpio3);
	if(ret != SUCCESS)
		goto error_gpio2;

	ret = adpd410x_reset(dev);
	if(ret != SUCCESS)
		goto error_gpio3;

	/* Do power-up sequence described in errata. */
	ret = gpio_direction_output(dev->gpio0, GPIO_HIGH);
	if(ret != SUCCESS)
		goto error_gpio3;
	ret = adpd410x_reg_write(dev, 0xB5, 0x04);
	if(ret != SUCCESS)
		goto error_gpio3;
	ret = adpd410x_reg_write(dev, 0xB5, 0x00);
	if(ret != SUCCESS)
		goto error_gpio3;
	ret = gpio_tristate(dev->gpio0);
	if(ret != SUCCESS)
		goto error_gpio3;

	ret = adpd410x_reg_read(dev, ADPD410X_REG_CHIP_ID, &reg_temp);
	if(ret != SUCCESS)
		goto error_gpio3;
	if(reg_temp != 0x00C2)
		goto error_gpio3;

	ret = adpd410x_reg_read(dev, ADPD410X_REG_SYS_CTL, &reg_temp);
	if(ret != SUCCESS)
		goto error_gpio3;
	reg_temp |= (dev->clk_opt << BITP_SYS_CTL_ALT_CLOCKS) &
		    BITM_SYS_CTL_ALT_CLOCKS;
	ret = adpd410x_reg_write(dev, ADPD410X_REG_SYS_CTL, reg_temp);
	if(ret != SUCCESS)
		goto error_gpio3;

	/**
	 * Enable the 1MHz oscillator if the internal low frequency oscillator
	 * is used.
	 */
	if((dev->clk_opt == ADPD410X_INTLFO_INTHFO) ||
	    (dev->clk_opt == ADPD410X_INTLFO_EXTHFO)) {
		ret = adpd410x_reg_read(dev, ADPD410X_REG_SYS_CTL, &reg_temp);
		if(ret != SUCCESS)
			goto error_gpio3;
		reg_temp |= (BITM_SYS_CTL_OSC_1M_EN | BITM_SYS_CTL_LFOSC_SEL);
		ret = adpd410x_reg_write(dev, ADPD410X_REG_SYS_CTL, reg_temp);
		if(ret != SUCCESS)
			goto error_gpio3;
	}

	*device = dev;

	return SUCCESS;

error_gpio3:
	gpio_remove(dev->gpio3);
error_gpio2:
	gpio_remove(dev->gpio2);
error_gpio1:
	gpio_remove(dev->gpio1);
error_gpio0:
	gpio_remove(dev->gpio0);
error_phy:
	if(dev->dev_type == ADPD4100)
		spi_remove(dev->dev_ops.spi_phy_dev);
	else
		i2c_remove(dev->dev_ops.i2c_phy_dev);
error_dev:
	free(dev);

	return FAILURE;
}

/**
 * @brief Free memory allocated by adpd410x_setup().
 * @param dev - Device handler.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t adpd410x_remove(struct adpd410x_dev *dev)
{
	int32_t ret;

	if(!dev)
		return FAILURE;

	if(dev->dev_type == ADPD4100)
		ret = spi_remove(dev->dev_ops.spi_phy_dev);
	else
		ret = i2c_remove(dev->dev_ops.i2c_phy_dev);
	if(ret != SUCCESS)
		return FAILURE;

	ret = gpio_remove(dev->gpio0);
	if(ret != SUCCESS)
		return FAILURE;
	ret = gpio_remove(dev->gpio1);
	if(ret != SUCCESS)
		return FAILURE;
	ret = gpio_remove(dev->gpio2);
	if(ret != SUCCESS)
		return FAILURE;
	ret = gpio_remove(dev->gpio3);
	if(ret != SUCCESS)
		return FAILURE;

	free(dev);

	return SUCCESS;
}
