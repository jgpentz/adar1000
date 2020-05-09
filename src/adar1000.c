/**
 * MIT License
 * 
 * Copyright (c) 2020 Jimmy Pentz
 *
 * Reach me at: github.com/jgpentz, jpentz1(at)gmail.com
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sells
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/* ADAR1000 4-Channel, X Band and Ku Band Beamformer */
// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include "adar1000.h"


// ----------------------------------------------------------------------------
// Preprocessor Definitions and Constants
// ----------------------------------------------------------------------------
// VM_GAIN is 15 dB of gain in 128 steps. ~0.12 dB per step.
// A 15 dB attenuator can be applied on top of these values.
const uint8_t VM_GAIN[128] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
};

// VM_I and VM_Q are the settings for the vector modulator. 128 steps in 360 degrees. ~2.813 degrees per step.
const uint8_t VM_I[128] = {
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3E, 0x3E, 0x3D, 0x3D, 0x3C, 0x3C, 0x3B, 0x3A, 0x39, 0x38, 0x37,
	0x36, 0x35, 0x34, 0x33, 0x32, 0x30, 0x2F, 0x2E, 0x2C, 0x2B, 0x2A, 0x28, 0x27, 0x25, 0x24, 0x22,
	0x21, 0x01, 0x03, 0x04, 0x06, 0x07, 0x08, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x11, 0x12, 0x13, 0x14,
	0x16, 0x17, 0x18, 0x19, 0x19, 0x1A, 0x1B, 0x1C, 0x1C, 0x1D, 0x1E, 0x1E, 0x1E, 0x1F, 0x1F, 0x1F,
	0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1E, 0x1E, 0x1D, 0x1D, 0x1C, 0x1C, 0x1B, 0x1A, 0x19, 0x18, 0x17,
	0x16, 0x15, 0x14, 0x13, 0x12, 0x10, 0x0F, 0x0E, 0x0C, 0x0B, 0x0A, 0x08, 0x07, 0x05, 0x04, 0x02,
	0x01, 0x21, 0x23, 0x24, 0x26, 0x27, 0x28, 0x2A, 0x2B, 0x2D, 0x2E, 0x2F, 0x31, 0x32, 0x33, 0x34,
	0x36, 0x37, 0x38, 0x39, 0x39, 0x3A, 0x3B, 0x3C, 0x3C, 0x3D, 0x3E, 0x3E, 0x3E, 0x3F, 0x3F, 0x3F,
};

const uint8_t VM_Q[128] = {
	0x20, 0x21, 0x23, 0x24, 0x26, 0x27, 0x28, 0x2A, 0x2B, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x33, 0x34,
	0x35, 0x36, 0x37, 0x38, 0x38, 0x39, 0x3A, 0x3A, 0x3B, 0x3C, 0x3C, 0x3C, 0x3D, 0x3D, 0x3D, 0x3D,
	0x3D, 0x3D, 0x3D, 0x3D, 0x3D, 0x3C, 0x3C, 0x3C, 0x3B, 0x3A, 0x3A, 0x39, 0x38, 0x38, 0x37, 0x36,
	0x35, 0x34, 0x33, 0x31, 0x30, 0x2F, 0x2E, 0x2D, 0x2B, 0x2A, 0x28, 0x27, 0x26, 0x24, 0x23, 0x21,
	0x20, 0x01, 0x03, 0x04, 0x06, 0x07, 0x08, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x13, 0x14,
	0x15, 0x16, 0x17, 0x18, 0x18, 0x19, 0x1A, 0x1A, 0x1B, 0x1C, 0x1C, 0x1C, 0x1D, 0x1D, 0x1D, 0x1D,
	0x1D, 0x1D, 0x1D, 0x1D, 0x1D, 0x1C, 0x1C, 0x1C, 0x1B, 0x1A, 0x1A, 0x19, 0x18, 0x18, 0x17, 0x16,
	0x15, 0x14, 0x13, 0x11, 0x10, 0x0F, 0x0E, 0x0D, 0x0B, 0x0A, 0x08, 0x07, 0x06, 0x04, 0x03, 0x01,
};


// ----------------------------------------------------------------------------
// Function Definitions
// ----------------------------------------------------------------------------
/**
 * @brief	Initialize the ADC on the ADAR by setting the ADC with a 2 MHz clk, 
 *			and then enable it.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @warning	This is setup to only read temperature sensor data, not the power detectors.
 */
void Adar_AdcInit(const AdarDevice * p_adar, uint8_t broadcast)
{
	uint8_t data;

	data = ADAR1000_ADC_2MHZ_CLK | ADAR1000_ADC_EN;

	Adar_Write(p_adar, REG_ADC_CONTROL, data, broadcast);
}


/**
 * @brief	Read a byte of data from the ADAR.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @return	Returns a byte of data that has been converted from the temperature sensor.
 *
 * @warning	This is setup to only read temperature sensor data, not the power detectors.
 */
uint8_t Adar_AdcRead(const AdarDevice * p_adar, uint8_t broadcast)
{
	uint8_t data;

    // Start the ADC conversion
	Adar_Write(p_adar, REG_ADC_CONTROL, ADAR1000_ADC_ST_CONV, broadcast);

    // This is blocking for now... wait until data is converted, then read it
	while (!(Adar_Read(p_adar, REG_ADC_CONTROL) & 0x01))
	{
	}

	data = Adar_Read(p_adar, REG_ADC_OUT);

	return(data);
}


/**
 * @brief	Requests the device info from a specific ADAR and stores it in the
 *			provided AdarDeviceInfo struct.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param   info[out]	Struct that contains the device info fields.
 *
 * @return	Returns ADAR_ERROR_NOERROR if information was successfully received and stored in the struct.
 */
uint8_t Adar_GetDeviceInfo(const AdarDevice * p_adar, AdarDeviceInfo * info)
{
	*((uint8_t *)info) = Adar_Read(p_adar, 0x002);
	info->chip_type = Adar_Read(p_adar, 0x003);
	info->product_id = ((uint16_t)Adar_Read(p_adar, 0x004)) << 8;
	info->product_id |= ((uint16_t)Adar_Read(p_adar, 0x005)) & 0x00ff;
	info->scratchpad = Adar_Read(p_adar, 0x00A);
	info->spi_rev = Adar_Read(p_adar, 0x00B);
	info->vendor_id = ((uint16_t)Adar_Read(p_adar, 0x00C)) << 8;
	info->vendor_id |= ((uint16_t)Adar_Read(p_adar, 0x00D)) & 0x00ff;
	info->rev_id = Adar_Read(p_adar, 0x045);

	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief	Read the data that is stored in a single ADAR register.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param	mem_addr	Memory address of the register you wish to read from.
 *
 * @return	Returns the byte of data that is stored in the desired register.
 *
 * @warning	This function will clear ADDR_ASCN bits.
 * @warning The ADAR does not allow for block reads.
 */
uint8_t Adar_Read(const AdarDevice * p_adar, uint32_t mem_addr)
{
	uint8_t instruction[3];

    // Set SDO active
	Adar_Write(p_adar, REG_INTERFACE_CONFIG_A, INTERFACE_CONFIG_A_SDO_ACTIVE, 0);

	instruction[0] = 0x80 | ((p_adar->dev_addr & 0x03) << 5);
	instruction[0] |= ((0xff00 & mem_addr) >> 8);
	instruction[1] = (0xff & mem_addr);
	instruction[2] = 0x00;

	p_adar->Transfer(instruction, p_adar->p_rx_buffer, ADAR1000_RD_SIZE);

    // Set SDO Inactive
	Adar_Write(p_adar, REG_INTERFACE_CONFIG_A, 0, 0);

	return(p_adar->p_rx_buffer[2]);
}


/**
 * @brief	Block memory write to an ADAR device.
 *
 * @pre		ADDR_ASCN bits in register zero must be set!
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param	mem_addr  	Memory address of the register you wish to read from.
 * @param	p_data    	Pointer to block of data to transfer (must have two unused bytes preceding the data for instruction).
 * @param	size      	Size of data in bytes, including the two additional leading bytes.
 *
 * @warning First two bytes of data will be corrupted if you do not provide two unused leading bytes!
 */
void Adar_ReadBlock(const AdarDevice * p_adar, uint16_t mem_addr, uint8_t * p_data, uint32_t size)
{
    // Set SDO active
	Adar_Write(p_adar, REG_INTERFACE_CONFIG_A, INTERFACE_CONFIG_A_SDO_ACTIVE | INTERFACE_CONFIG_A_ADDR_ASCN, 0);

    // Prepare command
	p_data[0] = 0x80 | ((p_adar->dev_addr & 0x03) << 5);
	p_data[0] |= ((mem_addr) >> 8) & 0x1F;
	p_data[1] = (0xFF & mem_addr);

    // Start the transfer
	p_adar->Transfer(p_data, p_data, size);

	Adar_Write(p_adar, REG_INTERFACE_CONFIG_A, 0, 0);
    // Return nothing since we assume this is non-blocking and won't wait around
}


/**
 * @brief	Sets the Rx/Tx bias currents for the LNA, VM, and VGA to be in either
 *			low power setting or nominal setting.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param	p_bias[in]	An AdarBiasCurrents struct filled with bias settings
 *						as seen in the datasheet Table 6. SPI Settings for 
 *						Different Power	Modules
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @return	Returns ADAR_ERR_NOERROR if the bias currents were set 
 */
uint8_t Adar_SetBiasCurrents(const AdarDevice * p_adar, AdarBiasCurrents * p_bias, uint8_t broadcast)
{
	uint8_t bias = 0;

    // RX LNA/VGA/VM bias
	bias = (p_bias->rx_lna & 0x0f);
	Adar_Write(p_adar, REG_BIAS_CURRENT_RX_LNA, bias, broadcast); // RX LNA bias
	bias = (p_bias->rx_vga & 0x07 << 3) | (p_bias->rx_vm & 0x07);
	Adar_Write(p_adar, REG_BIAS_CURRENT_RX, bias, broadcast);     // RX VM/VGA bias

    // TX VGA/VM/DRV bias
	bias = (p_bias->tx_vga & 0x07 << 3) | (p_bias->tx_vm & 0x07);
	Adar_Write(p_adar, REG_BIAS_CURRENT_TX, bias, broadcast);     // TX VM/VGA bias
	bias = (p_bias->tx_drv & 0x07);
	Adar_Write(p_adar, REG_BIAS_CURRENT_TX_DRV, bias, broadcast); // TX DRV bias
	
	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief	Set the bias ON and bias OFF voltages for the four PA's and one LNA.
 *
 * @pre		This will set all 5 bias ON values and all 5 bias OFF values at once.
 * 			To enable these bias values, please see the data sheet and ensure that the BIAS_CTRL,
 * 			LNA_BIAS_OUT_EN, TR_SOURCE, TX_EN, RX_EN, TR (input to chip), and PA_ON (input to chip)
 * 			bits have all been properly set.
 *
 * @param	p_adar[in]		Adar pointer Which specifies the device and what function 
 *							to use for SPI transfer.
 * @param 	bias_on_voltage   Array that contains the bias ON voltages.
 * @param 	bias_off_voltage  Array that contains the bias OFF voltages.
 *
 * @return	Returns ADAR_ERR_NOERROR if the bias currents were set 
 */
uint8_t Adar_SetBiasVoltages(const AdarDevice * p_adar, uint8_t bias_on_voltage[5], uint8_t bias_off_voltage[5])
{
	uint8_t buffer[7];

    // Set PA bias DAC output
	memcpy(&buffer[2], bias_on_voltage, sizeof(buffer) - 2);
	Adar_WriteBlock(p_adar, REG_PA_CH1_BIAS_ON, buffer, sizeof(buffer));
	memcpy(&buffer[2], bias_off_voltage, sizeof(buffer) - 2);
	Adar_WriteBlock(p_adar, REG_PA_CH1_BIAS_OFF, buffer, sizeof(buffer));
	
	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief	Setup the ADAR to use settings that are transferred over SPI.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @return	Returns ADAR_ERR_NOERROR if the bias currents were set 
 */
uint8_t Adar_SetRamBypass(const AdarDevice * p_adar, uint8_t broadcast)
{
	uint8_t data;

	data = (MEM_CTRL_BIAS_RAM_BYPASS | MEM_CTRL_BEAM_RAM_BYPASS);

	Adar_Write(p_adar, REG_MEM_CTL, data, broadcast);

	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief	Set the VGA gain value of a Receive channel in dB.
 *
 * @param	p_adar[in]		Adar pointer Which specifies the device and what function 
 *							to use for SPI transfer.
 * @param 	channel			Channel in which to set the gain (1-4).
 * @param 	vga_gain_db		Gain to be applied to the channel, ranging from 0 - 30 dB. 
 *							(Intended operation >16 dB).
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @return 	Returns ADAR_ERROR_NOERROR if the gain was successfully set. 
 * 					ADAR_ERROR_FAILED if an invalid channel was selected.
 *
 * @warning 0 dB or 15 dB step attenuator may also be turned on, which is why intended operation is >16 dB.
 */
uint8_t Adar_SetRxVgaGain(const AdarDevice * p_adar, uint8_t channel, uint8_t vga_gain_db, uint8_t broadcast)
{
	uint8_t	 vga_gain_bits = 0;
	uint32_t mem_addr = 0;

	if((channel == 0) || (channel > 4))
	{
		return(ADAR_ERROR_FAILED);
	}

	mem_addr = REG_CH1_RX_GAIN + (channel & 0x03);

    // Set gain
	Adar_Write(p_adar, mem_addr, vga_gain_bits, broadcast);

    // Load the new setting
	Adar_Write(p_adar, REG_LOAD_WORKING, 0x1, broadcast);

	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief	Set the phase of a given receive channel using the I/Q vector modulator.
 *
 * @pre		According to the given @param phase, this sets the polarity (bit 5) and gain (bits 4-0)
 * 			of the @param channel, and then loads them into the working register.
 * 			A vector modulator I/Q look-up table has been provided at the beginning of this library.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param	channel   	Channel in which to set the gain (1-4).
 * @param	phase     	Byte that is used to set the polarity (bit 5) and gain (bits 4-0).
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @return 	Returns ADAR_ERROR_NOERROR if the phase was successfully set. 
 * 					ADAR_ERROR_FAILED if an invalid channel was selected.
 *
 * @note    To obtain your phase:
 *          phase = degrees * 128;
 *          phase /= 360;
 */
uint8_t Adar_SetRxPhase(const AdarDevice * p_adar, uint8_t channel, uint8_t phase, uint8_t broadcast)
{
	uint8_t	 i_val = 0;
	uint8_t	 q_val = 0;
	uint32_t mem_addr_i, mem_addr_q;

	if((channel == 0) || (channel > 4))
	{
		return(ADAR_ERROR_FAILED);
	}

	phase = phase % 128;
	i_val = VM_I[phase];
	q_val = VM_Q[phase];

	mem_addr_i = REG_CH1_RX_PHS_I + (channel & 0x03) * 2;
	mem_addr_q = REG_CH1_RX_PHS_Q + (channel & 0x03) * 2;

	Adar_Write(p_adar, mem_addr_i, i_val, broadcast);
	Adar_Write(p_adar, mem_addr_q, q_val, broadcast);
	Adar_Write(p_adar, REG_LOAD_WORKING, 0x1, broadcast);

	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief	Set the VGA gain value of a Tx channel in dB.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @return 	Returns ADAR_ERROR_NOERROR if the bias was successfully set. 
 * 					ADAR_ERROR_FAILED if an invalid channel was selected.
 *
 * @warning 0 dB or 15 dB step attenuator may also be turned on, which is why intended operation is >16 dB.
 */
uint8_t Adar_SetTxBias(const AdarDevice * p_adar, uint8_t broadcast)
{
	uint8_t	 vga_bias_bits;
	uint8_t	 drv_bias_bits;
	uint32_t mem_vga_bias;
	uint32_t mem_drv_bias;

	mem_vga_bias = REG_BIAS_CURRENT_TX;
	mem_drv_bias = REG_BIAS_CURRENT_TX_DRV;

    // Set bias to nom
	vga_bias_bits = 0x2D;
	drv_bias_bits = 0x06;

    // Set bias
	Adar_Write(p_adar, mem_vga_bias, vga_bias_bits, broadcast);
    // Set bias
	Adar_Write(p_adar, mem_drv_bias, drv_bias_bits, broadcast);

    // Load the new setting
	Adar_Write(p_adar, REG_LOAD_WORKING, 0x2, broadcast);

	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief	Set the VGA gain value of a Tx channel.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	channel     Tx channel in which to set the gain, ranging from 1 - 4.
 * @param 	gain   		Gain to be applied to the channel, ranging from 0 - 127,
 *						plus the MSb 15dB attenuator (Intended operation >16 dB).
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @return 	Returns ADAR_ERROR_NOERROR if the gain was successfully set. 
 * 					ADAR_ERROR_FAILED if an invalid channel was selected.
 *
 * @warning 0 dB or 15 dB step attenuator may also be turned on, which is why intended operation is >16 dB.
 */
uint8_t Adar_SetTxVgaGain(const AdarDevice * p_adar, uint8_t channel, uint8_t gain, uint8_t broadcast)
{
	uint32_t mem_addr;
	
	if((channel == 0) || (channel > 4))
	{
		return(ADAR_ERROR_FAILED);
	}

	mem_addr = REG_CH1_TX_GAIN + (channel & 0x03);

    // Set gain
	Adar_Write(p_adar, mem_addr, gain, broadcast);

    // Load the new setting
	Adar_Write(p_adar, REG_LOAD_WORKING, LD_WRK_REGS_LDTX_OVERRIDE, broadcast);

	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief 	Set the phase of a given transmit channel using the I/Q vector modulator.
 *
 * @pre		According to the given @param phase, this sets the polarity (bit 5) and gain (bits 4-0)
 * 			of the @param channel, and then loads them into the working register.
 * 			A vector modulator I/Q look-up table has been provided at the beginning of this library.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	channel   	Channel in which to set the gain (1-4).
 * @param 	phase     	Byte that is used to set the polarity (bit 5) and gain (bits 4-0).
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 *
 * @return 	Returns ADAR_ERROR_NOERROR if the phase was successfully set. 
 * 					ADAR_ERROR_FAILED if an invalid channel was selected.
 *
 * @note    To obtain your phase:
 *          phase = degrees * 128;
 *          phase /= 360;
 */
uint8_t Adar_SetTxPhase(const AdarDevice * p_adar, uint8_t channel, uint8_t phase, uint8_t broadcast)
{
	uint8_t	 i_val = 0;
	uint8_t	 q_val = 0;
	uint32_t mem_addr_i, mem_addr_q;

	if((channel == 0) || (channel > 4))
	{
		return(ADAR_ERROR_FAILED);
	}

	phase = phase % 128;
	i_val = VM_I[phase];
	q_val = VM_Q[phase];

	mem_addr_i = REG_CH1_TX_PHS_I + (channel & 0x03) * 2;
	mem_addr_q = REG_CH1_TX_PHS_Q + (channel & 0x03) * 2;

	Adar_Write(p_adar, mem_addr_i, i_val, broadcast);
	Adar_Write(p_adar, mem_addr_q, q_val, broadcast);
	Adar_Write(p_adar, REG_LOAD_WORKING, 0x1, broadcast);

	return(ADAR_ERROR_NOERROR);
}


/**
 * @brief 	Reset the whole ADAR device.
 *
 * @param	p_adar[in]	ADAR pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 */
void Adar_SoftReset(const AdarDevice * p_adar)
{
	uint8_t instruction[3];

	instruction[0] = ((p_adar->dev_addr & 0x03) << 5);
	instruction[1] = 0x00;
	instruction[2] = 0x81;

	p_adar->Transfer(instruction, NULL, sizeof(instruction));
}


/**
 * @brief 	Reset ALL ADAR devices in the SPI chain.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 */
void Adar_SoftResetAll(const AdarDevice * p_adar)
{
	uint8_t instruction[3];

	instruction[0] = 0x08;
	instruction[1] = 0x00;
	instruction[2] = 0x81;

	p_adar->Transfer(instruction, NULL, sizeof(instruction));
}


/**
 * @brief 	Write a byte of @param data to the register located at @param mem_addr.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	mem_addr  	Memory address of the register you wish to read from.
 * @param 	data      	Byte of data to be stored in the register.
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 						if this set to BROADCAST_ON.
 *
 * @warning If writing the same data to multiple registers, use ADAR_WriteBlock.
 */
void Adar_Write(const AdarDevice * p_adar, uint32_t mem_addr, uint8_t data, uint8_t broadcast)
{
	uint8_t instruction[3];
	
	if (broadcast)
	{
		instruction[0] = 0x08;
	}
	else
	{
		instruction[0] = ((p_adar->dev_addr & 0x03) << 5);
	}

	instruction[0] |= (0x1F00 & mem_addr) >> 8;
	instruction[1] = (0xFF & mem_addr);
	instruction[2] = data;

	p_adar->Transfer(instruction, NULL, sizeof(instruction));
}


/**
 * @brief 	Block memory write to an ADAR device.
 *
 * @pre 	ADDR_ASCN BITS IN REGISTER ZERO MUST BE SET!
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param	mem_addr 	Memory address of the register you wish to read from.
 * @param	p_data[in]  Pointer to block of data to transfer (must have two unused bytes 
 						preceding the data for instruction).
 * @param	size      	Size of data in bytes, including the two additional leading bytes.
 *
 * @warning First two bytes of data will be corrupted if you do not provide two unused leading bytes!
 */
void Adar_WriteBlock(const AdarDevice * p_adar, uint16_t mem_addr, uint8_t * p_data, uint32_t size)
{
    // Prepare command
	p_data[0] = ((p_adar->dev_addr & 0x03) << 5);
	p_data[0] |= ((mem_addr) >> 8) & 0x1F;
	p_data[1] = (0xFF & mem_addr);

    // Start the transfer
	p_adar->Transfer(p_data, NULL, size);

    // Return nothing since we assume this is non-blocking and won't wait around
}


/**
 * @brief	Set contents of the INTERFACE_CONFIG_A register.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	flags     	#INTERFACE_CONFIG_A_SOFTRESET, #INTERFACE_CONFIG_A_LSB_FIRST,
 *                  	#INTERFACE_CONFIG_A_ADDR_ASCN, #INTERFACE_CONFIG_A_SDO_ACTIVE
 * @param 	broadcast	Send the message as a broadcast to all ADARs in the SPI chain
 *						if this set to BROADCAST_ON.
 */
void Adar_WriteConfigA(const AdarDevice * p_adar, uint8_t flags, uint8_t broadcast)
{
	Adar_Write(p_adar, 0x00, flags, broadcast);
}


/**
 * @brief 	Write a byte of @param data to the register located at @param mem_addr and
 *			then read from the device and verify that the register was correctly set.
 *
 * @param	p_adar[in]	Adar pointer Which specifies the device and what function 
 *						to use for SPI transfer.
 * @param 	mem_addr  	Memory address of the register you wish to read from.
 * @param 	data      	Byte of data to be stored in the register.
 *
 * @return	Returns the number of attempts that it took to successfully write to a register,
 *			starting from zero.
 * @warning This function currently only supports writes to a single regiter in a single ADAR.
 */
uint8_t Adar_WriteVerify(const AdarDevice * p_adar, uint32_t mem_addr, uint8_t data)
{
	uint8_t rx_data;

	for (uint8_t ii = 0; ii < 3; ii++)
	{
		Adar_Write(p_adar, mem_addr, data, 0);

		// Can't read back from an ADAR with HW address 0
		if (!((p_adar->dev_addr) % 4))
		{
			return(ADAR_ERROR_INVALIDADDR);
		}
		rx_data = Adar_Read(p_adar, mem_addr);
		if (rx_data == data)
		{
			return(ii);
		}
	}

	return(ADAR_ERROR_FAILED);
}

