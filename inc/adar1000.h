/**
 * MIT License
 * 
 * Copyright (c) 2020 Jimmy Pentz
 *
 * Reach me at: github.com/jgpentz, jpentz1( at )gmail.com
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

#ifndef LIB_ADAR1000_H_
#define LIB_ADAR1000_H_

#ifndef NULL
#define NULL    (0)
#endif

// ----------------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


// ----------------------------------------------------------------------------
// Datatypes
// ----------------------------------------------------------------------------
extern const uint8_t VM_GAIN[128];
extern const uint8_t VM_I[128];
extern const uint8_t VM_Q[128];

/// A function pointer prototype for a SPI transfer, the 3 parameters would be
/// p_txData, p_rxData, and size (number of bytes to transfer), respectively.
typedef uint32_t (* Adar_SpiTransfer)(uint8_t *, uint8_t *, uint32_t);

/// Generic ADAR device that contains a hardware address, SPI transfer function
/// and a pointer to a buffer to receive data into.
typedef struct
{
	uint8_t			 dev_addr; 		///< 2-bit device hardware address, 0x00, 0x01, 0x10, 0x11
	Adar_SpiTransfer Transfer;		///< Function pointer to the function used for SPI transfers
	uint8_t *		 p_rx_buffer;	///< Data buffer to store received bytes into
} const AdarDevice;

/// Use this to store bias current values into, as seen in the datasheet 
/// Table 6. SPI Settings for Different Power Modules
typedef struct
{
	uint8_t rx_lna;		///< nominal:  8, low power: 5
	uint8_t rx_vm;		///< nominal:  5, low power: 2
	uint8_t rx_vga;		///< nominal: 10, low power: 3
	uint8_t tx_vm;		///< nominal:  5, low power: 2
	uint8_t tx_vga;		///< nominal:  5, low power: 5
	uint8_t tx_drv;		///< nominal:  6, low power: 3
} AdarBiasCurrents;

/// Useful for queries regarding the device info
typedef struct
{
	uint8_t	 norm_operating_mode : 2;
	uint8_t	 cust_operating_mode : 2;
	uint8_t	 dev_status : 4;
	uint8_t	 chip_type;
	uint16_t product_id;
	uint8_t	 scratchpad;
	uint8_t	 spi_rev;
	uint16_t vendor_id;
	uint8_t	 rev_id;
} AdarDeviceInfo;

/// Return types for functions in this library
typedef enum {
	ADAR_ERROR_NOERROR		= 0,
	ADAR_ERROR_FAILED		= 1,
	ADAR_ERROR_INVALIDADDR	= 2,
} AdarErrorCodes;


// ----------------------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------------------
void Adar_AdcInit(const AdarDevice * p_adar, uint8_t broadcast_bit);

uint8_t Adar_AdcRead(const AdarDevice * p_adar, uint8_t broadcast_bit);

uint8_t Adar_GetDeviceInfo(const AdarDevice * p_adar, AdarDeviceInfo * info);

uint8_t Adar_Read(const AdarDevice * p_adar, uint32_t mem_addr);

void Adar_ReadBlock(const AdarDevice * p_adar, uint16_t mem_addr, uint8_t * p_data, uint32_t size);

uint8_t Adar_SetBiasCurrents(const AdarDevice * p_adar, AdarBiasCurrents * p_bias, uint8_t broadcast_bit);

uint8_t Adar_SetBiasVoltages(const AdarDevice * p_adar, uint8_t bias_on_voltage[5], uint8_t bias_off_voltage[5]);

uint8_t Adar_SetRamBypass(const AdarDevice * p_adar, uint8_t broadcast_bit);

uint8_t Adar_SetRxVgaGain(const AdarDevice * p_adar, uint8_t channel, uint8_t vga_gain_db, uint8_t broadcast_bit);

uint8_t Adar_SetRxPhase(const AdarDevice * p_adar, uint8_t channel, uint8_t phase, uint8_t broadcast_bit);

uint8_t Adar_SetTxBias(const AdarDevice * p_adar, uint8_t broadcast_bit);

uint8_t Adar_SetTxVgaGain(const AdarDevice * p_adar, uint8_t channel, uint8_t vga_gain_db, uint8_t broadcast_bit);

uint8_t Adar_SetTxPhase(const AdarDevice * p_adar, uint8_t channel, uint8_t phase, uint8_t broadcast_bit);

void Adar_SoftReset(const AdarDevice * p_adar);

void Adar_SoftResetAll(const AdarDevice * p_adar);

void Adar_Write(const AdarDevice * p_adar, uint32_t mem_addr, uint8_t data, uint8_t broadcast_bit);

void Adar_WriteBlock(const AdarDevice * p_adar, uint16_t mem_addr, uint8_t * p_data, uint32_t size);

void Adar_WriteConfigA(const AdarDevice * p_adar, uint8_t flags, uint8_t broadcast);

uint8_t Adar_WriteVerify(const AdarDevice * p_adar, uint32_t mem_addr, uint8_t data);


// ----------------------------------------------------------------------------
// Preprocessor Definitions and Constants
// ----------------------------------------------------------------------------
// Using BROADCAST_ON will send a command to all ADARs that share a bus
#define BROADCAST_OFF					 0
#define BROADCAST_ON					 1

// The minimum size of a read from the ADARs consists of 3 bytes
#define ADAR1000_RD_SIZE				 3

// Address at which the TX RAM starts
#define ADAR_TX_RAM_START_ADDR			 0x1800

// ADC Defines
#define ADAR1000_ADC_2MHZ_CLK			 0x00
#define ADAR1000_ADC_EN					 0x60
#define ADAR1000_ADC_ST_CONV			 0x70

/* REGISTER DEFINITIONS */
#define REG_INTERFACE_CONFIG_A			 0x000
#define REG_INTERFACE_CONFIG_B			 0x001
#define REG_DEV_CONFIG					 0x002
#define REG_SCRATCHPAD					 0x00A
#define REG_TRANSFER					 0x00F
#define REG_CH1_RX_GAIN					 0x010
#define REG_CH2_RX_GAIN					 0x011
#define REG_CH3_RX_GAIN					 0x012
#define REG_CH4_RX_GAIN					 0x013
#define REG_CH1_RX_PHS_I				 0x014
#define REG_CH1_RX_PHS_Q				 0x015
#define REG_CH2_RX_PHS_I				 0x016
#define REG_CH2_RX_PHS_Q				 0x017
#define REG_CH3_RX_PHS_I				 0x018
#define REG_CH3_RX_PHS_Q				 0x019
#define REG_CH4_RX_PHS_I				 0x01A
#define REG_CH4_RX_PHS_Q				 0x01B
#define REG_CH1_TX_GAIN					 0x01C
#define REG_CH2_TX_GAIN					 0x01D
#define REG_CH3_TX_GAIN					 0x01E
#define REG_CH4_TX_GAIN					 0x01F
#define REG_CH1_TX_PHS_I				 0x020
#define REG_CH1_TX_PHS_Q				 0x021
#define REG_CH2_TX_PHS_I				 0x022
#define REG_CH2_TX_PHS_Q				 0x023
#define REG_CH3_TX_PHS_I				 0x024
#define REG_CH3_TX_PHS_Q				 0x025
#define REG_CH4_TX_PHS_I				 0x026
#define REG_CH4_TX_PHS_Q				 0x027
#define REG_LOAD_WORKING				 0x028
#define REG_PA_CH1_BIAS_ON				 0x029
#define REG_PA_CH2_BIAS_ON				 0x02A
#define REG_PA_CH3_BIAS_ON				 0x02B
#define REG_PA_CH4_BIAS_ON				 0x02C
#define REG_LNA_BIAS_ON					 0x02D
#define REG_RX_ENABLES					 0x02E
#define REG_TX_ENABLES					 0x02F
#define REG_MISC_ENABLES				 0x030
#define REG_SW_CONTROL					 0x031
#define REG_ADC_CONTROL					 0x032
#define REG_ADC_CONTROL_TEMP_EN			 0xf0
#define REG_ADC_OUT						 0x033
#define REG_BIAS_CURRENT_RX_LNA			 0x034
#define REG_BIAS_CURRENT_RX				 0x035
#define REG_BIAS_CURRENT_TX				 0x036
#define REG_BIAS_CURRENT_TX_DRV			 0x037
#define REG_MEM_CTL						 0x038
#define REG_RX_CHX_MEM					 0x039
#define REG_TX_CHX_MEM					 0x03A
#define REG_RX_CH1_MEM					 0x03D
#define REG_RX_CH2_MEM					 0x03E
#define REG_RX_CH3_MEM					 0x03F
#define REG_RX_CH4_MEM					 0x040
#define REG_TX_CH1_MEM					 0x041
#define REG_TX_CH2_MEM					 0x042
#define REG_TX_CH3_MEM					 0x043
#define REG_TX_CH4_MEM					 0x044
#define REG_PA_CH1_BIAS_OFF				 0x046
#define REG_PA_CH2_BIAS_OFF				 0x047
#define REG_PA_CH3_BIAS_OFF				 0x048
#define REG_PA_CH4_BIAS_OFF				 0x049
#define REG_LNA_BIAS_OFF				 0x04A
#define REG_TX_BEAM_STEP_START			 0x04D
#define REG_TX_BEAM_STEP_STOP			 0x04E
#define REG_RX_BEAM_STEP_START			 0x04F
#define REG_RX_BEAM_STEP_STOP			 0x050

// REGISTER CONSTANTS
#define INTERFACE_CONFIG_A_SOFTRESET	 	((1 << 7) | (1 << 0))
#define INTERFACE_CONFIG_A_LSB_FIRST	 	((1 << 6) | (1 << 1))
#define INTERFACE_CONFIG_A_ADDR_ASCN	 	((1 << 5) | (1 << 2))
#define INTERFACE_CONFIG_A_SDO_ACTIVE		((1 << 4) | (1 << 3))

#define LD_WRK_REGS_LDRX_OVERRIDE		   	(1 << 0)
#define LD_WRK_REGS_LDTX_OVERRIDE		   	(1 << 1)

#define RX_ENABLES_TX_VGA_EN			   	(1 << 0)
#define RX_ENABLES_TX_VM_EN				   	(1 << 1)
#define RX_ENABLES_TX_DRV_EN			   	(1 << 2)
#define RX_ENABLES_CH3_TX_EN			   	(1 << 3)
#define RX_ENABLES_CH2_TX_EN			   	(1 << 4)
#define RX_ENABLES_CH1_TX_EN			   	(1 << 5)
#define RX_ENABLES_CH0_TX_EN			   	(1 << 6)

#define TX_ENABLES_TX_VGA_EN			   	(1 << 0)
#define TX_ENABLES_TX_VM_EN				   	(1 << 1)
#define TX_ENABLES_TX_DRV_EN			   	(1 << 2)
#define TX_ENABLES_CH3_TX_EN			   	(1 << 3)
#define TX_ENABLES_CH2_TX_EN			   	(1 << 4)
#define TX_ENABLES_CH1_TX_EN			   	(1 << 5)
#define TX_ENABLES_CH0_TX_EN			   	(1 << 6)

#define MISC_ENABLES_CH4_DET_EN			   	(1 << 0)
#define MISC_ENABLES_CH3_DET_EN			   	(1 << 1)
#define MISC_ENABLES_CH2_DET_EN			   	(1 << 2)
#define MISC_ENABLES_CH1_DET_EN			   	(1 << 3)
#define MISC_ENABLES_LNA_BIAS_OUT_EN	   	(1 << 4)
#define MISC_ENABLES_BIAS_EN			   	(1 << 5)
#define MISC_ENABLES_BIAS_CTRL			   	(1 << 6)
#define MISC_ENABLES_SW_DRV_TR_MODE_SEL	   	(1 << 7)

#define SW_CTRL_POL						   	(1 << 0)
#define SW_CTRL_TR_SPI					   	(1 << 1)
#define SW_CTRL_TR_SOURCE				   	(1 << 2)
#define SW_CTRL_SW_DRV_EN_POL			   	(1 << 3)
#define SW_CTRL_SW_DRV_EN_TR			   	(1 << 4)
#define SW_CTRL_RX_EN					   	(1 << 5)
#define SW_CTRL_TX_EN					   	(1 << 6)
#define SW_CTRL_SW_DRV_TR_STATE			   	(1 << 7)

#define MEM_CTRL_RX_CHX_RAM_BYPASS		   	(1 << 0)
#define MEM_CTRL_TX_CHX_RAM_BYPASS		   	(1 << 1)
#define MEM_CTRL_RX_BEAM_STEP_EN		   	(1 << 2)
#define MEM_CTRL_TX_BEAM_STEP_EN		   	(1 << 3)
#define MEM_CTRL_BIAS_RAM_BYPASS		   	(1 << 5)
#define MEM_CTRL_BEAM_RAM_BYPASS		   	(1 << 6)
#define MEM_CTRL_SCAN_MODE_EN			   	(1 << 7)

#endif /* LIB_ADAR1000_H_ */

