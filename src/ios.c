//*****************************************************************************
//
// Copyright (c) 2021, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision b0-release-20210111-1514-g6a1d4008b7 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define     I2C_ADDR            0x10

static void *g_pIOSHandle;

//*****************************************************************************
//
// Message buffers.
//
// data from the IOS interface, which is only 8 bits wide.
//
//*****************************************************************************

#define AM_IOS_TX_BUFSIZE_MAX   1023
uint8_t g_pui8TxFifoBuffer[AM_IOS_TX_BUFSIZE_MAX];

//*****************************************************************************
//
// SPI Slave Configuration
//
//*****************************************************************************
static am_hal_ios_config_t g_sIOSSpiConfig =
{
	// Configure the IOS in SPI mode.
	.ui32InterfaceSelect = AM_HAL_IOS_USE_SPI,

	// Eliminate the "read-only" section, so an external host can use the
	// entire "direct write" section.
	.ui32ROBase = 0x78,

	// Making the "FIFO" section as big as possible.
	.ui32FIFOBase = 0x80,

	// We don't need any RAM space, so extend the FIFO all the way to the end
	// of the LRAM.
	.ui32RAMBase = 0x100,

	// FIFO Threshold - set to half the size
	.ui32FIFOThreshold = 0x20,

	.pui8SRAMBuffer = g_pui8TxFifoBuffer,
	.ui32SRAMBufferCap = AM_IOS_TX_BUFSIZE_MAX,
};

//*****************************************************************************
//
// I2C Slave Configuration
//
//*****************************************************************************
am_hal_ios_config_t g_sIOSI2cConfig =
{
	// Configure the IOS in I2C mode.
	.ui32InterfaceSelect = AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(I2C_ADDR << 1),

	// Eliminate the "read-only" section, so an external host can use the
	// entire "direct write" section.
	.ui32ROBase = 0x78,

	// Set the FIFO base to the maximum value, making the "direct write"
	// section as big as possible.
	.ui32FIFOBase = 0x80,

	// We don't need any RAM space, so extend the FIFO all the way to the end
	// of the LRAM.
	.ui32RAMBase = 0x100,

	// FIFO Threshold - set to half the size
	.ui32FIFOThreshold = 0x40,

	.pui8SRAMBuffer = g_pui8TxFifoBuffer,
	.ui32SRAMBufferCap = AM_IOS_TX_BUFSIZE_MAX,
};


//*****************************************************************************
//
// Configure the SPI slave.
//
//*****************************************************************************
void ios_set_up(bool bSpi)
{
	if (bSpi)
	{
		// Configure SPI interface
		am_bsp_ios_pins_enable(0, AM_HAL_IOS_USE_SPI);
		//
		// Configure the IOS interface and LRAM structure.
		//
		am_hal_ios_initialize(0, &g_pIOSHandle);
		am_hal_ios_power_ctrl(g_pIOSHandle, AM_HAL_SYSCTRL_WAKE, false);
		am_hal_ios_configure(g_pIOSHandle, &g_sIOSSpiConfig);
	}
	else
	{
		// Configure I2C interface
		am_bsp_ios_pins_enable(0, AM_HAL_IOS_USE_I2C);
		//
		// Configure the IOS interface and LRAM structure.
		//
		am_hal_ios_initialize(0, &g_pIOSHandle);
		am_hal_ios_power_ctrl(g_pIOSHandle, AM_HAL_SYSCTRL_WAKE, false);
		am_hal_ios_configure(g_pIOSHandle, &g_sIOSI2cConfig);
	}
}



