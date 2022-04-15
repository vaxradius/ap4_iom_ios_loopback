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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define     I2C_ADDR            0x10

//*****************************************************************************
//
// Global message buffer for the IO master.
//
//*****************************************************************************
uint32_t DMATCBBuffer[256]; // For non-blocking transfer


void *g_IOMHandle;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
static am_hal_iom_config_t g_sIOMSpiConfig =
{
	.eInterfaceMode = AM_HAL_IOM_SPI_MODE,
	.ui32ClockFreq = AM_HAL_IOM_1MHZ,
	.eSpiMode = AM_HAL_IOM_SPI_MODE_0,
	//
	// Non-Blocking transaction memory configuration
	// Set length and pointer to Transfer Control Buffer.
	// Length is in 4 byte multiples
	.pNBTxnBuf = DMATCBBuffer,
       .ui32NBTxnBufLength = sizeof(DMATCBBuffer) / 4,
};

#define MAX_SPI_SIZE    1023

static am_hal_iom_config_t g_sIOMI2cConfig =
{
	.eInterfaceMode = AM_HAL_IOM_I2C_MODE,
	.ui32ClockFreq  = AM_HAL_IOM_1MHZ,
	//
	// Non-Blocking transaction memory configuration
	// Set length and pointer to Transfer Control Buffer.
	// Length is in 4 byte multiples
	.pNBTxnBuf = DMATCBBuffer,
       .ui32NBTxnBufLength = sizeof(DMATCBBuffer) / 4,
};

void iom_slave_read(bool bSpi, uint32_t offset, uint32_t *pBuf, uint32_t size)
{
	am_hal_iom_transfer_t       Transaction;

	Transaction.ui32InstrLen    = 1;
#if defined(AM_PART_APOLLO4B)
	Transaction.ui64Instr = offset;
#else
	Transaction.ui32Instr = offset;
#endif
	Transaction.eDirection      = AM_HAL_IOM_RX;
	Transaction.ui32NumBytes    = size;
	Transaction.pui32RxBuffer   = pBuf;
	Transaction.bContinue       = false;
	Transaction.ui8RepeatCount  = 0;
	Transaction.ui32PauseCondition = 0;
	Transaction.ui32StatusSetClr = 0;

	if ( bSpi )
	{
		Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
	}
	else
	{
		Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
	}
	am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
}


void iom_slave_read_nonblocking(bool bSpi, uint32_t offset, uint32_t *pBuf, uint32_t size, am_hal_iom_callback_t pfnCallback)
{
	am_hal_iom_transfer_t       Transaction;

	Transaction.ui32InstrLen    = 1;
#if defined(AM_PART_APOLLO4B)
	Transaction.ui64Instr = offset;
#else
	Transaction.ui32Instr = offset;
#endif
	Transaction.eDirection      = AM_HAL_IOM_RX;
	Transaction.ui32NumBytes    = size;
	Transaction.pui32RxBuffer   = pBuf;
	Transaction.bContinue       = false;
	Transaction.ui8RepeatCount  = 0;
	Transaction.ui32PauseCondition = 0;
	Transaction.ui32StatusSetClr = 0;

	if ( bSpi )
	{
		Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
	}
	else
	{
		Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
	}
	am_hal_iom_nonblocking_transfer(g_IOMHandle, &Transaction, pfnCallback, NULL);
}

void iom_slave_write(bool bSpi, uint32_t offset, uint32_t *pBuf, uint32_t size)
{
	am_hal_iom_transfer_t       Transaction;

	Transaction.ui32InstrLen    = 1;
#if defined(AM_PART_APOLLO4B)
	if(bSpi)
		Transaction.ui64Instr = (offset | 0x80); //Write bit for IOS SPI
	else
		Transaction.ui64Instr = offset;
#else
	Transaction.ui32Instr = offset;
#endif
	Transaction.eDirection      = AM_HAL_IOM_TX;
	Transaction.ui32NumBytes    = size;
	Transaction.pui32TxBuffer   = pBuf;
	Transaction.bContinue       = false;
	Transaction.ui8RepeatCount  = 0;
	Transaction.ui32PauseCondition = 0;
	Transaction.ui32StatusSetClr = 0;

	if ( bSpi )
	{
		Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
	}
	else
	{
		Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
	}
	am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
}

void iom_set_up(uint32_t iomModule, bool bSpi)
{
	//
	// Initialize the IOM.
	//
	am_hal_iom_initialize(iomModule, &g_IOMHandle);

	am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);

	if ( bSpi )
	{
		//
		// Set the required configuration settings for the IOM.
		//
		am_hal_iom_configure(g_IOMHandle, &g_sIOMSpiConfig);

		//
		// Configure the IOM pins.
		//
		am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_SPI_MODE);
	}
	else
	{
		//
		// Set the required configuration settings for the IOM.
		//
		am_hal_iom_configure(g_IOMHandle, &g_sIOMI2cConfig);

		//
		// Configure the IOM pins.
		//
		am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_I2C_MODE);
	}

	//
	// Enable the IOM.
	//
	am_hal_iom_enable(g_IOMHandle);

	am_hal_iom_interrupt_enable(g_IOMHandle, AM_HAL_IOM_INT_DCMP);

	//
	// Enable the iom interrupt in the NVIC.
	//
	NVIC_ClearPendingIRQ((IRQn_Type)(IOMSTR0_IRQn + iomModule));
	NVIC_EnableIRQ((IRQn_Type)(IOMSTR0_IRQn + iomModule));
	
}

void am_iomaster1_isr(void)
{
    uint32_t ui32Status;

    if (!am_hal_iom_interrupt_status_get(g_IOMHandle, true, &ui32Status))
    {
        if ( ui32Status )
        {
            am_hal_iom_interrupt_clear(g_IOMHandle, ui32Status);
            am_hal_iom_interrupt_service(g_IOMHandle, ui32Status); //ui32Status & ~(0x24) --> Clear 0x24 if  step by step debugging
        }
    }
}


