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
// This is part of revision release_sdk_4_0_1-bef824fa27 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define     IOM_MODULE          1
#define     USE_SPI             0   // 0 = I2C, 1 = SPI

#define IOSOFFSET_WRITE_INTEN       0x78

bool bTransationDone = false;

void iom_callback(void *pCallbackCtxt, uint32_t transactionStatus)
{
	bTransationDone = true;
}



void floating_point_operation(void)
{
	float fA = 976.123f;
	float fB = -123.777f;
	float fC = 898.33f;

	//
	// Enable floating point.
	//
	am_hal_sysctrl_fpu_enable();
	am_hal_sysctrl_fpu_stacking_enable(true);

	am_util_stdio_printf("%.3f \n", fA*fB/fC);

}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
	uint32_t ioIntEnable;

	am_hal_pwrctrl_mcu_memory_config_t McuMemCfg =
	{
		.eCacheCfg    = AM_HAL_PWRCTRL_CACHE_ALL,
		.bRetainCache = true,
		.eDTCMCfg     = AM_HAL_PWRCTRL_DTCM_128K,
		.eRetainDTCM  = AM_HAL_PWRCTRL_DTCM_128K,
		.bEnableNVM0  = true,
		.bRetainNVM0  = false
	};

	am_hal_pwrctrl_sram_memcfg_t SRAMMemCfg =
	{
		.eSRAMCfg         = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithMCU   = AM_HAL_PWRCTRL_SRAM_NONE,
		.eActiveWithDSP   = AM_HAL_PWRCTRL_SRAM_NONE,
		.eSRAMRetain      = AM_HAL_PWRCTRL_SRAM_NONE
	};

	const am_hal_cachectrl_config_t am_hal_cachectrl_benchmark =
	{
		.bLRU                       = 0,
		.eDescript                  = AM_HAL_CACHECTRL_DESCR_1WAY_128B_512E,
		.eMode                      = AM_HAL_CACHECTRL_CONFIG_MODE_INSTR,
	};

	//
	// Set the cache configuration
	//
	am_hal_cachectrl_config(&am_hal_cachectrl_benchmark);
	am_hal_cachectrl_enable();

	//
	// Configure the board for low power operation.
	//
	am_bsp_low_power_init();

	//
	// Update memory configuration to minimum.
	//
	am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);
	am_hal_pwrctrl_sram_config(&SRAMMemCfg);

	//
	// Initialize the printf interface for UART output.
	//
	am_bsp_uart_printf_enable();

	//
	// Print the banner.
	//
	am_util_stdio_terminal_clear();
	am_util_stdio_printf("ap4_iom_ios_loopback Example\n");

	floating_point_operation();

	ios_set_up(USE_SPI);
	am_util_delay_ms(50);
	iom_set_up(IOM_MODULE, USE_SPI);

	 am_hal_interrupt_master_enable();

	/*blocking transfer*/
	ioIntEnable = 0xA5;
	iom_slave_write(USE_SPI, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1);
	ioIntEnable = 0x00;
	iom_slave_read(USE_SPI, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1);

	/*non-blocking transfer*/
	ioIntEnable = 0x00;
	iom_slave_read_nonblocking(USE_SPI, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1, iom_callback);

	//
	// We are done printing.
	// Disable the UART
	//
	am_bsp_uart_printf_disable();


	while (1)
	{
		//
		// Go to Deep Sleep until wakeup.
		//
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
	}
}
