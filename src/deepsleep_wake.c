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
#include "am_util_debug.h"

#include "rtos.h"

void floating_point_operation(void)
{
	float fA = 976.123f;
	float fB = -123.777f;
	float fC = 898.33f;

	//
	// Enable the floating point module, and configure the core for lazy
	// stacking.
	//
	//am_hal_sysctrl_fpu_enable();
	//am_hal_sysctrl_fpu_stacking_enable(true);

	am_util_debug_printf("%.3f \n", fA*fB/fC);

}

//*****************************************************************************
//
// Enable printing to the console.
//
//*****************************************************************************
void
enable_print_interface(void)
{
    //
    // Initialize a debug printing interface.
    //
    am_bsp_itm_printf_enable();
}

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
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


	//
	// Enable the floating point module, and configure the core for lazy
	// stacking.
	//
	am_hal_sysctrl_fpu_enable();
	am_hal_sysctrl_fpu_stacking_enable(true);


	//
	// Set the cache configuration
	//
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();


	//
	// Configure the board for low power.
	//
	am_bsp_low_power_init();
	//
	// Turn off crypto
	//
	am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_CRYPTO_POWERDOWN, NULL);


	//
	// Turn off unneeded memory
	//
	am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);
	am_hal_pwrctrl_sram_config(&SRAMMemCfg);

	// Enable printing to the console.
	//
#ifdef AM_DEBUG_PRINTF
	enable_print_interface();
	//
	// Print the banner.
	//
	am_util_stdio_terminal_clear();
	am_util_debug_printf("FreeRTOS Example\n");
	floating_point_operation();

#endif

	//
	// Run the application.
	//
	run_tasks();

	//
	// We shouldn't ever get here.
	//
	while (1)
	{
	}

}

