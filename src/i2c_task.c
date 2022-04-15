//*****************************************************************************
//
//! @file i2c_task.c
//!
//! @brief Task to handle radio operation.
//!
//*****************************************************************************

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

//*****************************************************************************
//
// Global includes for this project.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
//*****************************************************************************
//
// FreeRTOS include files.
//
//*****************************************************************************
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "portable.h"
#include "semphr.h"
#include "event_groups.h"

#define     IOM_MODULE          1
#define     USE_SPI             0   // 0 = I2C, 1 = SPI

#define IOSOFFSET_WRITE_INTEN       0x78

bool bTransationDone = false;

void iom_callback(void *pCallbackCtxt, uint32_t transactionStatus)
{
	bTransationDone = true;
}

//*****************************************************************************
//
// i2c task handle.
//
//*****************************************************************************
TaskHandle_t i2c_task_handle;

//*****************************************************************************
//
// Perform initial setup for the i2c task.
//
//*****************************************************************************
void
i2cTaskSetup(void)
{
	//NVIC_SetPriority(COOPER_IOM_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);
	//NVIC_SetPriority(AM_COOPER_IRQn, NVIC_configMAX_SYSCALL_INTERRUPT_PRIORITY);

	ios_set_up(USE_SPI);
	am_util_delay_ms(50);
	iom_set_up(IOM_MODULE, USE_SPI);
	//NVIC_SetPriority((IRQn_Type)(IOMSTR0_IRQn + IOM_MODULE), 0);
}

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************
void
i2cTask(void *pvParameters)
{
	uint32_t ioIntEnable;
	/* Block for 500ms. */
	const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

	while(1)
	{

		/*blocking transfer*/
		ioIntEnable = 0xA5;
		iom_slave_write(USE_SPI, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1);
		ioIntEnable = 0x00;
		iom_slave_read(USE_SPI, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1);

		/*non-blocking transfer*/
		ioIntEnable = 0x00;
		iom_slave_read_nonblocking(USE_SPI, IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1, iom_callback);
		
		vTaskDelay( xDelay );

	};
}
