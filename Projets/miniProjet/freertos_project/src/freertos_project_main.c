/*
 * @brief Blinky example using timers and sysTick
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */


#include "board.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"



extern void pwm_task(void *pvParameters);
 extern void pwm_init(void);
extern void i2C_slave_Task(void *pvParameters);
extern void i2c_setup(void);
extern void SdTask (void *pvParameters) ;


/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
//	Board_LED_Set(2, true);
}

/* LED1 toggle thread */
static void vLEDTask1(void *pvParameters) {
	bool LedState = false;
	uint32_t i;

	while (1) {

//					Board_LED_Set(2, LedState);
//					LedState = (bool) !LedState;
		
			
		

					/* About a 3Hz on/off toggle rate */
				vTaskDelay(configTICK_RATE_HZ *10);

	}
}

/* LED2 toggle thread */
static void vLEDTask2(void *pvParameters) {
	bool LedState = false;

	while (1) {
//		Board_LED_Set(0, LedState);
//		LedState = (bool) !LedState;

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ*2 );
	}
}

/* UART (or output) thread */
static void vUARTTask(void *pvParameters) {
	uint32_t tickCnt = 0 , count;

	while (1) {
		count =10000000;
		while(--count);
	DEBUGOUT("Tick: %d \r\n", tickCnt);
		tickCnt++;

		/* About a 1s delay here */
		vTaskDelay(configTICK_RATE_HZ);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	BaseType_t i ;
	prvSetupHardware(); 
	pwm_init();
	i2c_setup();
	
//		/* SD log thread */
//	xTaskCreate(SdTask, "sdTask",
//				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL),
//				(TaskHandle_t *) NULL);

	/* I2C thread */
	i =xTaskCreate(i2C_slave_Task, "i2cTask",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
	
if (i!=pdPASS){
		DEBUGOUT("task create error \r\n");
	}
	
	/* servomotor control task */
	i = xTaskCreate(pwm_task, "servomotor",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	if (i!=pdPASS){
		DEBUGOUT("task create error \r\n");
	}

	/* UART output thread, simply counts seconds */
	i= xTaskCreate(vUARTTask, "vTaskUart",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);
	if (i!=pdPASS){
		DEBUGOUT("task create error \r\n");
	}

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
