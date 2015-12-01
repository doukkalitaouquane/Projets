/*
 * @brief SD/MMC example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
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

#include <string.h>
#include "board.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "timers.h"


#include "ff.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


/* buffer size (in byte) for R/W operations */
#define BUFFER_SIZE     4096

STATIC FATFS fatFS;	/* File system object */
STATIC FIL fileObj;	/* File object */
STATIC INT buffer[BUFFER_SIZE / 4];		/* Working buffer */
STATIC volatile int32_t sdcWaitExit = 0;

STATIC volatile Status  eventResult = ERROR;

/* An array to hold handles to the created timers. */
 TimerHandle_t xSdTimer;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* SDMMC card info structure */
//SDMMC_CARD_T sdCardInfo;
volatile uint32_t timerCntms = 0; /* Free running milli second timer */

extern void MMC_disk_timerproc(TimerHandle_t pxTimer);

/*****************************************************************************
 * Private functions
 ****************************************************************************/









/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Error processing function: stop with dying message
 * @param	rc	: FatFs return value
 * @return	Nothing
 */
void die(FRESULT rc)
{
	DEBUGOUT("Failed with rc=%u.\r\n", rc);
	//	for (;; ) {}
}




	
/**
 * @brief	Main routine for SDMMC example
 * @return	Nothing
 */
void  sdhcTask(void *pvParameters)
{
	FRESULT rc;		/* Result code */
	DIR dir;		/* Directory object */
	FILINFO fno;	/* File information object */
	UINT bw, br, i;
	uint8_t *ptr;
	char debugBuf[64];
	
//	xSdTimer = xTimerCreate( /* Just a text name, not used by the RTOS kernel. */
//                     "Timer SD",
//                     /* The timer period 10 ms. */
//                     ( 10/portTICK_PERIOD_MS),
//                     /* The timers will auto-reload themselves when they
//                     expire. */
//                     pdTRUE,
//                     /* Assign each timer a unique id equal to its array
//                     index. */
//                     ( void * ) 1,
//                     /* timer  callback when it expires. */
//                     MMC_disk_timerproc
//                   );

	
	
	DEBUGOUT("\r\nHello NXP Semiconductors\r\nSD Card demo\r\n");

	

	f_mount(0, &fatFS);		/* Register volume work area (never fails) */

	DEBUGOUT("\r\nOpen an existing file (test.txt).\r\n");

	rc = f_open(&fileObj, "test.txt", FA_READ);
	if (rc) {
		die(rc);
	}
	else {
		for (;; ) {
			/* Read a chunk of file */
			rc = f_read(&fileObj, buffer, sizeof buffer, &br);
			if (rc || !br) {
				break;					/* Error or end of file */
			}
			ptr = (uint8_t *) buffer;
			for (i = 0; i < br; i++) {	/* Type the data */
				DEBUGOUT("%c", ptr[i]);
			}
		}
		if (rc) {
			die(rc);
		}

		DEBUGOUT("\r\nClose the file.\r\n");
		rc = f_close(&fileObj);
		if (rc) {
			die(rc);
		}
	}

	DEBUGOUT("\r\nCreate a new file (hello.txt).\r\n");
	rc = f_open(&fileObj, "hello.txt", FA_WRITE | FA_CREATE_ALWAYS);
	if (rc) {
		die(rc);
	}
	else {

		DEBUGOUT("\r\nWrite a text data. (Hello world!)\r\n");

		rc = f_write(&fileObj, "Hello world!\r\n", 14, &bw);
		if (rc) {
			die(rc);
		}
		else {
			sprintf(debugBuf, "%u bytes written.\r\n", bw);
			DEBUGOUT("%s\r\n",debugBuf);
		}
		DEBUGOUT("\r\nClose the file.\r\n");
		rc = f_close(&fileObj);
		if (rc) {
			die(rc);
		}
	}
	DEBUGOUT("\r\nOpen root directory.\r\n");
	rc = f_opendir(&dir, "");
	if (rc) {
		die(rc);
	}
	else {
		DEBUGOUT("\r\nDirectory listing...\r\n");
		for (;; ) {
			/* Read a directory item */
			rc = f_readdir(&dir, &fno);
			if (rc || !fno.fname[0]) {
				break;					/* Error or end of dir */
			}
			if (fno.fattrib & AM_DIR) {
				sprintf(debugBuf, "   <dir>  %s\r\n", fno.fname);
			}
			else {
				sprintf(debugBuf, "   %8lu  %s\r\n", fno.fsize, fno.fname);
			}
			DEBUGOUT("%s\r\n",debugBuf);
		}
		if (rc) {
			die(rc);
		}
	}
	DEBUGOUT("\r\nTest completed.\r\n");
	for (;; ) {}
}
