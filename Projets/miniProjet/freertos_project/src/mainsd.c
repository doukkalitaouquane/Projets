/**************************************************************************//**
 * @file     main.c
 * @brief    Main program to test SDC/MMC functionality
 *
 * @note
 * Copyright (C) 2010 NXP Semiconductors(NXP). All rights reserved.
 *
 ******************************************************************************/ 
                  

#include <stdio.h>
#include "board.h"                   /* LPC15xx definitions                */

#include "FreeRTOS.h"
#include "timers.h"
#include "sd.h"

/* Macro definitions for single/multiple sector read/write */
#define S_SECTOR_INDEX 103
#define S_FILL_VALUE 0xAA

#define M_SECTOR_INDEX  203
#define M_SECTOR_NUM    2
#define M_FILL_VALUE    0x55

/* The number of bytes to display in terminal */
#define DISPLAY_SIZE    32

/* data buffer */
uint8_t *buf = (uint8_t *)0x2007C000; // 16KB

volatile uint32_t Timer = 0;

/*----------------------------------------------------------------------------
  software timer: Executed periodically
 *----------------------------------------------------------------------------*/
 
 TimerHandle_t xSdTimer; 
 static void vSdTimerCallback( TimerHandle_t pxTimer )
{         
	 disk_timerproc(); 
	 Timer++;
        
}

/*----------------------------------------------------------------------------
  Test R/W of single sector
 *----------------------------------------------------------------------------*/
void SingleSector_RW_Test()
{
    uint32_t i;

    printf("\n>Single sector read/write test ...\r\n");

    printf("Read sector #%d:\n", S_SECTOR_INDEX);
    if (SD_ReadSector(S_SECTOR_INDEX, buf, 1) == SD_FALSE)
    {
        printf("Failed to read sector %d.\r\n",  S_SECTOR_INDEX);
        while (1);
    }
    printf("(Only display the first %d bytes to avoid too many content in terminal).\r\n", DISPLAY_SIZE); 
    for (i=0;i<DISPLAY_SIZE;)
    {        
        printf("%2x ", buf[i++]);
        if ((i%16)==0) printf("\n");
    }

    printf("Fill sector #%d with 0x%x.\r\n", S_SECTOR_INDEX, S_FILL_VALUE);
    for (i=0;i<512;i++) buf[i] = S_FILL_VALUE;
    if (SD_WriteSector(S_SECTOR_INDEX, buf, 1) == SD_FALSE)
    {
        printf("Failed to write sector %d.\r\n", S_SECTOR_INDEX);
        while (1);
    }

    printf("Read sector #%d\r\n", S_SECTOR_INDEX);
    if (SD_ReadSector(S_SECTOR_INDEX, buf, 1) == SD_FALSE)
    {
        printf("Failed to read sector %d.\r\n",  S_SECTOR_INDEX);
        while (1);
    }
    printf("(Only display the first %d bytes to avoid too many content in terminal).\r\n", DISPLAY_SIZE);
    for (i=0;i<DISPLAY_SIZE;)
    {        
        printf("%2x ", buf[i++]);
        if ((i%16)==0) printf("\n");
    }
}

/*----------------------------------------------------------------------------
  Test R/W of multiple sectors
 *----------------------------------------------------------------------------*/
void MultiSector_RW_Test ()
{
    uint32_t i;

    printf("\r\n>Multiple sector read/write test ...\r\n");

    printf("Read %d sectors from #%d:\r\n", M_SECTOR_NUM, M_SECTOR_INDEX);
    if (SD_ReadSector(M_SECTOR_INDEX, buf, M_SECTOR_NUM) == SD_FALSE)
    {
        printf("Failed to read sectors from %d.\r\n",  M_SECTOR_INDEX);
        while (1);
    }
    printf("(Only display the first %d bytes to avoid too many content in terminal).\r\n", DISPLAY_SIZE);
    for (i=0;i<DISPLAY_SIZE;)
    {        
        printf("%2x ", buf[i++]);
        if ((i%16)==0) printf("\r\n");
    }

    printf("Fill %d sectors from #%d with 0x%x.\r\n", M_SECTOR_NUM, M_SECTOR_INDEX, M_FILL_VALUE);
    for (i=0;i<512*M_SECTOR_NUM;i++) buf[i] = M_FILL_VALUE;
    if (SD_WriteSector(M_SECTOR_INDEX, buf, M_SECTOR_NUM) == SD_FALSE)
    {
        printf("Failed to write sectors from %d.\r\n", M_SECTOR_INDEX);
        while (1);
    }

    printf("Read %d sectors from #%d:\r\n", M_SECTOR_NUM, M_SECTOR_INDEX);
    if (SD_ReadSector(M_SECTOR_INDEX, buf, M_SECTOR_NUM) == SD_FALSE)
    {
        printf("Failed to read sectors from %d.\r\n",  M_SECTOR_INDEX);
        while (1);
    }
    printf("Only display the first %d bytes to avoid too many content in terminal.\r\n", DISPLAY_SIZE);
    for (i=0;i<DISPLAY_SIZE;)
    {        
        printf("%2x ", buf[i++]);
        if ((i%16)==0) printf("\r\n");
    }   
}

/*----------------------------------------------------------------------------
  Main Program
 *----------------------------------------------------------------------------*/
void SdTask (void *pvParameters) 
{
    uint32_t i;

	xSdTimer = xTimerCreate( 
                     "SdTimer",  /*  name, not used by the RTOS kernel. */									
                     ( 10)/portTICK_PERIOD_MS , /* The timer period: 10 ms. */                     
                     pdTRUE, /* The timers will auto-reload */                     
                     ( void * ) 1, /* Assign ID timer . */                    
                     vSdTimerCallback  /* callback  */
                   );
										 
		if( xSdTimer == NULL )
         {
             printf(" The timer was not created. \r\n");
         }
         else
         {
             /* Start the timer.  No block time is specified, and even if one
             was it would be ignored because the RTOS scheduler has not yet
             been started. */
             if( xTimerStart( xSdTimer, 0 ) != pdPASS )
             {
                printf("The timer could not be set into the Active state\r\n");
             }
         }
    
    printf("\nAccess SDC/MMC via SPI on NXP LPC1700. "__DATE__" "__TIME__"\r\n");

    if (SD_Init () == SD_FALSE)
    {
        printf("Failed to init the card, pls check the card.\r\n");
        while (1);
    }

    if (SD_ReadConfiguration() == SD_FALSE)
    {
        printf("Failed to read card CID or CSD.\r\n");
        while (1);
    }

    printf("Card init OK.\r\n");
    printf("Card type: ");
    switch (CardType)
    {
        case CARDTYPE_MMC:
            printf("MMCr\\n");
            break;
        case CARDTYPE_SDV1:
            printf("Version 1.x Standard Capacity SD card.r\\n");
            break;
        case CARDTYPE_SDV2_SC:
            printf("Version 2.0 or later Standard Capacity SD card.\r\n");
            break;
        case CARDTYPE_SDV2_HC:
            printf("Version 2.0 or later High/eXtended Capacity SD card.\r\n");
            break;
        default:
            break;            
    }
    printf("Sector size: %d bytes \r\n", CardConfig.sectorsize);
    printf("Sector count: %d\r\n", CardConfig.sectorcnt);
    printf("Block size: %d sectors \r\n", CardConfig.blocksize);
    printf("Card capacity: %d MByte\r\n", (((CardConfig.sectorcnt >> 10) * CardConfig.sectorsize)) >> 10);
    printf("OCR(hex): ");
    for (i=0;i<4;i++) printf("%02x ", CardConfig.ocr[i]);
    printf("\r\n");
    printf("CID(hex): ");
    for (i=0;i<16;i++) printf("%02x ", CardConfig.cid[i]);
    printf("\r\n");
    printf("CSD(hex): ");
    for (i=0;i<16;i++) printf("%02x ", CardConfig.csd[i]);
    printf("\r\n");


    /* Test read/write of single sector */
    SingleSector_RW_Test ();

    /* Test read/write of multiple sectors */
    MultiSector_RW_Test ();


    /* Read speed test */
    printf("\n>Read speed test ...\r\n");

    i = 16;
    printf("\r\nReading %d sectors (%d bytes) ...", i, i*512);
    Timer = 0;
    if (SD_ReadSector(100, buf, i) == SD_FALSE)
    {
        printf("Failed.\r\n");
        while (1);
    }
    printf(" at speed of %d kB/sec.\r\n", Timer ? ((i*512) / Timer) : 0);

    i = 32;
    printf("Reading %d sectors (%d bytes) ...", i, i*512);
    Timer = 0;
    if (SD_ReadSector(100, buf, i) == SD_FALSE)
    {
        printf("Failed.\r\n");
        while (1);
    }
    printf(" at speed of %d kB/sec.\r\n", Timer ? ((i*512) / Timer) : 0);

    /* Write speed test */
    printf("\r\n>Write speed test ...\r\n");
    i = 16;
    printf("\r\nWriting %d sectors (%d bytes) ...", i, i*512);
    Timer = 0;
    if (SD_WriteSector(100, buf, i) == SD_FALSE)
    {
        printf("Failed.\r\n");
        while (1);
    }
    printf(" at speed of %d kB/sec.\n", Timer ? ((i*512) / Timer) : 0);

    i = 32;
    printf("Writing %d sectors (%d bytes) ...", i, i*512);
    Timer = 0;
    if (SD_WriteSector(100, buf, i) == SD_FALSE)
    {
        printf("Failed.\r\n");
        while (1);
    }
    printf(" at speed of %d kB/sec.\n", Timer ? ((i*512) / Timer) : 0);

    printf ("\nTest complete successfully.\r\n");

    while (1); 
}


