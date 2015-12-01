/*-----------------------------------------------------------------------*/
/* MMC/SDSC/SDHC (in SPI mode) control module for LPC15xx Version 0.8.0  */
/*-----------------------------------------------------------------------*/




#include "FreeRTOSConfig.h"

#include "FreeRTOS.h"

#include "board.h"

#include "integer.h"
#include "spi_sd_lpc15xx.h"
#include "semphr.h"
#include "timers.h"
/// #include "monitor.h"

/* available modes: */
#define SPI_SD_USE_POLLING  0
#define SPI_SD_USE_FIFO     1
#define SPI_SD_USE_DMA      2

/* used mode : */
#define SPI_SD_ACCESS_MODE  SPI_SD_USE_DMA

// For Olimex LPC1766-STK
// MMC_PWR (P-Channel FET): P0.21
// SSEL0: P0.6, SCK1: P0.7, MISO1: P0.8, MOSI1: P0.9
#define SOCKET_POWER_PORT        0
#define SOCKET_POWER_PIN        24
#define SOCKET_POWER_MASK       (1 << SOCKET_POWER_PIN)
#define SOCKET_POWER_OPENDRAIN  PINSEL_PINMODE_OPENDRAIN /* 33k pull-up mounted */

/* used SSP-port: */
#define CARD_SSP                 1

/*--------------------------------------------------------------------------
   Module Private Functions and Variables
---------------------------------------------------------------------------*/

/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD9	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
#define ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define USE_INTEGER_CLOCK



/* Number of SPI0 TX descriptors used for DMA  (ping-pong)*/
#define SPI0TXDESC 1




/* Number of SPI0 RX descriptors used for DMA ping- pong */
#define SPI0RXDESC  1







/* DMA descriptors must be aligned to 16 uint8_ts */
#if defined(__CC_ARM)
__align(16) static DMA_CHDESC_T dmaTXDesc[SPI0TXDESC];
__align(16) static DMA_CHDESC_T dmaRXDesc[SPI0RXDESC];
#endif /* defined (__CC_ARM) */

/* IAR support */
#if defined(__ICCARM__)
#pragma data_alignment=16
static DMA_CHDESC_T dmaTXDesc[SPI0TXDESC];
#pragma data_alignment=16
static DMA_CHDESC_T dmaRXDesc[SPI0RXDESC];
#endif /* defined (__ICCARM__) */

#if defined( __GNUC__ )
static DMA_CHDESC_T dmaTXDesc[SPI0TXDESC] __attribute__ ((aligned(16)));
static DMA_CHDESC_T dmaRXDesc[SPI0RXDESC] __attribute__ ((aligned(16)));
#endif /* defined (__GNUC__) */

/******************************************************************************/



#if ( SPI_SD_ACCESS_MODE == SPI_SD_USE_DMA )

enum dma_direction_ { MEM_TO_CARD, CARD_TO_MEM };

#define DMA_CHANNEL_TX          0
#define DMA_CHANNEL_TX_HANDLE   LPC_GPDMACH0
#define DMA_CHANNEL_RX          1
#define DMA_CHANNEL_RX_HANDLE   LPC_GPDMACH1

#define DMA_DUMMY_SIZE 512
#if USE_DMA_DUMMY_RAM
static
#else
static const
#endif /* USE_DMA_DUMMY_RAM */
uint8_t dma_dummy[DMA_DUMMY_SIZE] = {
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };
#endif /* USE_DMA */


static const uint32_t socket_state_mask_cp = (1 << 0);
static const uint32_t socket_state_mask_wp = (1 << 1);
	
	/* Transmit and Receive Buffers */
static uint16_t xferArray[1];
static uint16_t rx_buff[1];
/* SPI Transfer Setup */
static SPI_DATA_SETUP_T XferSetup;

static volatile DSTATUS Stat = STA_NOINIT;  /* Disk status */

static volatile uint32_t Timer1, Timer2;       /* 100Hz decrement timers */
	
	
SemaphoreHandle_t xSpiTxRxSemaphore = NULL;
static uint8_t CardType;                       /* Card type flags */

static uint8_t socket_powered;
static volatile int intErrCode;

enum speed_setting { INTERFACE_SLOW, INTERFACE_FAST };

SPI_CFG_T spiCfg; // SPI configuration struct

/*-----------------------------------------------------------------------*/
/* socket control low-level functions                                    */
/*-----------------------------------------------------------------------*/

static INLINE uint32_t socket_is_empty(void)
{
	return 0; 
}

static INLINE  uint32_t socket_is_write_protected(void)
{
	return 0; 
}

static void socket_power_on()
{
	/* Toggle state, low is on, high is off */
		Chip_GPIO_SetPinState(LPC_GPIO, SOCKET_POWER_PORT , SOCKET_POWER_PIN, 0);
	//GPIO_ClearValue(SOCKET_POWER_PORT, SOCKET_POWER_MASK);
	socket_powered = 1;
}

static void socket_power_off()
{
	Chip_GPIO_SetPinState(LPC_GPIO, SOCKET_POWER_PORT , SOCKET_POWER_PIN, 1);
	//GPIO_SetValue(SOCKET_POWER_PORT, SOCKET_POWER_MASK);
	socket_powered = 0;
}

static INLINE  uint8_t socket_is_powered()
{
	return socket_powered;
}



// configuration de pin d'allimentation de la carte SD : P0.24 as output
static void socket_init()
{
		/* Enable the clock to the Switch Matrix */
			Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
			Chip_IOCON_PinMuxSet(LPC_IOCON, SOCKET_POWER_PORT, SOCKET_POWER_PIN, (IOCON_MODE_INACT |
							IOCON_DIGMODE_EN|IOCON_OPENDRAIN_EN|IOCON_FUNC0));
			
			/* Disable the clock to the Switch Matrix to save power */
			Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
			
			Chip_GPIO_SetDir(LPC_GPIO, SOCKET_POWER_PORT, SOCKET_POWER_PIN, 1); // output
	

}


/*-----------------------------------------------------------------------*/
/* SPI low-level functions                                               */
/*-----------------------------------------------------------------------*/
#if ( CARD_SSP == 1 )

static INLINE void select_card()
{
	// SSEL0 P0.27 low
	Chip_GPIO_SetPinState(LPC_GPIO, 0 , 27, 0);
	
}

static INLINE void de_select_card()
{
	// SSEL0 high
	Chip_GPIO_SetPinState(LPC_GPIO, 0 , 27, 1);
	
}

static void spi_set_speed( enum speed_setting speed )
{
		
	if ( speed == INTERFACE_SLOW ) {
		spiCfg.ClkDiv  = Chip_SPI_CalClkRateDivider(LPC_SPI0, 400000);
		
	} else {
		spiCfg.ClkDiv  = Chip_SPI_CalClkRateDivider(LPC_SPI0, 25000000);
		
		
	}
	Chip_SPI_SetConfig(LPC_SPI0, &spiCfg);
}

static void Init_SPI_PinMux(void)
{
#if (defined(BOARD_NXP_LPCXPRESSO_1549))

	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	/*
	 * Initialize SPI0 pins connect
	 * SCK0: PINASSIGN3[23:16]: Select P0.16
	 * MOSI0: PINASSIGN3[]: Select P0.28
	 * MISO0: PINASSIGN3[31:24] : Select P0.12
	 * SSEL0: PINASSIGN4[7:0]: Select P0.27
	 */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 16, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 28, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 27, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, 16);	/* P0.16 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, 28);/* P0.28 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_MISO_IO, 12);/* P0.12 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_SSELSN_0_IO, 27);	/* P0.27 */

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
#else
	/* Configure your own SPI pin muxing here if needed */
#warning "No SPI pin muxing defined"
#endif
}

/* Turn on LED to indicate an error */
static void errorSPI(void)
{
	Board_LED_Set(0, true);
	while (1) {}
}





	static void setupSpiMaster()
{
	
	SPI_DELAY_CONFIG_T spiDelayCfg;
	/* Initialize SPI Block */
	Chip_SPI_Init(LPC_SPI0);
	/* Set SPI Config register */
	spiCfg.ClkDiv = 0xFFFF;	/* Set Clock divider to maximum */
	spiCfg.Mode = SPI_MODE_MASTER;	/* Enable Master Mode */
	spiCfg.ClockMode = SPI_CLOCK_MODE0;	/* Enable Mode 0 */
	spiCfg.DataOrder = SPI_DATA_MSB_FIRST;	/* Transmit MSB first */
	/* Slave select polarity is active low */
	spiCfg.SSELPol = (SPI_CFG_SPOL0_LO | SPI_CFG_SPOL1_LO | SPI_CFG_SPOL2_LO | SPI_CFG_SPOL3_LO);
	Chip_SPI_SetConfig(LPC_SPI0, &spiCfg);
	/* Set Delay register */
	spiDelayCfg.PreDelay = 2;
	spiDelayCfg.PostDelay = 2;
	spiDelayCfg.FrameDelay = 2;
	spiDelayCfg.TransferDelay = 2;
	Chip_SPI_DelayConfig(LPC_SPI0, &spiDelayCfg);
	/* Enable Loopback mode for this example */
	Chip_SPI_EnableLoopBack(LPC_SPI0);
	/* Enable SPI0 */
	Chip_SPI_Enable(LPC_SPI0);
}
void spi_close(void)
{
	
	
	Chip_SPI_Disable(LPC_SPI0);
	Chip_SPI_DeInit(LPC_SPI0);
	

}

	/* Master SPI transmit in interrupt mode */
	static void WriteSpiMssg(uint16_t *pTxBuff, uint16_t *pRxBuff,uint32_t count, uint8_t dataSize)
	{
		/* Init variable used as semaphore */
		intErrCode = -1;
		/* Setup Transfer structure, this data should be retained for the entire transmission */
		XferSetup.pTx = pTxBuff;	/* Transmit Buffer */
		XferSetup.pRx = pRxBuff;/* Receive Buffer */
		XferSetup.DataSize = dataSize * 8;	/* Data size in bits */
		XferSetup.Length = count/dataSize;	/* Total frame length */
		/* Assert only SSEL0 */
		XferSetup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1 | SPI_TXCTL_DEASSERT_SSEL2 |
						 SPI_TXCTL_DEASSERT_SSEL3;
		XferSetup.TxCnt = 0;
		XferSetup.RxCnt = 0;
		if (pRxBuff ==NULL)
		{
		Chip_SPI_WFrames_Blocking(LPC_SPI0, &XferSetup) ;
		}
		else if(pTxBuff==NULL)
		{
			Chip_SPI_ReadFrames_Blocking(LPC_SPI_T *pSPI, SPI_DATA_SETUP_T *pXfSetup)
		}else{
			Chip_SPI_RWFrames_Blocking(LPC_SPI0, &XferSetup) ;
		}
		
		if (Chip_SPI_Int_RWFrames(LPC_SPI0, &XferSetup) == ERROR) {
			errorSPI();
		}
		/* Enable interrupts after initiating transmission */
		Chip_SPI_Int_Cmd(LPC_SPI0,
						 SPI_INTENSET_RXRDYEN | SPI_INTENSET_TXRDYEN | SPI_INTENSET_RXOVEN | SPI_INTENSET_TXUREN,
						 ENABLE);

		
		}
	}

/*****************************************************************************
 * Public functions
 ****************************************************************************/




		static void spi_transfer(
			uint8_t dir,          /* MEM_TO_CARD or CARD_TO_MEM                          */
			uint16_t *buff,  /* TO_CARD       : 512 uint8_t data block to be transmitted
														FROM_CARD     : Data buffer to store received data    */
			uint8_t uint8_ts  
					/* TO_CARD       : uint8_t count (must be multiple of 2)
														FROM_CARD     : uint8_t count (must be 512)              */
		)
		{	 
			uint8_t dataSize;
			uint16_t pBuff;
			pBuff = (uint16_t*)buff;
			
			if (uint8_ts==1) 
				dataSize=1;			
			else 
				dataSize =2;
		
					if ( dir == MEM_TO_CARD ) {  // transmission
						
						WriteSpiMssg(pBuff, NULL,uint8_ts,dataSize);
						
						

					} else { // reception  
									
						WriteSpiMssg( NULL,pBuff,uint8_ts,dataSize);
					}
				

		
				}
		
/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/
static uint8_t wait_ready (void)
{
	uint16_t RxBuf;
	uint16_t TxBuf= 0xFFFF;

	Timer2 = 50;	/* Wait for ready in timeout of 500ms */
	WriteSpiMssg(&TxBuf, &RxBuf,2,2);
	do
		WriteSpiMssg(&TxBuf, &RxBuf,2,2);
	while ((RxBuf != 0xFFFF) && Timer2);

	return (uint8_t)RxBuf;
}

/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/
static void release_spi (void)
{
	de_select_card();
	
}

/*-----------------------------------------------------------------------*/
/* Power up/down                                                         */
/*-----------------------------------------------------------------------*/
static void power_on()
{
	socket_init();
	socket_power_on();

	for (Timer1 = 25; Timer1; );	/* Wait for 250ms */

	/* Setup SPI pin muxing */
	Init_SPI_PinMux();

	/* Allocate SPI handle, setup rate, and initialize clocking */
	setupSpiMaster();
	spi_set_speed(INTERFACE_SLOW);
	de_select_card();
}

static void power_off()
{
	if (!(Stat & STA_NOINIT)) {
		select_card();
		wait_ready();
		release_spi();
	}
	spi_close();
	socket_power_off();
	Stat |= STA_NOINIT;		/* Set STA_NOINIT */
}



/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/
static uint8_t send_cmd (
	uint16_t cmd,		/* Command uint8_t */
	uint32_t arg		/* Argument */
)
{
	uint16_t n, res;


	if (cmd & 0x80) {	/* ACMD<n> is the command sequence of CMD55-CMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card and wait for ready */
	de_select_card();
	select_card();
	if (wait_ready() != 0xFF) {
		return 0xFF;  // time out expered
	}

	/* Send command packet */
	WriteSpiMssg(&cmd, NULL,1,1);					/* Start + Command index */
	
	WriteSpiMssg((uint16_t*)&arg, NULL,4,2);
	n = 0x0100;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x9500;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x8700;			/* Valid CRC for CMD8(0x1AA) */
	WriteSpiMssg(&n, NULL,1,1);

	/* Receive command response */
	if (cmd == CMD12) WriteSpiMssg( NULL, &n,1,1);		/* Skip a stuff uint8_t when stop reading */

	n = 10;								/* Wait for a valid response in timeout of 10 attempts */
	do
		WriteSpiMssg( NULL, &res,1,1);	
	while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/
DSTATUS MMC_disk_initialize(void)
{
	uint16_t n, cmd, ty, ocr[4];

	if (Stat & STA_NODISK) return Stat;	/* No card in the socket */

	power_on();							/* Force socket power on and initialize interface */
	spi_set_speed(INTERFACE_SLOW);
	for (n = 10; n; n--) WriteSpiMssg( NULL, &dum,1,1);	/* 80 dummy clocks with card de-selected */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		Timer1 = 100;						/* Initialization timeout of 1000 milliseconds */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDHC */
			for (n = 0; n < 4; n++) ocr[n] = WriteSpiMssg( NULL, &dum,1,1);		/* Get trailing return value of R7 response */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {				/* The card can work at VDD range of 2.7-3.6V */
				while (Timer1 && send_cmd(ACMD41, 1UL << 30));	/* Wait for leaving idle state (ACMD41 with HCS bit) */
				if (Timer1 && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = WriteSpiMssg( NULL, &dum,1,1);
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
				}
			}
		} else {							/* SDSC or MMC */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDSC */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMC */
			}
			while (Timer1 && send_cmd(cmd, 0));			/* Wait for leaving idle state */
			if (!Timer1 || send_cmd(CMD16, 512) != 0)	/* Set R/W block length to 512 */
				ty = 0;
		}
	} else {
		// xprintf("cmd 0 failed\n");
	}
	CardType = ty;
	release_spi();

	if (ty) {			/* Initialization succeeded */
		Stat &= ~STA_NOINIT;		/* Clear STA_NOINIT */
		spi_set_speed(INTERFACE_FAST);
	} else {			/* Initialization failed */
		power_off();
	}

	return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/
DSTATUS MMC_disk_status(void)
{
	return Stat;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT MMC_disk_read(
	uint8_t *buff,			/* Pointer to the data buffer to store read data */
	uint32_t sector,		/* Start sector number (LBA) */
	uint8_t count			/* Sector count (1..255) */
)
{
	if (!count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to uint8_t address if needed */

	if (count == 1) {	/* Single block read */
		if (send_cmd(CMD17, sector) == 0)	{ /* READ_SINGLE_BLOCK */
			if (rcvr_datablock(buff, 512)) {
				count = 0;
			}
		}
	}
	else {				/* Multiple block read */
		if (send_cmd(CMD18, sector) == 0) {	/* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512)) {
					break;
				}
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0);				/* STOP_TRANSMISSION */
		}
	}
	release_spi();

	return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
#if _FS_READONLY == 0
DRESULT MMC_disk_write(
	const uint8_t *buff,	/* Pointer to the data to be written */
	uint32_t sector,		/* Start sector number (LBA) */
	uint8_t count			/* Sector count (1..255) */
)
{
	if (!count) return RES_PARERR;
	if (Stat & STA_NOINIT) return RES_NOTRDY;
	if (Stat & STA_PROTECT) return RES_WRPRT;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to uint8_t address if needed */

	if (count == 1) {	/* Single block write */
		if ((send_cmd(CMD24, sector) == 0)	/* WRITE_BLOCK */
			&& xmit_datablock(buff, 0xFE))
			count = 0;
	}
	else {				/* Multiple block write */
		if (CardType & CT_SDC) send_cmd(ACMD23, count);
		if (send_cmd(CMD25, sector) == 0) {	/* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC)) break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD))	/* STOP_TRAN token */
				count = 1;
		}
	}
	release_spi();

	return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY == 0 */


DSTATUS MMC_disk_ioctl(
	uint8_t ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	uint8_t n, csd[16], *ptr = buff;
	WORD csize;

	res = RES_ERROR;

	if (ctrl == CTRL_POWER) {
		switch (*ptr) {
		case 0:		/* Sub control code == 0 (POWER_OFF) */
			if (socket_is_powered())
				power_off();		/* Power off */
			res = RES_OK;
			break;
		case 1:		/* Sub control code == 1 (POWER_ON) */
			power_on();				/* Power on */
			res = RES_OK;
			break;
		case 2:		/* Sub control code == 2 (POWER_GET) */
			*(ptr+1) = (uint8_t)socket_is_powered();
			res = RES_OK;
			break;
		default :
			res = RES_PARERR;
		}
	}
	else {
		if (Stat & STA_NOINIT) return RES_NOTRDY;

		switch (ctrl) {
		case CTRL_SYNC :		/* Make sure that no pending write process */
			select_card();
			if (wait_ready() == 0xFF)
				res = RES_OK;
			break;

		case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (uint32_t) */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
				if ((csd[0] >> 6) == 1) {	/* SDC version 2.00 */
					csize = csd[9] + ((WORD)csd[8] << 8) + 1;
					*(uint32_t*)buff = (uint32_t)csize << 10;
				} else {					/* SDC version 1.XX or MMC*/
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
					*(uint32_t*)buff = (uint32_t)csize << (n - 9);
				}
				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
			*(WORD*)buff = 512;
			res = RES_OK;
			break;

		case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (uint32_t) */
			if (CardType & CT_SD2) {	/* SDC version 2.00 */
				if (send_cmd(ACMD13, 0) == 0) {	/* Read SD status */
					WriteSpiMssg( NULL, &dum,1,1);
					if (rcvr_datablock(csd, 16)) {				/* Read partial block */
						for (n = 64 - 16; n; n--) WriteSpiMssg( NULL, &dum,1,1);	/* Purge trailing data */
						*(uint32_t*)buff = 16UL << (csd[10] >> 4);
						res = RES_OK;
					}
				}
			} else {					/* SDC version 1.XX or MMC */
				if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {	/* Read CSD */
					if (CardType & CT_SD1) {	/* SDC version 1.XX */
						*(uint32_t*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
					} else {					/* MMC */
						*(uint32_t*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
					}
					res = RES_OK;
				}
			}
			break;

		case MMC_GET_TYPE :		/* Get card type flags (1 uint8_t) */
			*ptr = CardType;
			res = RES_OK;
			break;

		case MMC_GET_CSD :		/* Receive CSD as a data block (16 uint8_ts) */
			if (send_cmd(CMD9, 0) == 0		/* READ_CSD */
				&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_CID :		/* Receive CID as a data block (16 uint8_ts) */
			if (send_cmd(CMD10, 0) == 0		/* READ_CID */
				&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_OCR :		/* Receive OCR as an R3 resp (4 uint8_ts) */
			if (send_cmd(CMD58, 0) == 0) {	/* READ_OCR */
				for (n = 4; n; n--) *ptr++ = WriteSpiMssg( NULL, &dum,1,1);
				res = RES_OK;
			}
			break;

		case MMC_GET_SDSTAT :	/* Receive SD status as a data block (64 uint8_ts) */
			if (send_cmd(ACMD13, 0) == 0) {	/* SD_STATUS */
				WriteSpiMssg( NULL, &dum,1,1);
				if (rcvr_datablock(ptr, 64))
					res = RES_OK;
			}
			break;

		default:
			res = RES_PARERR;
		}

		release_spi();
	}

	return res;
}

/*-----------------------------------------------------------------------*/
/* Device Timer Interrupt Procedure  (Platform dependent)                */
/*-----------------------------------------------------------------------*/
/* This function must be called in period of 10ms                        */
void MMC_disk_timerproc(void)
{
	static uint32_t pv;
	uint32_t ns;
	uint8_t n, s;

	n = Timer1;                /* 100Hz decrement timers */
	if (n) Timer1 = --n;
	n = Timer2;
	if (n) Timer2 = --n;

	ns = pv;
	pv = socket_is_empty() | socket_is_write_protected();	/* Sample socket switch */

	if (ns == pv) {                         /* Have contacts stabled? */
		s = Stat;

		if (pv & socket_state_mask_wp)      /* WP is H (write protected) */
			s |= STA_PROTECT;
		else                                /* WP is L (write enabled) */
			s &= ~STA_PROTECT;

		if (pv & socket_state_mask_cp)      /* INS = H (Socket empty) */
			s |= (STA_NODISK | STA_NOINIT);
		else                                /* INS = L (Card inserted) */
			s &= ~STA_NODISK;

		Stat = s;
	}
}

		
				