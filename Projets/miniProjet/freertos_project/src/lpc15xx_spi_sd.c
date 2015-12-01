

#include "board.h"
#include "lpc15xx_spi_sd.h"


static SPI_DATA_SETUP_T  XferSetup;


static SPI_CFG_T spiCfg; // SPI configuration struct

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

static void setupSpiMaster(void)
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
	spiDelayCfg.FrameDelay = 0;
	spiDelayCfg.TransferDelay = 2;
	Chip_SPI_DelayConfig(LPC_SPI0, &spiDelayCfg);
		/* Enable SPI0 */
	Chip_SPI_Enable(LPC_SPI0);
}

/**
  * @brief  Configure SPI0 clock rate.
  *
  * @param  SPI_CLOCKRATE: Specifies the SPI clock rate.
  *         The value should be SPI_CLOCKRATE_LOW or SPI_CLOCKRATE_HIGH.
  * @retval None 
  *
  *
  */

void SPI_ConfigClockRate (uint32_t SPI_CLOCKRATE)
{
		Chip_SPI_Disable(LPC_SPI0);
    spiCfg.ClkDiv  = Chip_SPI_CalClkRateDivider(LPC_SPI0, SPI_CLOCKRATE); 
		Chip_SPI_SetConfig(LPC_SPI0, &spiCfg);
		Chip_SPI_Enable(LPC_SPI0);
}

/**
  * @brief  Set SSEL to low: select spi slave.
  *
  * @param  None.
  * @retval None 
  */
void SPI_CS_Low (void)
{
        // SSEL0 P0.27 low
	/* Assert only SSEL0 */
	XferSetup.ssel = SPI_TXCTL_ASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1 | SPI_TXCTL_DEASSERT_SSEL2 |
					 SPI_TXCTL_DEASSERT_SSEL3;  
//	Chip_GPIO_SetPinState(LPC_GPIO, 0 , 27, 0);	
}

/**
  * @brief  Set SSEL to high: de-select spi slave.
  *
  * @param  None.
  * @retval None 
  */
void SPI_CS_High (void)
{
    /* SSEL is GPIO, set to high.  */
    // SSEL0 high
	/* Assert only SSEL0 */
	XferSetup.ssel = SPI_TXCTL_DEASSERT_SSEL0 | SPI_TXCTL_DEASSERT_SSEL1 | SPI_TXCTL_DEASSERT_SSEL2 |
					 SPI_TXCTL_DEASSERT_SSEL3;    
//	Chip_GPIO_SetPinState(LPC_GPIO, 0 , 27, 1);  
}


/**
  * @brief  Initializes the SPI0.
  *
  * @param  None
  * @retval None 
  */
void SPI_Init (void) 
{
	Init_SPI_PinMux();
	setupSpiMaster();

    /* Configure SPI0 clock rate to 400kHz  */
    SPI_ConfigClockRate (SPI_CLOCKRATE_LOW);

    /* Set SSEL to high */
    SPI_CS_High ();
}





/**
  * @brief  Send one byte via MOSI and simutaniously receive one byte via MISO.
  *
  * @param  data: Specifies the byte to be sent out.
  * @retval Returned byte.
  *
  * Note: Each time send out one byte at MOSI, Rx FIFO will receive one byte. 
  */

uint8_t SPI_SendByte (uint8_t data)
{
	uint16_t TxBuff = (uint16_t)(data);
	uint16_t RxBuff;
	/* Setup Transfer structure, this data should be retained for the entire transmission */
	XferSetup.pTx = &TxBuff;	/* Transmit Buffer */
	XferSetup.pRx = &RxBuff;/* Receive Buffer */
	XferSetup.DataSize = 8;	/* Data size in bits */
	XferSetup.Length = 1;	/* Total frame length : one frame*/
	XferSetup.TxCnt = 0;
	XferSetup.RxCnt = 0;
	Chip_SPI_RWFrames_Blocking(LPC_SPI0, &XferSetup) ;
	
	printf(" received byte:0x%x\r\n", RxBuff);
    /* Return the received value */              
    return (uint8_t)RxBuff;                        
}

/**
  * @brief  Receive one byte via MISO.
  *
  * @param  None.
  * @retval Returned received byte.
  */
uint8_t SPI_RecvByte (void)
{
    /* Send 0xFF to provide clock for MISO to receive one byte */
    return SPI_SendByte (0xFF);
}

/* --------------------------------- End Of File ------------------------------ */
