/*
===============================================================================
*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

// TODO: switch mode (comment DMA out for INTERRUPT mode)
#define DMA

/* SPI master handle and memory for ROM API */
SPI_HANDLE_T *spiHandleMaster;

SPI_PARAM_T param;
SPI_CONFIG_T spi_cfg;
SPI_DMA_CFG_T dma_cfg;
/* Use a buffer size larger than the expected return value of
   spi_get_mem_size() for the static SPI handle type */
static uint32_t spiMasterHandleMEM[0x20];
static DMA_HANDLE_T     dma_handle;	/* handle to DMA */

uint32_t receive_tag;

#define SIZE_BUFFERS            (1024)
uint16_t src[SIZE_BUFFERS], dst[SIZE_BUFFERS];

//Define the DMA receive callback function
static ErrorCode_t dma_transfer_callback(SPI_HANDLE_T handle, SPI_DMA_CFG_T *dma_cfg) {
	DMA_CHANNEL_T chn;
	DMA_TASK_T tsk;
	ErrorCode_t error_code;

	chn.event = DMA_ROM_CH_EVENT_PERIPH;
	chn.hd_trigger = DMA_ROM_CH_HWTRIG_BURSTPOWER_1; // no hardware trigger -> 0
	chn.priority = 3;
	if(param.cb != NULL) chn.cb_func = param.cb; // callback for transfer done. pageref 637 / section 40.4.8.5

	//receiver task
	tsk.ch_num = dma_cfg->dma_rxd_num;
	tsk.config 		= DMA_ROM_TASK_CFG_SEL_INTA | DMA_ROM_TASK_CFG_SW_TRIGGER;
	tsk.data_type 	= DMA_ROM_TASK_DATA_WIDTH_16 | DMA_ROM_TASK_SRC_INC_0 | DMA_ROM_TASK_DEST_INC_1;
	//attention on DMA_ROM_TASK_DATA_WIDTH_X -> need to be the width of the buffer.. this costs me 2 days
	tsk.data_length = param.size - 1;

	tsk.src = DMA_ADDR(&LPC_SPI0->RXDAT); /* Byte aligned */
	tsk.dst = DMA_ADDR(&dst[0] + tsk.data_length); //pointer to param.rx_buffer brings nothing.. need the real address
	tsk.task_addr = (uint32_t) NULL; // for task head, no memory is needed -> only for pingpong or linked list else write only to registers
	error_code = LPC_DMAD_API->dma_init(dma_handle, &chn, &tsk);
	if(error_code != LPC_OK) return error_code;

	//transmitter task
	tsk.ch_num = dma_cfg->dma_txd_num;
	//chn.cb_func = NULL;
	tsk.data_type = DMA_ROM_TASK_DATA_WIDTH_16 | DMA_ROM_TASK_SRC_INC_1 | DMA_ROM_TASK_DEST_INC_0;
	//attention on DMA_ROM_TASK_DATA_WIDTH_X -> need to be the width of the buffer.. this costs me 2 days
	tsk.src = DMA_ADDR(&src[0] + tsk.data_length); //pointer to param.tx_buffer brings nothing.. need the real address
	tsk.dst = DMA_ADDR(&LPC_SPI0->TXDAT); /* Byte aligned */
	tsk.task_addr = (uint32_t) NULL; // for task head, no memory is needed -> only for pingpong or linked list else write only to registers
	error_code = LPC_DMAD_API->dma_init(dma_handle, &chn, &tsk);
	return error_code;
}


static void Init_SPI_PinMux(void){
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 5, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 6, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 7, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));
	//Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 8, (IOCON_MODE_INACT | IOCON_DIGMODE_EN));

	Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO, 5);	/* P0.5 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, 6);/* P0.6 */
	Chip_SWM_MovablePinAssign(SWM_SPI0_MISO_IO, 7);/* P0.7 */
	//Chip_SWM_MovablePinAssign(SWM_SPI0_SSELSN_0_IO, 8);	/* P0.8 */

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
#ifdef RTOS
	vSemaphoreCreateBinary(spiEventSemaphore);
#endif
}

static void init_SPI_DMA_POINTER(void){
	/* Enable SPI clock and reset SPI peripheral - the boot ROM does not do this */
	Chip_SPI_Init(LPC_SPI0);
	/* Setup the SPI0 handle info{LPC_SPI0_BASE := (uint32_t)LPC_SPI0} */
	spiHandleMaster = LPC_SPID_API->spi_setup(LPC_SPI0_BASE, (uint8_t *) spiMasterHandleMEM);
	/* DMA initialization - enable DMA clocking and reset DMA if needed */
	Chip_DMA_Init(LPC_DMA);
	/*info{LPC_DMA_BASE := (uint32_t)LPC_DMA}*/
	dma_handle = LPC_DMAD_API->dma_setup(LPC_DMA_BASE, (uint8_t*)DMA_ADDR(Chip_DMA_Table));
	NVIC_EnableIRQ(DMA_IRQn);
}

/* Turn on LED to indicate an error */
 static void errorSPI(void)
{
	Board_LED_Set(0, true);
	while (1) {}
}

 //Define the receive callback function - This function is called when the DMA transfer has finished and sets the EOT bit to one in the SP TXDATCTL register (see Table 369) to set the SSEL signal to HIGH
/* n is 0 in dma mode pageref 637 / section 40.4.8.5 */
 static void receive_callback(ErrorCode_t err_code, uint32_t n) {
 	if(err_code != LPC_OK) errorSPI();

 // set the EOT flag in the TXCTL register
 	if (LPC_SPI0->CFG & SPI_MODE_MASTER) LPC_SPI0->TXCTRL |= SPI_TXDATCTL_EOT;
 	receive_tag = 1;
 }


 static void Setup_SPI_Master(void){
	/* Setup SPI0 configuration record */
	spi_cfg.delay = SPI_DLY_PRE_DELAY(0) | SPI_DLY_POST_DELAY(0) |
						 SPI_DLY_FRAME_DELAY(0) | SPI_DLY_TRANSFER_DELAY(0);
	/* SysClock divided is set to maximum */
	spi_cfg.divider = 0x0001; //frequency PCLK/(Value+1) pageref 407 - section 25.6.10
	/* Loopback mode, master mode and SPI block enabled - refpage 398 */
	spi_cfg.config = (1 << 0) | // SPI enabled
					 (1 << 2) | // mode 1 = Master, 0 = Slave
					 (0 << 3) | // LSB First mode (MSB first order.
					 (1 << 4) | // CPHA
					 (1 << 5) | // CPLO
					 (1 << 7) | // Loopback mode enabled
					 (0 << 8) ; // SSEL0 low active
	spi_cfg.error_en = SPI_STAT_RXOV | SPI_STAT_TXUR;

	dma_cfg.dma_txd_num = DMAREQ_SPI0_TX;
	dma_cfg.dma_rxd_num = DMAREQ_SPI0_RX;
	dma_cfg.hDMA = dma_handle;

	/* Init SPI0 */
	LPC_SPID_API->spi_init(spiHandleMaster, &spi_cfg);

	/* Enable SPI0 interrupt */
	NVIC_EnableIRQ(SPI0_IRQn);
}

/**
 * @brief	Handle SPI0 interrupt by calling SPI ROM handler
 * 			not needed for DMA Transfer
 * @return	Nothing
 */
void SPI0_IRQHandler(void)
{
	/* Call SPI ISR function in ROM with the SPI handle */
	LPC_SPID_API->spi_isr(spiHandleMaster);
}

/**
 * @brief	DMA Interrupt Handler
 * @return	None
 */
void DMA_IRQHandler(void)
{
	/* Call ROM driver handler */
	LPC_DMAD_API->dma_isr(dma_handle);
}

//DMA transmits data. Slave sends some data back to the DMA
void Transmit(void) {
	receive_tag = 0;

	//Set up the SPI parameter structure SPI_PARAM_T for the SPI
	param.tx_buffer = (uint16_t *)src;
	param.size = SIZE_BUFFERS; //x transfers of TXDATCTL_FSIZE size data.
	param.rx_buffer = (uint16_t *)dst;
	param.fsize_sel = 0x0F0E0000; //SPI_TXDATCTL_ASSERT_SSEL0 | SPI_TXDATCTL_LEN(16);
#ifdef DMA
	param.driver_mode = 0x02; // DMA mode
#else
	param.driver_mode = 0x01; // Interrupt mode
#endif
	param.tx_rx_flag = 0x02; // TX AND RX needed to generate clock..
	param.eof_flag = 0;
	param.cb = (SPI_CALLBK_T) receive_callback;
	// DMA set-up - must not be NULL for Interrupt mode
	param.dma_cfg = &dma_cfg;
	param.dma_cb = (SPI_DMA_REQ_T) dma_transfer_callback;

	int error = LPC_SPID_API->spi_master_transfer(spiHandleMaster, &param);
	if (error != LPC_OK) {
		/* Signal SPI error */
		errorSPI();
	}
	while (!receive_tag){
		Chip_PMU_SleepState(LPC_PMU); //wait for receive tag to be set
	}
}

int main(void) {
	int i;
    SystemCoreClockUpdate();
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);

    Init_SPI_PinMux();
    init_SPI_DMA_POINTER();
    Setup_SPI_Master();
    /* fill src buffer */
    for(i = 0; i < SIZE_BUFFERS; i++ ){
    	src[i] = i;
    }
	/* Loop forever */
	while (1) {
		/* Write simple message over SPI */
		Transmit();
		/* Toggle LED to show activity. */
		Board_LED_Toggle(0);
	}

	/* Code never reaches here. Only used to satisfy standard main() */
	return 0;
}