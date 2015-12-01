
 
#include "board.h"
#include "FreeRTOS.h"
#include "semphr.h"










/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* I2CS transfer record for master and slave operations */
static I2CM_XFER_T  i2cmXferRec;

/* I2C clock is set to 1.8MHz */
#define I2C_CLK_DIVIDER     (40)

/* 100KHz I2C bit-rate - going too fast may prevent the salev from responding
   in time */
#define I2C_BITRATE         (50000)
/* Standard I2C mode */
#define I2C_MODE    (0																																																																																																																																																																								)

/* Emulated EEPROM slave addresses */
#define EEPROM0SLVADD       (0x28)
#define EEPROM1SLVADD       (0x2C)

#if defined(BOARD_NXP_LPCXPRESSO_1549)
/** Our slave address and I2C information */
#define LPC_I2C_PORT         LPC_I2C0
#define LPC_I2C_INTHAND      I2C0_IRQHandler
#define LPC_IRQNUM           I2C0_IRQn
#endif

/* Emulated EEPROM device2 - size, buffer, and current address */
#define EMUEEPROMSIZE 512
//static uint8_t eepromData[2][EMUEEPROMSIZE];
//static uint16_t eepromAddr[2];
static int curSlave, addrbytes;

/* work buffers for this example */
uint8_t txWorkBuff[EMUEEPROMSIZE], rxWorkBuff[EMUEEPROMSIZE];
static SemaphoreHandle_t xI2cSemaphore;
static  BaseType_t xHigherPriorityTaskWoken;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initializes pin muxing for I2C interface - note that SystemInit() may
   already setup your pin muxing at system startup */
static void Init_I2C_PinMux(void)
{
#if defined(BOARD_NXP_LPCXPRESSO_1549)
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_DIGMODE_EN | I2C_MODE);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
	Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
#else
	/* Configure your own I2C pin muxing here if needed */
#error "No I2C Pin Muxing defined for this example"
#endif
}

/* Setup I2C */
static void setupI2CMaster(void)
{
	/* Enable I2C clock and reset I2C peripheral */
	Chip_I2C_Init(LPC_I2C_PORT);

	/* Setup clock rate for I2C */
	Chip_I2C_SetClockDiv(LPC_I2C_PORT, I2C_CLK_DIVIDER);
	
	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C_PORT, I2C_BITRATE);

//	/* Enable I2C master interface */
	//Chip_I2CM_Enable(LPC_I2C_PORT);
}

/* Setup I2C */
static void setupI2CSlave(void)
{
	/* Some common I2C init was performed in setupI2CMaster(), so it doesn't
	   need to be done again for the slave setup. */

	/* Emulated EEPROM 0 is on slave index 0 */
	Chip_I2CS_SetSlaveAddr(LPC_I2C_PORT, 0, EEPROM0SLVADD);
	/* Disable Qualifier for Slave Address 0 */
	Chip_I2CS_SetSlaveQual0(LPC_I2C_PORT, false, 0);
	/* Enable Slave Address 0 */
	Chip_I2CS_EnableSlaveAddr(LPC_I2C_PORT, 0);

	/* Emulated EEPROM 1 is on slave index 1 */
	Chip_I2CS_SetSlaveAddr(LPC_I2C_PORT, 1, EEPROM1SLVADD);
	/* Enable Slave Address 1 */
	Chip_I2CS_EnableSlaveAddr(LPC_I2C_PORT, 1);

	/* Clear interrupt status and enable slave interrupts */
	Chip_I2CS_ClearStatus(LPC_I2C_PORT, I2C_STAT_SLVDESEL);
	Chip_I2C_EnableInt(LPC_I2C_PORT, I2C_INTENSET_SLVPENDING | I2C_INTENSET_SLVDESEL);

	/* Enable I2C slave interface */
	Chip_I2CS_Enable(LPC_I2C_PORT);
}




/* Handler for slave start callback */
static void processSlaveTransferStart(uint8_t addr)
{
	if (addr == EEPROM0SLVADD) {
		curSlave = 0;
	}
	else {
		curSlave = 1;
	}

	addrbytes = 0;
}

/* Handler for slave send callback */
static uint8_t processSlaveTransferSend(uint8_t *data)
{
	/* Send data from emulated EEPROM */
	*data = txWorkBuff[addrbytes++];

	return 0;
}

/* Handler for slave receive callback */
static uint8_t processSlaveTransferRecv(uint8_t data)
{
	
	rxWorkBuff[addrbytes++] =data;

	return 0;
}

/* Handler for slave transfer complete callback */
static void processSlaveTransferDone(void)
{
	/* Is it time for vATask() to run? */
    xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( xI2cSemaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/* I2C slavecallback function list */
const static I2CS_XFER_T i2csCallBacks = {
	&processSlaveTransferStart,
	&processSlaveTransferSend,
	&processSlaveTransferRecv,
	&processSlaveTransferDone
};

/* Populate EEPROM with some initial data */
static void fillWorkBuff(uint8_t *buff, uint8_t seed)
{
	int i;

	for (i = 0; i < EMUEEPROMSIZE; i++) {
		buff[i] = seed;
		seed = seed + (uint8_t) i;
	}
}



/* Compate 2 fixed length buffers */
static bool compFail(uint8_t *ptr1, uint8_t *ptr2)
{
	int i;

	for (i  = 0; i < EMUEEPROMSIZE; i++) {
		if (ptr1[i] != ptr2[i]) {
			return true;
		}
	}

	return false;
}


/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle I2C1 interrupt by calling I2CM interrupt transfer handler
 * @return	Nothing
 */
void LPC_I2C_INTHAND(void)
{
	uint32_t state = Chip_I2C_GetPendingInt(LPC_I2C_PORT);

	
	/* I2C slave related interrupt */
	while (state & (I2C_INTENSET_SLVPENDING | I2C_INTENSET_SLVDESEL)) {
		Chip_I2CS_XferHandler(LPC_I2C_PORT, &i2csCallBacks);

		/* Update state */
		state = Chip_I2C_GetPendingInt(LPC_I2C_PORT);
	}
}
	
	
	
	void i2c_setup(void)
{
	/* Setup I2C pin muxing */
	Init_I2C_PinMux();

	/* Setup I2C, master, and slave */
	setupI2CMaster();
  setupI2CSlave();

	/* Enable the interrupt for the I2C */
	NVIC_EnableIRQ(LPC_IRQNUM);
		
	xI2cSemaphore =xSemaphoreCreateBinary();
	

	DEBUGOUT(" I2C slaves are ready\r\n");
}

/**
 * @brief	Main routine for I2C example
 * @return	Function should not exit
 */
void i2C_slave_Task (void *pvParameters)
{
uint8_t i;
	

	/* Test I2c slave  */
	while (1) {
		
//		DEBUGOUT(" matched @: %u\r\n",Chip_I2C_GetPendingInt(LPC_I2C0));
		i = Chip_I2CS_ReadByte(LPC_I2C0);
		
  	xSemaphoreTake( xI2cSemaphore, portMAX_DELAY );
//		DEBUGOUT(" read value: 0x%x\r\n",i);
		
		DEBUGOUT(" I2C  transaction complete\r\n");
		
	}

	
	
}
