
	#include "board.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "portmacro.h"

	/**
	 * @brief SCT0 timer : used to capture infrared code. the out put of tsop38 is conncted to P1.6
	 */

	#define SCT0_INTERRUPT_PRIORITY 1
	#define SCT_CONFIG_INSYNC_IPUT0     (0x7f << 9)	/*!< SCT clock */
	#define CST0_CLOCK_PRESCAL 71  // clock cst0 with 1MHZ

	/* RC5  protocol timing */
	#define BIT_START_TIME_MIN  	5000 /* us*/
	#define BIT_START_TIME_MAX  	5100 /* us*/
	#define BIT_0_TIME_MIN  			2100  /* us*/
	#define BIT_0_TIME_MAX  			2400/* us*/
	#define BIT_1_TIME_MIN  			1000  /* us*/
	#define BIT_1_TIME_MAX  			1300 /* us*/
	#define FRAME_LENGTH  				33 /* start bit + 32 bit data*/

	

QueueHandle_t irCommandQueue;

	 
	
static uint32_t  count=0 , command=0, lastCapValue, rCapValue, period;

void SCT0_IRQHandler(void)
{
	/* xHigherPriorityTaskWoken must be set to pdFALSE before it is used. */
		static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
/* failing edge  */
	if (LPC_SCT0->EVFLAG & SCT_EVT_0) {
		Chip_SCT_ClearEventFlag(LPC_SCT0,SCT_EVT_0);				//Clear interrupt flag					
			
		
		
		
	}
	if (LPC_SCT0->EVFLAG & SCT_EVT_1){
		Chip_SCT_ClearEventFlag(LPC_SCT0,SCT_EVT_1);			//Clear interrupt flag event 1
		if(count==0){
			lastCapValue = LPC_SCT0->CAP[1].U; /* command start*/
			count++;
		}else {	
			
				rCapValue = LPC_SCT0->CAP[1].U;
				period =rCapValue-	lastCapValue; 
				lastCapValue = rCapValue;
				if ((period>BIT_START_TIME_MIN  )&&(period<BIT_START_TIME_MAX )) { // start frame
					command = 0;
				}else 
					if(( count<FRAME_LENGTH)&&(period>BIT_1_TIME_MIN)&&(period<BIT_1_TIME_MAX)){
						command = (command<<1 )+1;
						count++;
					}else 
						if(( count<FRAME_LENGTH)&&(period>BIT_0_TIME_MIN)&&(period<BIT_0_TIME_MAX)){
							command = (command<<1 );
							count++;
						}else if(count==FRAME_LENGTH){	
									xQueueSendFromISR( irCommandQueue, &command, &xHigherPriorityTaskWoken );
									portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
								count=0;	
														
						}else{
							/* frame decode error */
							count =0;
						}
			}
		}
}
		
				


	void InfraRed_init(void) {
		/* reset flags */
		
		DEBUGOUT(" system clock: %u \r\n", Chip_Clock_GetSystemClockRate());
		 /* Create a queue capable of containing 10 uint32_t values. */
		irCommandQueue = xQueueCreate( 10, sizeof( uint32_t ) );
    if( irCommandQueue == 0 )
    {
        DEBUGOUT("out of memory FreeRTOS cannot create irCommandQueue\r\n");
    }
		
		/* pin used to capture tsop38238 signal P1.6*/
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, 1, 6);

		/* clock sct0 enable and reset timer */
		Chip_SCT_Init(LPC_SCT0);
		/* 32 bit timer
		* bus clock
		*
		*/
		Chip_SCT_Config(LPC_SCT0, SCT_CONFIG_32BIT_COUNTER|SCT_CONFIG_CLKMODE_BUSCLK|
						SCT_CONFIG_INSYNC_IPUT0);

		

		Chip_SCT_SetCount(LPC_SCT0 ,0);									//set counter to 0

LPC_SCT0->EVEN = SCT_EVT_0|SCT_EVT_1;								//enable event 0 and event 1

	 /* REGMODEn = 1: Register0 operate as capture and capture control register */
		LPC_SCT0->REGMODE |= (SCT_EVT_0 |SCT_EVT_1);							//register 0 and 1 are a capture register


		LPC_SCT0->CAPCTRL[0].U |= (SCT_EVT_0);						//event 0 causes capture 0
		LPC_SCT0->CAPCTRL[1].U |= (SCT_EVT_1);						//event 1 causes capture 1

		LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF;					//happens in every state
		LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF;					//happens in every state

		/* defines the conditions for event n to occur */
		LPC_SCT0->EVENT[0].CTRL = 	(0 << 0) |		        			//use capture register 0
									(0x2 << 10) |				//detect falling edges
									(0x2 << 12) |				//use I/O condition only
									(1 << 14) |					//STATEV is loaded
									(0 << 15);					//new state: 0
									/* defines the conditions for event n to occur */
		LPC_SCT0->EVENT[1].CTRL = 	(0 << 0) |
									(0x1 << 10) |				//detect raising edges
									(0x2 << 12) |				//use I/O condition only
									(1 << 14) |					//STATEV is loaded  (reset vlaue undefined)
									(0 << 15);					//new state: 0 (reset value undefined ) !!!

	/* Selects an input source for SCT0 input: P1.6 maped to input 0*/
		Chip_INMUX_SelectSCT0Src(0, SCT0_INMUX_PIO1_6);
		
		/* set prescalr register to 71 SCT0 clocked system bus 72MHZ/72 = 1MHZ*/
		Chip_SCT_SetControl(LPC_SCT0, SCT_CTRL_PRE_L(CST0_CLOCK_PRESCAL));

		//	timerFreq = Chip_Clock_GetSysTickClockRate();

		NVIC_SetPriority(SCT0_IRQn, SCT0_INTERRUPT_PRIORITY); // interrupt priority

		Chip_SCT_EnableEventInt(LPC_SCT0, SCT_EVT_0|SCT_EVT_1); 			/* Enable an Interrupt on the Capture Event */

		NVIC_EnableIRQ(SCT0_IRQn);




		Chip_SCT_ClearControl(LPC_SCT0, SCT_CTRL_HALT_L|SCT_CTRL_HALT_H );//start counter
	}
	