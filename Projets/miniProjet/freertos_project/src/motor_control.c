

		#include "board.h"
		#include "sct_capture.h"
		
		
		
		/*****************************************************************************
		 * Private types/enumerations/variables
		****************************************************************************/
		
		#define SCT_PWM 	LPC_SCT1 /* Use SCT1 for PWM */
		#define SCT_PWM_PIN_OUT 1 /* COUT1 Generate square wave */
		#define SCT_PWM_PIN_LED 0 /* COUT0 [index 2] Controls LED */
		
		#define SCT_PWM_OUT_INDEX 1 /* Index of OUT PWM */
		#define SCT_PWM_LED_INDEX 2 /* Index of LED PWM */
		#define SCT_PWM_RATE 50 /* PWM frequency 10 KHz 50 hrz*/
		
		/* Systick timer tick rate, to change duty cycle */
		#define TICKRATE_HZ 1000 /* 1 ms Tick rate */
		
		enum {ON=0x3fc0, OFF=0xbf40,  R= 0xdf20, G=0x5fa0};
		
		
		
		/*****************************************************************************
		 * Public types/enumerations/variables
		 ****************************************************************************/
		
		/*****************************************************************************
		 * Private functions
		 ****************************************************************************/
		
		/* Setup board specific pin muxing */
		static void pwm_setup_pin(void)
		{
			 /* Enable SWM clock before altering SWM */
			 Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
			
			#if defined(BOARD_NXP_LPCXPRESSO_1549)
			 /* Connect SCT output 1 to PIO0_29  and output 0 to PIO0.3*/
			 Chip_SWM_MovablePinAssign(SWM_SCT1_OUT1_O, 29);
			 Chip_SWM_MovablePinAssign(SWM_SCT1_OUT0_O, 3);
			#endif
			
			 Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
		}
		
		/*****************************************************************************
		 * Public functions
		 ****************************************************************************/
		

		
		/* Example entry point */
		void pwm_init(void)
		{
		 		
		 		
		 /* Initialize the SCT as PWM and set frequency */
		 Chip_SCTPWM_Init(SCT_PWM);
		 Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);
		
		 /* Setup Board specific output pin */
		 pwm_setup_pin();
		
			/* Use SCT0_OUT1 pin, match register, MATCH. Uses the specified match only,.... */
			/* Setup the OUTPUT pin corresponding to the PWM index */
			/*index: is index of match register used */
		 Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_OUT_INDEX, SCT_PWM_PIN_OUT);
		 Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_LED_INDEX, SCT_PWM_PIN_LED);
		
		 /* Start with 0% duty cycle */
			/* match register index to reload, number of tick*/
		 Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT_INDEX, Chip_SCTPWM_GetTicksPerCycle(SCT_PWM)/2); //50%
		 Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED_INDEX, 0);
		 Chip_SCTPWM_Start(SCT_PWM);
			
			/*init infrared interface */
			InfraRed_init();
		}
		
static uint8_t speedIncrease(uint8_t percent){
			
			if (percent==10)
				return 10;
			else{
				percent = percent+1;
				return percent;
			}
		}
static uint8_t speedDecrease(uint8_t percent){
			
			if (percent==1)
				return percent;
			else{
				percent = percent -1;
				return percent;
			}
		}

void pwm_task(void *pvParameters)
{			
		uint32_t command ;
		uint8_t percent=0;
//	void pwm_init(void);
	while (1)
	{
		xQueueReceive( irCommandQueue, &( command), portMAX_DELAY );
		DEBUGOUT(" receved command : 0X%X \r\n", command);
		
		switch(command & 0xffff)
		{
			case ON:
				percent = 10;
				break;
			case OFF:
				percent = 0;
				break;
			case R:
				percent = speedIncrease(percent);
				break;
			case G:
				percent = speedDecrease(percent);
				break;
			default:
				break;
		}
		
		 /* Increase dutycycle by 10% every second */
		 Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_OUT_INDEX,
	Chip_SCTPWM_PercentageToTicks(SCT_PWM,percent));  //Converts a percentage to ticks, Percentage to convert (0 - 100)
		 
		
		 /* Increment or Decrement Dutycycle by 0.5% every 10ms */
		 Chip_SCTPWM_SetDutyCycle(SCT_PWM, SCT_PWM_LED_INDEX,
		 Chip_SCTPWM_PercentageToTicks(SCT_PWM,percent)/2);
	}
		 
		 
	}
