/**
 ******************************************************************************
 * File Name          : dio.c
 * Date               : 04/01/2018 5:22:19
 * Description        : digital input / output processing
 ******************************************************************************
 *
 *
 ******************************************************************************
 */
 
 
/* Includes ------------------------------------------------------------------*/
#include "dio.h"
#include "pwm.h"

/* Typedef -------------------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
/* Variable ------------------------------------------------------------------*/
uint8_t din_state;
uint8_t relay_output[2];
volatile uint32_t error_signal_flags;
volatile uint32_t error_signal_timer;


/* Macro ---------------------------------------------------------------------*/
/* Program code   ------------------------------------------------------------*/
void DIO_Service(void)
{
	uint8_t j;
	
	static enum
	{
		DIO_INIT			= 0x00,
		DIO_SERVICE			= 0x01
		
	} DIN_ServiceState = DIO_INIT;
	
	
	switch(DIN_ServiceState)
	{
		/** ==========================================================================*/
		/**    		D I G I T A L    I / O   	I N I T I A L I Z A T I O N			  */
		/** ==========================================================================*/
		case DIO_INIT:
		{
			HC595_OutputDisable();
			relay_output[0] = 0;
			relay_output[1] = 0;
			HAL_SPI_Transmit(&hspi1, relay_output, 2, 10);
			HC595_ShiftLatch();
			HC595_OutputEnable();
			DIN_ServiceState = DIO_SERVICE;
			break;
		}		
		/** ==========================================================================*/
		/**  		D I G I T A L   I N P U T  	&  	R E L A Y   O U P U T    		  */
		/** ==========================================================================*/		
		case DIO_SERVICE:
		{
			/**
			*	get digital input state
			*/
			din_state = 0;
			
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) din_state |= 0x01;
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) din_state |= 0x02;
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_13) == GPIO_PIN_SET) din_state |= 0x04;
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_SET) din_state |= 0x08;
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) din_state |= 0x10;
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) din_state |= 0x20;
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) din_state |= 0x40;
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) din_state |= 0x80;
			/**
			*	set loop ventil state
			*/
			for(j = 0; j  < PWM_BUFFER_SIZE; j++)
			{
				if(pwm[j] >= PWM_ZERO_TRESHOLD) relay_output[1] |= 0x80;
			}
			/**
			*	set digital output state
			*/
			if(IsERROR_PwmControllerFailureActiv() 		||	\
				IsERROR_PwmOverloadProtectionActiv() 	||	\
				IsERROR_TemperatureSensorActiv()		||	\
				IsERROR_RadioLinkFailureActiv()			||	\
				IsERROR_PowerFailureActiv())
			{
				relay_output[0] = 0;
				relay_output[1] = 0;				
			}
			
			HAL_SPI_Transmit(&hspi1, relay_output, 2, 10);
			HC595_ShiftLatch();
			break;
		}
		
		
		default:
		{
			DIN_ServiceState = DIO_INIT;
			break;
		}
	}
}


/******************************   END OF FILE  ********************************/
