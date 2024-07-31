/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// enccoder

extern volatile int32_t encoder;

// direksyon için kumanda sinyali EXTI_2
extern volatile uint32_t starttime3;
extern volatile uint32_t endtime3 ;
extern volatile float pulsewidth3;
extern volatile uint32_t count3;
extern volatile uint32_t pulsewidth_prev;
// proximity 1 extı3
volatile uint16_t falling_detected2=0; //proximity2
// proximity2 extı5
volatile uint16_t falling_detected=0; //proximity1
volatile uint32_t counter2=0;

// TULPAR İÇİN KUMANDA SİNYALİ
volatile uint32_t starttime1=0;
volatile uint32_t endtime1=0;
volatile uint32_t pulsewidth1=0;
volatile uint32_t count1=0;
volatile int32_t dutycycle0=0;
volatile int32_t dutycycle1=0;
volatile int32_t dutycycle2=0;
extern volatile uint32_t  dutycyclenew1;
extern volatile uint32_t  dutycyclenew2;
volatile uint32_t degisken1=0;
volatile uint32_t degisken2=0;
// FREN İÇİN KUMANDA SİNYALİ
volatile uint32_t starttime = 0;
volatile uint32_t endtime = 0;
volatile uint32_t pulsewidth = 0;
volatile uint32_t count=0;
volatile uint32_t signal=0;
volatile uint32_t command=0;
extern volatile uint32_t pompa;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)==GPIO_PIN_SET){
		falling_detected=0;
	}
	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)== GPIO_PIN_RESET){
		falling_detected=1;
	}
	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)==GPIO_PIN_SET){
		falling_detected2=0;
	}
	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)== GPIO_PIN_RESET){
		falling_detected2=1;
	}
	pompa++;

  if(pompa<5000)
  {

 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,ENABLE); // PİN DE�?İ�?EBİLİR
  }
  else
  {

   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,DISABLE); // PİN DE�?İ�?EBİLİR

  }

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	// Encoderin A Sinyalinden çıkan kablo PB0 e bağlanacak
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_RESET)
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET)
			{

				encoder++;


			}
			else
			{

				encoder--;
			}



		}
		else
		{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_SET)
					{

						encoder--;


					}
					else
					{

						encoder++;
					}


		}

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	// // Encoderin B Sinyalinden çıkan kablo PB1 e  bağlanacak
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==GPIO_PIN_RESET)
			{
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_SET)
				{

					encoder--;


				}
				else
				{

					encoder++;
				}



			}
			else
			{
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)==GPIO_PIN_SET)
						{

							encoder++;


						}
						else
						{

							encoder--;
						}


			}


  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	// Kumandadan gelen sinyalin anlamlandırılması için kullanılan interrupt
		// alıcıdan gelen kablo PB2 portuna bağlanacak
		count3=TIM5->CNT;
					if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)==GPIO_PIN_SET)
					{

					starttime3=count3;

					}
					else
					{

						endtime3=count3;


					}
						if(endtime3>starttime3)
						{

							pulsewidth3=endtime3-starttime3;

						}


  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */


  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	// FREN İÇİN KUMANDA SİNYALİ
	count=TIM2->CNT;
				if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)==GPIO_PIN_SET)
				{
				starttime=count;
				}
				else
				{
					endtime=count;
				}
					if(endtime>starttime)
					{
						pulsewidth=endtime-starttime;


					}

					command = pulsewidth;

		if(command <=1500){
		signal = 1;
		}

		else {
		signal=0;
		}

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	// tulparı sürmek için kumanda sinyali PA15
	count1=TIM5->CNT;
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)==GPIO_PIN_SET)
				{
				starttime1=count1;
				}
				else
				{
					endtime1=count1;
				}
					if(endtime1>starttime1)
					{
						pulsewidth1=endtime1-starttime1;
						pulsewidth1=pulsewidth1/20;

					}
					dutycycle0=pulsewidth1;

					if(dutycycle0<=6298)
					{
						degisken1=-((dutycycle0-6298)/16);
						if(degisken1<25)
						{
							dutycyclenew1=0;
						}
						else
						{
							dutycyclenew1=degisken1;
						}


					}
					if(dutycycle0>6298)
					{
						degisken2=((dutycycle0-6298)/16);
							if(degisken2<25)
								{
							      dutycyclenew2=0;
								}
							else
								{
								  dutycyclenew2=degisken2;
								}


}

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
