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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//for brake
volatile uint32_t starttime = 0;
volatile uint32_t endtime = 0;
volatile uint32_t pulsewidth = 0;
volatile uint32_t count=0;
volatile uint32_t dutycycle=0;
volatile uint16_t falling_detected=0; //proximity1
volatile uint32_t signal=0;
volatile uint32_t command=0;
volatile uint32_t falling_detected2=0;//proximity2
//tulpa
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
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	count1=TIM5->CNT;
				if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)==GPIO_PIN_SET)
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

					if(dutycycle0<=2019)
					{
						degisken1=-((dutycycle0-2019)/5);
						if(degisken1<25)
						{
							dutycyclenew1=0;
						}
						else
						{
							dutycyclenew1=degisken1;
						}


					}
					if(dutycycle0>2019)
										{
											degisken2=((dutycycle0-2019)/5);
											if(degisken2<25)
											{
												dutycyclenew2=0;
											}
											else
											{
												dutycyclenew2=degisken2;
											}


										}

					//dutycycle0 = pulsewidth1;

//					if(dutycycle0<=-90)
//						{
//							if(dutycycle0>-102){
//								  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET); //REWERSE
//								  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, RESET);
//								  dutycycle1=(-((dutycycle0*2)+108)-72)*6;
//
////								  __HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1,dutycycle1);
////								  counter++;
//								}
//						}
//
//						if(dutycycle0>-90)
//						  {
//							  if(dutycycle0<-78)
//							  {
//								  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET); //FORWARD
//								  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
//								  dutycycle2=(((dutycycle0+90)*2)*6)-12;
//
////								  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,dutycycle2);
////								  counter--;
//							  }
//						  }

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(TULPAR_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)==GPIO_PIN_SET){
		falling_detected2=0;
	}
	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)== GPIO_PIN_RESET){
		falling_detected2=1;
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(PROX2_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
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
						pulsewidth=(((pulsewidth-84000)/672)-1);

					}

					command = pulsewidth;

		if(command <= 6391240){
		signal = 1;
		}

		else {
		signal=0;
		}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BRAKE_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)==GPIO_PIN_SET){
	falling_detected=0;
}
if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)== GPIO_PIN_RESET){
	falling_detected=1;
}


  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(PROX_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
