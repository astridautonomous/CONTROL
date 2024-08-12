/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  ************************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t RX_Buffer [32] ;
uint8_t tour=0;
uint32_t Encoder=0;
uint32_t Brake=0;
uint32_t Drive=0;
// ENCODER
volatile int32_t encoder=0;
// direksiyon için kumanda sinyali değişkenleri extı_2
 volatile uint32_t starttime3=0;
 volatile uint32_t endtime3=0;
 volatile float pulsewidth3=0;
 volatile uint32_t count3=0;
 volatile uint32_t pulsewidth_prev=0;

 // proximity 1 extı3
 extern volatile uint32_t falling_detected2;
 // proximity2 extı5
 extern volatile uint16_t falling_detected;

 // TULPAR İÇİN KUMANDA DE�?İ�?KENLERİ
 extern volatile uint32_t starttime1;
 extern volatile uint32_t endtime1;
 extern volatile uint32_t pulsewidth1;
 extern volatile uint32_t count1;
 extern volatile int32_t dutycycle0;
 extern volatile int32_t dutycycle1;
 extern volatile int32_t dutycycle2;
 volatile uint32_t  dutycyclenew1=0;
 volatile uint32_t  dutycyclenew2=0;

 // FREN İÇİN KUMANDA SİNYALİ
 extern volatile uint32_t starttime;
 extern volatile uint32_t endtime;
 extern volatile uint32_t pulsewidth;
 extern volatile uint32_t count;
 extern volatile uint32_t command;
 extern volatile uint32_t signal;

 // pid kontrol değişkenleri
 volatile float ref_aci=0;
 volatile uint32_t t=0;
 volatile uint32_t t_prev=0;
 volatile float motoraci=0;
 volatile float aci1=0;
 volatile uint32_t aci1_prev=0;
 volatile int32_t dt=0;
 volatile int32_t error=0;
 volatile int32_t error_prev=0;
 volatile int32_t inte=0;
 volatile int32_t inte_prev=0;
 int32_t vmax=100;
 int32_t vmin=-100;
 volatile int32_t v=0;
 // tuning parametreleri
 float kp=0.2;
 float kd=0;
 float ki=0.1;
volatile uint32_t counter=0;
volatile uint32_t pompa=0;
uint32_t b=0;
 // BO�?
 volatile int AA=0;
 volatile uint32_t duty=0;
 volatile uint32_t BB=0;
 volatile uint32_t BA=0;
 volatile uint32_t motorhiz=0;

 volatile uint32_t periyot=0, periyot1=0, periyot2=0;
 int sınırpos = 5;
 int sınırneg = -5;
 uint32_t counnt=0,capture_value=0;
 float frequency=0,rpm=0;

 void motor_fonksiyonu()
 {
// 	if(dutycycle0<6298)
// 		{
// 			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, ENABLE); //REWERSE
// 			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, DISABLE);
// 			__HAL_TIM_SET_COMPARE(&htim3 ,TIM_CHANNEL_1, dutycyclenew1);
// 			counter++;
// 		}
//
// 	if(dutycycle0>6298)
// 		{
// 			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, DISABLE); //FORWARD
// 			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, ENABLE);
// 			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dutycyclenew2);
// 			counter--;
// 		}
// 	if(dutycycle0==6298)
// 	{
// 		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, DISABLE); //FORWARD
// 		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, DISABLE);
// 		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,dutycyclenew1);
//	}

	 if(dutycycle0>6298)
	  		{
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, DISABLE); //FORWARD
	  			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, ENABLE);
	  			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dutycyclenew2);
	  			//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, Drive);
	  			counter--;
	  		}
	  	else
	  	{
	  		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0);


	  	}
 }
 void motorfonksiyon_pid()
 {
	// direksiyon
// 	ref_aci=-((pulsewidth3-179000)/82)+10;
// 	aci1=encoder*0.12;
// 	motoraci=aci1+720;
// 	t=TIM2->CNT;
// 		  if(t<t_prev)
// 		  {
// 		 	  dt=(4294967295 - t_prev) + t + 1;
// 		  }
// 		 else
// 		  {
// 		 	  dt=t-t_prev;
// 		  }
//
//
// 	error=ref_aci-motoraci;// error hesabı
// 	inte=inte_prev+(dt*(error+error_prev)/2); // hatanın integrali
//
// 		 	// Hatayı sınırlıyoruz ki overflow olmasın
// 		  if (dt != 0)
// 		  {
// 		 	   int32_t max_inte=200;
// 		 	  int32_t min_inte=-200;
//
// 		 	  if(inte>max_inte)
// 		 	  {
// 		 		  inte=max_inte;
//
// 		 	  }
// 		 	  else if(inte<min_inte)
// 		 	  {
// 		 		  inte=min_inte;
//
// 		 	  }
// 		 	}
//
//
//
// 		 	v=(kp*error)+(ki*inte)+(kd*((error-error_prev)/dt)); //PID den çıkan sinyal
//
// 		 	if(v>vmax)
// 		 	{
// 		 		v=vmax;
// 	//	 		inte=inte_prev;
// 		 	}
// 		 	if(v<vmin)
// 		 	{
// 		 		v=vmin;
// 		 	//	inte=inte_prev;
//
// 		 	}
//
// 	 duty=(200*abs(v)/vmax);
// 	if(duty>200)
// 	{
//
// 		duty=200;
// 	}
// 	if(v<0)
// 	{
// 		// sağa dönüş
// 		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,duty);
// 		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
//BB=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_3);
//BA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_4);
// 	}
// 	else if(v>0)
// 	{
// 		//sola dönüş
// 		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
// 		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,duty);
// 		BB=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_3);
// 		BA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_4);
//
// 	}
// 	else
// 	{
// 		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
// 		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
// 		BB=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_3);
// 		BA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_4);
//
// 	}
// 	if(v==50)
// 	 	{
// 	 		periyot1=TIM4->CNT;
//
// 	 	}
// 	else if(v==-50)
// 	{
//
// 		periyot2=TIM4->CNT;
//
// 	}
// 	periyot=(periyot2-periyot1)/2000;
// 	t_prev=t;
// 	inte_prev=inte;
// 	error_prev=error;
//
// 	HAL_Delay(10);

	 	ref_aci=-((pulsewidth3-84000)/69.3)+1333;
		aci1=encoder*0.12;
		motoraci=aci1+720;
		error=ref_aci-motoraci;// error hesabı
		//error=Encoder-motoraci;
		if(error>sınırpos)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,60);
			sınırpos = 5;
			sınırneg = -5;

		}
		else if(error<sınırneg)
		{

			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,60);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
			sınırpos = 5;
			sınırneg = -5;
		}
		else
		{
			sınırneg = -10;
			sınırpos = 10;
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
		}

 }
// int _write(int file ,char *ptr,int len )
// {
//
//	 int i=0;
//	 for(i=0;i<len;i++)
//		 ITM_SendChar((*ptr++));
//	 return len;
// }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);// FREN MOTORU PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);// FREN MOTORU PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);// TULPAR PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);// DİREKSİYON PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);// DİREKSİYON  MOTORU LEFT PWM(YENİ)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);// DİREKSİYON PWM
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,ENABLE);// FREN RIGHT ENABLE
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,ENABLE);// FREN LEFT ENABLE
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,DISABLE);// TULPAR FORWARD ENABLE
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,DISABLE);// TULPAR REVERSE ENABLE
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, ENABLE); //DİREKSİYON RİGHT ENABLE
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, ENABLE);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,ENABLE);// DİREKSİYON LEFT ENABLE
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,ENABLE);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,DISABLE);

  // KALDIRILABİLİR ALTTAKİ KISIM
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,ENABLE);
// HAL_Delay(3000);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,DISABLE);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* USER CODE END WHILE */

	      /* USER CODE BEGIN 3 */

	  HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)RX_Buffer, 32);
	  if (HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)RX_Buffer, 32) == HAL_OK)
		{
			tour++;
		}
	  Encoder = RX_Buffer[1]+RX_Buffer[2]+RX_Buffer[3]+RX_Buffer[4]+RX_Buffer[5];
	  Brake = RX_Buffer[6];
	  Drive = RX_Buffer[7];

	  counnt++;
	  	  if(counnt>2000000)
	  	  {
	  	  	rpm=0;

	  	  }


b++;

	  		motor_fonksiyonu();
	  		motorfonksiyon_pid();
	  		AA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);

	  		//if(Brake == 1)
	  if(signal == 1){
			if(falling_detected2==1){
			  		while(1){

			  			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			  			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			  			motor_fonksiyonu();
			  			motorfonksiyon_pid();
			  			AA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);

			  			//if(Brake != 1||falling_detected2==0)
			  			if(signal != 1||falling_detected2==0){
			  				break;
			  				AA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);
			  			}
			  		}
			  	}
	  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); //holding the  motor
	  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 85);
	  	motor_fonksiyonu();
	  	motorfonksiyon_pid();
	  	AA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);



//	  	if(falling_detected2==0)
//	  	{
//
//	  		motor_fonksiyonu();
//	  		motorfonksiyon_pid();
//
//	  		AA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);
//	  	}
	  }
//	  if(Brake == 0)
	  if(signal == 0){
			if(falling_detected==1){
			  		while(1){
			  			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			  			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			  			motor_fonksiyonu();
			  			motorfonksiyon_pid();
			  			AA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);

			  			//if(Brake != 0||falling_detected==0)
			  			if(signal != 0||falling_detected==0){
			  				pompa=0;
			  				break;
			  			}
			  		}
			  	}
	  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60); //holding the  motor
	  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	  	motor_fonksiyonu();
	  	motorfonksiyon_pid();
	  	AA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);

//	  	if(falling_detected==0)
//	  		{
//	  			motor_fonksiyonu();
//	  			AA=__HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);
//	  			motorfonksiyon_pid();
//
//	  		}
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 64;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 85-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 19;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 124;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 41999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE5 PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 PA5 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
