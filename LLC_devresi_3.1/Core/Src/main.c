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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TOTAL_PAGES 6 //sayfa sayisi
#include "LCD1602.h"
#include "stdio.h"
void startButton(void);
void menuControl(void);//menulerde gezinme, kp ve ki, degerlerini azaltma
void pageProcess(int page);
void pageName(int numberPage);

char yazi[16]=" ";
char vol[16]=" ";
char fre[16]=" ";
char cur[16]=" ";
int changePage = 0;
uint32_t data[2];
uint32_t adcBuffer[2];
int period,frekans;


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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance==ADC1){
		data[0]=adcBuffer[0];
		data[1]=adcBuffer[1];
		
		
	}

}
void controlADC(){
	
	while(1){
		lcd_put_cur(0,0);
		sprintf(vol,"Voltage= %d " ,(data[0]*100)/4096);
		lcd_send_string(vol);
		lcd_put_cur(1,0);
		sprintf(cur,"Current= %d " ,(data[1]*5)/4096);
		lcd_send_string(cur);
		if(data[0]>2400){
			lcd_put_cur(0,0);
			lcd_send_string("Hedef Aralikta  ");
			lcd_put_cur(1,0);
			lcd_send_string("Menu gecis");
			HAL_Delay(2500);
			break;
			
		}
		
		
	}
	
}

void pageName(int numberPage){
	if(changePage==0){
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("Frekans okuma");
		lcd_put_cur(1,0);
		lcd_send_string("Akim okuma");
	}else if(changePage==1){
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("I Ayari");
	}else if(changePage==2){
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("P Ayari");
	
	}else if(changePage==3){
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("Sistem Baslatma");
	
	}else if(changePage==4){
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("Sistem Durdurma");
	
	}else if(changePage==5){
		lcd_clear();
		lcd_put_cur(0,0);
		lcd_send_string("Adc Okuma");
	
	}
	
}

void pageProcess(int page){
	switch(page){

	
		case 0:
				lcd_clear();
				
				while(1){
					lcd_put_cur(0,0);
					frekans=72000000/TIM1->ARR;
					sprintf(fre,"Frekans= %d",frekans);
					lcd_send_string(fre);
					lcd_put_cur(1,0);
					sprintf(cur,"Akim= %d " ,(data[1]*5)/4096);
					lcd_send_string(cur);
					if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==1){
							lcd_clear();
							lcd_put_cur(0,0);
							lcd_send_string("Cikis yapildi");
							menuControl();
					
					}
					
				
				}
			break;
		case 1:
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("KI islemi");
			break;
		case 2:
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("KP islemi");
				
			break;
		case 3:
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Baslatiliyor");
				HAL_Delay(2000);
				lcd_put_cur(0,0);
				lcd_send_string("Giris Voltage");
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);
				HAL_Delay(5000);
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Cikis gnd");
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,1);
				HAL_Delay(5000);
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Cikis Voltage");
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,1);
				HAL_Delay(5000);
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Frekans");
				lcd_put_cur(1,0);
				lcd_send_string("Baslatildi");
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
				HAL_Delay(2000);
				lcd_clear();
				
				lcd_send_string("Menuye gecildi");
				
				menuControl();
			break;
		case 4:
				
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Durduruluyor");
				HAL_Delay(2000);
				lcd_clear();
				lcd_send_string("Frekans");
				lcd_put_cur(1,0);
				lcd_send_string("Durduruluyor");
				HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
				HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
				HAL_Delay(5000);
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Giris Voltage");
				lcd_put_cur(1,0);
				lcd_send_string("Cikis Voltage");
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,0);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);
				HAL_Delay(5000);
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Cikis gnd");
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,0);
				HAL_Delay(2000);
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Devre Kapatildi");
				HAL_Delay(2000);
				lcd_clear();
				while(1){
					lcd_put_cur(0,0);
					sprintf(vol,"Voltage= %d " ,(data[0]*100)/4096);
					lcd_send_string(vol);
					lcd_put_cur(1,0);
					sprintf(cur,"Current= %d " ,(data[1]*5)/4096);
					lcd_send_string(cur);
					if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==1){
							lcd_clear();
							lcd_put_cur(0,0);
							lcd_send_string("Cikis yapildi");
							menuControl();
					
					}
				}
				
			break;
		case 5:
				lcd_clear();
				while(1){
					lcd_put_cur(0,0);
					sprintf(vol,"Voltage= %d " ,(data[0]*100)/4096);
					lcd_send_string(vol);
					lcd_put_cur(1,0);
					sprintf(cur,"Current= %d " ,(data[1]*5)/4096);
					lcd_send_string(cur);
					if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==1){
							lcd_clear();
							lcd_put_cur(0,0);
							lcd_send_string("Cikis yapildi");
							menuControl();
					
					}
				}
		break;
			
		default:
			lcd_clear();	
			lcd_put_cur(0,0);
			lcd_send_string("Sayfa Hatasi");
			
			
	}
		
			
	
		while(1){
		
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==1){
				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Cikis yapildi");
				menuControl();
		}
	
	}
	
}

void menuControl(void){
	while(1){
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==1){
			changePage = (changePage + 1) % TOTAL_PAGES;
			pageName(changePage);
			HAL_Delay(100);
		
		}else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==1){
		
			if (changePage == 0) {
				changePage = TOTAL_PAGES - 1;
				pageName(changePage);
			}else{
				changePage = changePage - 1;
				pageName(changePage);
				
			}
			HAL_Delay(100);
		
		}else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)==1){
			
				pageProcess(changePage);
			
		
		}
	
	}
	
}

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);

	period=TIM1->ARR;
	frekans=72000000/TIM1->ARR;
	HAL_ADC_Start_DMA(&hadc1,adcBuffer,2);
	
	
	lcd_init();
	lcd_put_cur(0,0);
	lcd_send_string("Merhaba");
	HAL_Delay(1500);
	lcd_clear();
	
	lcd_put_cur(0,0);
	lcd_send_string("Sistem Kontrol");
	lcd_put_cur(1,0);
	lcd_send_string("Ediliyor");
	HAL_Delay(5000);
	lcd_clear();
	
	controlADC();
	menuControl();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 822;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 411;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 32;
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           PA7 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
