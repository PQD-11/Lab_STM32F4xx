/* USER CODE BEGIN Header */
/** bai1
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIGNAL_LEN 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef Node1_TxHeader;
CAN_RxHeaderTypeDef Node1_RxHeader;

uint8_t Node1_TxData[8];
uint8_t Node1_RxData[8];

uint32_t Node1_TxMailbox;

int timer_adc = 0;
int timer_10s = 1;
uint16_t var1 = 0;
uint32_t Seed;
uint32_t KeyECU;
uint32_t KeyTester;
int is_validKey = 1;
int caseFalse = 0;
int is_start = 0;
uint8_t left_button = 0;
uint8_t right_button = 0;
uint8_t middle_button = 0;
uint16_t data_length;
uint8_t number_CF;
uint8_t current_pos;
uint8_t SNum;

uint8_t LEFT_DATA[SIGNAL_LEN] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
uint8_t RIGHT_DATA[SIGNAL_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t MIDDLE_DATA[SIGNAL_LEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t is_get_Write_data_by_id_node1 = 0;
uint8_t is_get_Write_data_by_id_node2 = 0;

CAN_TxHeaderTypeDef Node2_TxHeader;
CAN_RxHeaderTypeDef Node2_RxHeader;

uint8_t Node2_TxData[8];
uint8_t Node2_RxData[8];

uint32_t Node2_TxMailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t RandomSeed()
{
	Seed = rand() % (0xFFFFFFFF - 0x1) + 1;
	return Seed;
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Node1_RxHeader, Node1_RxData);

	// if this is send seed (from ECU to tester)
	if (Node1_RxHeader.DLC == 8 && Node1_RxData[1] == 0x67 && Node1_RxData[2] == 0x01)
	{
		Node1_TxData[0] = 0x06;
		Node1_TxData[1] = 0x27;
		Node1_TxData[2] = 0x02;
		Node1_TxData[3] = Node1_RxData[3] + 1;
		Node1_TxData[4] = Node1_RxData[4] + 1;
		Node1_TxData[5] = Node1_RxData[5] + 1;
		Node1_TxData[6] = Node1_RxData[6] + 1;
	    HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
		 &Node1_TxMailbox);
	}

	// if this is response access
	if (Node1_RxHeader.DLC == 8 && Node1_RxData[1] == 0x67 && Node1_RxData[2] == 0x02)
	{
		printf("unlock ECU\r\n\n");
		is_validKey = 1;
		timer_10s = 0;
		is_start = 1;
		HAL_TIM_Base_Start_IT(&htim2);
	}
	if (Node1_RxHeader.DLC == 8 && Node1_RxData[1] == 0x7F && Node1_RxData[2] == 0x67)
	{
		printf("Nagative response from ECU!\r\n");
		if (Node1_RxData[3] == 0x35)
		{
			printf("Invalid key!\r\n");
	  		printf("\nPerpare for request new seed...\r\n\n");
			is_validKey = 0;
			is_start = 1;
			HAL_TIM_Base_Start_IT(&htim3);
		}

	}
	// adc
	if (Node1_RxHeader.DLC == 8 && Node1_RxData[1] == 0x62)
	{
		printf("Tester received ADC value: %x-%x-%x-%x-%x-%x-%x-%x\n", Node1_RxData[0],Node1_RxData[1],Node1_RxData[2],Node1_RxData[3], Node1_RxData[4], Node1_RxData[5], Node1_RxData[6], Node1_RxData[7]);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED0_Pin);
	}

	// joystick
	if (Node1_RxHeader.StdId == 0x7A2 && (Node1_RxData[0] >> 4) == 0x3) //Flow control
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			printf("Tester received FC: %x-%x-%x-%x-%x-%x-%x-%x\r\n", Node1_RxData[0],Node1_RxData[1],Node1_RxData[2],Node1_RxData[3], Node1_RxData[4], Node1_RxData[5], Node1_RxData[6], Node1_RxData[7]);

			SNum = 0x00;

			if((data_length-3) % 7 == 0)
				number_CF = (data_length-3)/7;
			else
				number_CF = ((data_length-3)/7) + 1;

			if(left_button == 1 && SNum <= 0x0F)
			{
				for(int i = 0; i < number_CF; i++)
				{
					Node1_TxData[0] = 0x21 + SNum;
					SNum += 0x01;
					for(int j = 1; j<8; j++)
					{
						if(current_pos <= SIGNAL_LEN)
							Node1_TxData[j]=LEFT_DATA[current_pos++];
						else
							Node1_TxData[j] = 0x00;
					}
					HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData, &Node1_TxMailbox);
				}
			left_button = 0;
			}

			else if(right_button == 1 && SNum <= 0x0F)
			{
				for(int i = 0; i < number_CF; i++)
				{
					Node1_TxData[0] = 0x21 + SNum;
					SNum += 0x01;
					for(int j = 1; j<8; j++)
					{
						if(current_pos <= SIGNAL_LEN)
							Node1_TxData[j]=RIGHT_DATA[current_pos++];
						else
							Node1_TxData[j] = 0x00;
					}
					HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData, &Node1_TxMailbox);
				}
			right_button = 0;
			}

			else if(middle_button == 1 && SNum <= 0x0F)
			{
				for(int i = 0; i < number_CF; i++)
				{
					Node1_TxData[0] = 0x21 + SNum;
					SNum += 0x01;
					for(int j = 1; j<8; j++)
					{
						if(current_pos <= SIGNAL_LEN)
							Node1_TxData[j]=MIDDLE_DATA[current_pos++];
						else
							Node1_TxData[j] = 0x00;
					}
					HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData, &Node1_TxMailbox);
				}
			middle_button = 0;
			}
		}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Node2_RxHeader, Node2_RxData);

	// if this is request seed (from tester to ECU)
	if (Node2_RxHeader.DLC == 8 && Node2_RxData[1] == 0x27 && Node2_RxData[2] == 0x01)
	{
		printf("Request seed\r\n");
//		HAL_Delay(10);
		Node2_TxData[0] = 0x06;
		Node2_TxData[1] = Node2_RxData[1] + 0x40;
		Node2_TxData[2] = 0x01;
		RandomSeed();
		printf("Seed: %x\r\n",Seed);
		Node2_TxData[3] = (Seed & 0xff000000UL) >> 24;
		Node2_TxData[4] = (Seed & 0x00ff0000UL) >> 16;
		Node2_TxData[5] = (Seed & 0x0000ff00UL) >>  8;
		Node2_TxData[6] = (Seed & 0x000000ffUL);
//		printf("compare Seed: %x-%x-%x-%x\n", Node2_TxData[3], Node2_TxData[4], Node2_TxData[5], Node2_TxData[6]);
		KeyECU = Seed + 0x01010101 + caseFalse;
		printf("KeyECU: %x\r\n",KeyECU);
		HAL_CAN_AddTxMessage(&hcan2, &Node2_TxHeader, Node2_TxData,
					 &Node2_TxMailbox);
	}
	// if this is send key (from tester to ECU)
	if (Node2_RxHeader.DLC == 8 && Node2_RxData[1] == 0x27 && Node2_RxData[2] == 0x02)
	{

		KeyTester = (Node2_RxData[3] << 24) + (Node2_RxData[4] << 16) + (Node2_RxData[5] << 8) + Node2_RxData[6];
		printf("Keytester: %x\r\n\n",KeyTester);
		if (KeyTester == KeyECU)
		{
			Node2_TxData[0] = 0x02;
			Node2_TxData[1] = Node2_RxData[1] + 0x40;
			Node2_TxData[2] = 0x02;
		}
		else
		{
			Node2_TxData[0] = 0x03;
			Node2_TxData[1] = 0x7F;
			Node2_TxData[2] = Node2_RxData[1] + 0x40;
			Node2_TxData[3] = 0x35;
		}
		HAL_CAN_AddTxMessage(&hcan2, &Node2_TxHeader, Node2_TxData,
					 &Node2_TxMailbox);
	}
	// adc
	if (Node2_RxHeader.DLC == 8 && Node2_RxData[1] == 0x22)
	{
		// Send response with ADC, Service id + 0x40
		Node2_TxData[1] = Node2_RxData[1] + 0x40;
		Node2_TxData[2] = (var1>>8) & 0xff; //high
		Node2_TxData[3] = var1 & 0xff; //low
		HAL_CAN_AddTxMessage(&hcan2, &Node2_TxHeader, Node2_TxData,
					 &Node2_TxMailbox);
	}

	// joystick
	if (Node2_RxHeader.StdId == 0x712 && Node2_RxData[2] == 0x2E && (Node2_RxData[0] >> 4) == 0x1 && Node2_RxData[3] == 0xF0 && Node2_RxData[4] == 0x02) //write data by Identifier and First frame
		{

			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
			printf("\n MCU received FF: %x-%x-%x-%x-%x-%x-%x-%x\r\n", Node2_RxData[0],Node2_RxData[1],Node2_RxData[2],Node2_RxData[3], Node2_RxData[4], Node2_RxData[5], Node2_RxData[6], Node2_RxData[7]);

			Node2_TxData[0] = 0x30;
			Node2_TxData[1] = 0x00;
			Node2_TxData[2] = 0x00;
			Node2_TxData[3] = 0x00;	//2E + 40
			Node2_TxData[4] = 0x00;
			Node2_TxData[5] = 0x00;
			Node2_TxData[6] = 0x00;
			Node2_TxData[7] = 0x00;
			HAL_CAN_AddTxMessage(&hcan2, &Node2_TxHeader, Node2_TxData, &Node2_TxMailbox);
		}

		if(Node2_RxHeader.StdId == 0x712 && (Node2_RxData[0] >> 4) == 0x2)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
			printf("MCU received CF: %x-%x-%x-%x-%x-%x-%x-%x\r\n\n", Node2_RxData[0],Node2_RxData[1],Node2_RxData[2],Node2_RxData[3], Node2_RxData[4], Node2_RxData[5], Node2_RxData[6], Node2_RxData[7]);
		}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance)
	{
		timer_adc = 1;
	}
	if(htim->Instance == htim3.Instance)
	{
		timer_10s = (timer_10s == 0) ? 1 : 0;
		HAL_GPIO_TogglePin(GPIOB, LED0_Pin);
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);



  // Activate the notification
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  // Activate the notification
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  Node1_TxHeader.DLC = 8;  // data length
  Node1_TxHeader.IDE = CAN_ID_STD;
  Node1_TxHeader.RTR = CAN_RTR_DATA;
  Node1_TxHeader.StdId = 0x712;  // ID

  Node1_TxData[0] = 0x02; //Type 0: Single Frame
  Node1_TxData[1] = 0x27; //Service ID
  Node1_TxData[2] = 0x01;

  Node2_TxHeader.DLC = 8;  // data length
  Node2_TxHeader.IDE = CAN_ID_STD;
  Node2_TxHeader.RTR = CAN_RTR_DATA;
  Node2_TxHeader.StdId = 0x7A2;  // ID
  caseFalse = 1;
	printf("\nSTART PROGRAMMING...\r\n\n");
	printf("Check security!\n");
	HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
	 &Node1_TxMailbox);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (is_validKey == 0 && timer_10s == 1 && is_start == 1)  // timer10s = 1 (over)
	  	  {
	  		  Node1_TxHeader.DLC = 8;  // data length
	  		  Node1_TxHeader.IDE = CAN_ID_STD;
	  		  Node1_TxHeader.RTR = CAN_RTR_DATA;
	  		  Node1_TxHeader.StdId = 0x712;  // ID

	  		  Node1_TxData[0] = 0x02; //Type 0: Single Frame
	  		  Node1_TxData[1] = 0x27; //Service ID
	  		  Node1_TxData[2] = 0x01;
	  		  caseFalse = 0;
	  		  HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
	  				  &Node1_TxMailbox);
	  		timer_10s = (timer_10s == 0) ? 1 : 0;
	  		HAL_TIM_Base_Stop_IT(&htim3);
	  	  }

	  // read adc value
	  if (is_validKey == 1 && timer_adc == 1 && timer_10s == 0 && is_start == 1)
	  {
		  Node1_TxData[0] = 0x07; //Type 0: Single Frame
		  Node1_TxData[1] = 0x22; //Service ID
		  Node2_TxData[0] = 0x07;
		  Node2_TxData[4] = 0x00;
		  Node2_TxData[5] = 0x00;
		  Node2_TxData[6] = 0x00;
		  Node2_TxData[7] = 0x00;
		  HAL_ADC_Start_IT(&hadc1);
		  HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData,
			 &Node1_TxMailbox);
		  timer_adc = 0; // flag for timer
	  }

	  // joystick
	  if ((is_validKey == 1) && ((left_button == 1) || (right_button == 1) || (middle_button == 1)) && (timer_10s == 0) && (is_start == 1))
	  	  {
	  		  //Send first frame
	  		  current_pos = 0;

	  		  Node1_TxData[0] = 0x10;
	  		  Node1_TxData[1] = 0x0A; // 0x019 : 0000 0001 1001
	  		  Node1_TxData[2] = 0x2E;

	  		  //DATA ID
	  		  Node1_TxData[3] = 0xF0;
	  		  Node1_TxData[4] = 0x02;

	  		  Node2_TxData[0] = 0x00;
	  		  Node2_TxData[1] = 0x00;
	  		  Node2_TxData[2] = 0x00;
	  		  Node2_TxData[3] = 0x00;
	  		  Node2_TxData[4] = 0x00;
	  		  Node2_TxData[5] = 0x00;
	  		  Node2_TxData[6] = 0x00;
	  		  Node2_TxData[7] = 0x00;

	  		  if(left_button == 1)
	  		  {
	  			  Node1_TxData[5] = LEFT_DATA[current_pos++];
	  		  	  Node1_TxData[6] = LEFT_DATA[current_pos++];
	  		  	  Node1_TxData[7] = LEFT_DATA[current_pos++];
	  		  }
	  		  else if(right_button == 1)
	  		  {
	  			  Node1_TxData[5] = RIGHT_DATA[current_pos++];
	  			  Node1_TxData[6] = RIGHT_DATA[current_pos++];
	  			  Node1_TxData[7] = RIGHT_DATA[current_pos++];
	  		  }
	  		  else if(middle_button == 1)
	  		  {
	  			  Node1_TxData[5] = MIDDLE_DATA[current_pos++];
	  			  Node1_TxData[6] = MIDDLE_DATA[current_pos++];
	  			  Node1_TxData[7] = MIDDLE_DATA[current_pos++];
	  		  }
	  		  data_length = (Node1_TxData[0] & 0x0f) + Node1_TxData[1];
	  		  HAL_CAN_AddTxMessage(&hcan1, &Node1_TxHeader, Node1_TxData, &Node1_TxMailbox);
	  	  }
	  	  HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18; // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x7A2 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x7A2 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20; // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10; // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  canfilterconfig.FilterIdHigh = 0x712 << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x712 << 5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0; // doesn't matter in single can controllers

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
  /* USER CODE END CAN2_Init 2 */

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 19999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 41999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC4 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 300);
  return ch;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	 if(hadc->Instance == hadc1.Instance)
	 {
		 var1 = HAL_ADC_GetValue(&hadc1);
	 }
}
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4){
		left_button = 1;
	}
	else if (GPIO_Pin == GPIO_PIN_7){
		right_button = 1;
	}
	else if (GPIO_Pin == GPIO_PIN_13){
			middle_button = 1;
		}
}
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
