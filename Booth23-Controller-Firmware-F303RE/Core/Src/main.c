/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Motor_t {
	GPIO_TypeDef *dir1_port;
	uint16_t dir1_pin;
	GPIO_TypeDef *dir2_port;
	uint16_t dir2_pin;
	TIM_HandleTypeDef *pwm_tim;
	uint16_t pwm_tim_ch;
	TIM_HandleTypeDef *encoder_tim;
} Motor_t;

typedef enum Direction_t {
	OFF = 0,
	BLOCK = 1,
	FORWARD = 2,
	BACK = 3
} Direction_t;

typedef struct PID_Controller_t {
	Motor_t* motor;
	float pos;
	float last_e;
	float target;
	float K_p; // PID Gain
	float K_d; // Derivative time
	float E;   // Encoder coefficient
	float velocity_buf[5];
	float velocity;
	uint32_t t_millis;
	uint16_t last_encoder_pos;
	uint32_t last_ticks;
	uint32_t last_t_millis;
	int u;
} PID_Controller_t;

typedef enum CommandWord_t {
	Command_Word_GOTO = 0,
	Command_Word_SET = 1,
	Command_Word_OFF = 2,
	Command_Word_TENSION = 3,
	Command_Word_BLOCK = 4
} CommandWord_t;

typedef struct Command_t {
	CommandWord_t cmd;
	float p1, p2, p3;
} Command_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PWM_MAX 65535
#define SERIAL_C 1000.0
#define MAX_DIFF 2000
#define RX_BUF_SIZE 16
#define TX_BUF_SIZE 16
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t uart_rx_buf[RX_BUF_SIZE];
volatile bool uart_ready = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void motor_init(Motor_t* motor);
void drive_motor(Motor_t* motor, Direction_t dir, uint16_t speed);
void pid_init(PID_Controller_t* pid, Motor_t* motor,
			  float K_p, float K_d, float E);
void pid_set_target(PID_Controller_t* pid, float target);
void pid_loop(PID_Controller_t* pid);
void pid_set_pos(PID_Controller_t* pid, float abs);
uint16_t encoder_read_raw(Motor_t* motor);
uint8_t parse_command(Command_t* cmd, uint8_t buf[RX_BUF_SIZE]);
int clamp(int val, int lo, int hi);

void block(Motor_t* m1, Motor_t* m2, Motor_t* m3);
void off(Motor_t* m1, Motor_t* m2, Motor_t* m3);
void update_state(PID_Controller_t* p);
void tension(PID_Controller_t* p1, PID_Controller_t* p2, PID_Controller_t* p3);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int reception_complete = 0;
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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

    Motor_t motor1 = {
    	.dir1_port = DIR11_GPIO_Port,
    	.dir1_pin = DIR11_Pin,
    	.dir2_port = DIR12_GPIO_Port,
    	.dir2_pin = DIR12_Pin,
    	.pwm_tim = &htim4,
    	.pwm_tim_ch = TIM_CHANNEL_1,
    	.encoder_tim = &htim1
    };

    Motor_t motor2 = {
    	.dir1_port = DIR21_GPIO_Port,
    	.dir1_pin = DIR21_Pin,
    	.dir2_port = DIR22_GPIO_Port,
    	.dir2_pin = DIR22_Pin,
    	.pwm_tim = &htim4,
    	.pwm_tim_ch = TIM_CHANNEL_3,
    	.encoder_tim = &htim2
    };

    Motor_t motor3 = {
    	.dir1_port = DIR31_GPIO_Port,
    	.dir1_pin = DIR31_Pin,
    	.dir2_port = DIR32_GPIO_Port,
    	.dir2_pin = DIR32_Pin,
    	.pwm_tim = &htim4,
    	.pwm_tim_ch = TIM_CHANNEL_4,
    	.encoder_tim = &htim3
    };

    //float K_p = 500000.0; When E = 0.0011...
    float K_p12 = 1000.0;
    float K_p3  = 500.0;
    //float K_d = 100000.0; When E = 0.0011
    float K_d12 = 1000.0;
    float K_d3  = 100.0;
    float E12 = 1.178; //
    float E3  = 0.4712; //check this I coudn't find posted CPR

    PID_Controller_t pid1;
    PID_Controller_t pid2;
    PID_Controller_t pid3;

    motor_init(&motor1);
    motor_init(&motor2);
    motor_init(&motor3);

    pid_init(&pid1, &motor1, K_p12, K_d12, E12);
    pid_init(&pid2, &motor2, K_p12, K_d12, E12);
    pid_init(&pid3, &motor3, K_p3, K_d3, E3);


    HAL_UART_Receive_DMA(&huart3, uart_rx_buf, RX_BUF_SIZE);
    HAL_UART_Transmit(&huart3, '0', 1, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    int millis = HAL_GetTick();
    int prev_millis = millis;

    bool continue_flag = false;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		millis = HAL_GetTick();
		if (millis - prev_millis >= 100 && !uart_ready) {
			// ready to receive every 100ms
			HAL_UART_Transmit(&huart3, '0', 1, 100);
			prev_millis = millis;
		}
		pid_loop(&pid1);
		pid_loop(&pid2);
		pid_loop(&pid3);

		if (uart_ready) {
			Command_t cmd;
			uint8_t r = parse_command(&cmd, uart_rx_buf);
			if (r == 0) {
				switch(cmd.cmd) {
				case Command_Word_GOTO:
					pid1.target = (float) cmd.p1;
					pid2.target = (float) cmd.p2;
					pid3.target = (float) cmd.p3;
					break;
				case Command_Word_SET:
					pid1.pos = (float) cmd.p1;
					pid2.pos = (float) cmd.p2;
					pid3.pos = (float) cmd.p3;

					pid1.target = pid1.pos;
					pid2.target = pid2.pos;
					pid3.target = pid3.pos;

					pid1.u = 0;
					pid2.u = 0;
					pid3.u = 0;

					pid1.velocity = 0.0;
					pid2.velocity = 0.0;
					pid3.velocity = 0.0;

					for (int i = 0; i < 5; i++) {
						pid1.velocity_buf[i] = 0.0;
						pid2.velocity_buf[i] = 0.0;
						pid3.velocity_buf[i] = 0.0;
					}
					break;
				case Command_Word_OFF:
					off(pid1.motor, pid2.motor, pid3.motor);
					uart_ready = false;
					HAL_UART_Receive_DMA(&huart3, uart_rx_buf, RX_BUF_SIZE);
					HAL_Delay(50);
					HAL_UART_Transmit(&huart3, '0', 1, 100);
					while (!uart_ready){}
					continue_flag = true;
					break;
				case Command_Word_TENSION:
					tension(&pid1, &pid2, &pid3);
					uart_ready = false;
					HAL_UART_Receive_DMA(&huart3, uart_rx_buf, RX_BUF_SIZE);
					HAL_Delay(50);
					HAL_UART_Transmit(&huart3, '0', 1, 100);
					while (!uart_ready){}
					continue_flag = true;
					break;
				case Command_Word_BLOCK:
					block(pid1.motor, pid2.motor, pid3.motor);
					uart_ready = false;
					HAL_UART_Receive_DMA(&huart3, uart_rx_buf, RX_BUF_SIZE);
					HAL_UART_Transmit(&huart3, '0', 1, 100);
					while (!uart_ready){}
					continue_flag = true;
					break;
				default:
					break;
				}
			}
			if (continue_flag) {
				continue_flag = false;
				continue;
			}
			HAL_UART_Transmit(&huart3, uart_rx_buf, sizeof(uart_rx_buf), 100);
			uart_ready = false;
		}
		HAL_Delay(5);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR31_Pin|LD2_Pin|DIR11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR32_Pin|DIR12_Pin|DIR22_Pin|DIR21_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR31_Pin LD2_Pin DIR11_Pin */
  GPIO_InitStruct.Pin = DIR31_Pin|LD2_Pin|DIR11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR32_Pin DIR12_Pin DIR22_Pin DIR21_Pin */
  GPIO_InitStruct.Pin = DIR32_Pin|DIR12_Pin|DIR22_Pin|DIR21_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void drive_motor(Motor_t* motor, Direction_t dir, uint16_t speed)
{
	switch (dir) {
		case OFF:
			HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, 0);
			HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, 0);
			break;
		case BLOCK:
			HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, 1);
			HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, 1);
			break;
		case FORWARD:
			HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, 1);
			HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, 0);
			break;
		case BACK:
			HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, 0);
			HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, 1);
			break;
	};

	__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_tim_ch, speed);
}

void motor_init(Motor_t *motor)
{
	HAL_TIM_PWM_Start(motor->pwm_tim, motor->pwm_tim_ch);
	HAL_TIM_Encoder_Start(motor->encoder_tim, TIM_CHANNEL_ALL);
}

void pid_init(PID_Controller_t* pid, Motor_t* motor,
		      float K_p, float K_d, float E)
{
	pid->motor = motor;
	pid->target = 0.0;
	pid->pos = 0.0;
	pid->last_e = 0.0;
	pid->K_p = K_p;
	pid->K_d = K_d;
	pid->E = E;
	pid->velocity_buf[0] = 0.0;
	pid->velocity_buf[1] = 0.0;
	pid->velocity_buf[2] = 0.0;
	pid->velocity_buf[3] = 0.0;
	pid->velocity_buf[4] = 0.0;
	pid->velocity = 0;
	pid->t_millis = HAL_GetTick() + 1;
	pid->last_t_millis = HAL_GetTick();
	pid->last_encoder_pos = encoder_read_raw(motor);

	pid->last_ticks = HAL_GetTick();
}

void pid_set_target(PID_Controller_t* pid, float target)
{
	pid->target = target;
}

void pid_loop(PID_Controller_t* pid)
{
	update_state(pid);
	float e = pid->pos - pid->target;
	float de = e - pid->last_e;

	pid->u = (int) (-pid->K_p * e - pid->K_d*de);

	pid->u = clamp(pid->u, -PWM_MAX, PWM_MAX);

	if (pid->u > 0) {
		drive_motor(pid->motor, FORWARD, pid->u);
	} else {
		drive_motor(pid->motor, BACK, -pid->u);
	}

	pid->last_e = e;
}

void pid_set_pos(PID_Controller_t* pid, float pos)
{
	pid->pos = pos;
}

uint16_t encoder_read_raw(Motor_t* motor)
{
	uint16_t val = (motor->encoder_tim->Instance->CNT) >> 2;
	return val;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(&huart3, uart_rx_buf, RX_BUF_SIZE);
	uart_ready = 1;
}

uint8_t parse_command(Command_t *cmd, uint8_t buf[RX_BUF_SIZE])
{
	int CMD_WORD_SIZE = 4;
	//Fixing CMD WORD TO 4, JUST ADD A SPACE TO SET: "SET "
	char cmd_word[CMD_WORD_SIZE];
	int p1_i, p2_i, p3_i;
	const float factor = 1.0; // read mm commands and set mm positions
	for (int i = 0; i < 4; i++) {
		cmd_word[i] = buf[i];
	};
	p1_i = *(int*)(&buf[CMD_WORD_SIZE]);
	p2_i = *(int*)(&buf[CMD_WORD_SIZE + 4]);
	p3_i = *(int*)(&buf[CMD_WORD_SIZE + 8]);

	if (strncmp(cmd_word, "GOTO", 4) == 0) {
		cmd->cmd = Command_Word_GOTO;
	} else if (strncmp(cmd_word, "SET ", 4) == 0) {
		cmd->cmd = Command_Word_SET;
	} else if (strncmp(cmd_word, "OFF ", 4) == 0) {
		cmd->cmd = Command_Word_OFF;
	} else if (strncmp(cmd_word, "BLCK", 4) == 0) {
		cmd->cmd = Command_Word_BLOCK;
	} else if (strncmp(cmd_word, "TNSN", 4) == 0) {
		cmd->cmd = Command_Word_TENSION;
	} else {
		return 1;
	}
	cmd->p1 = (float) p1_i * factor;
	cmd->p2 = (float) p2_i * factor;
	cmd->p3 = (float) p3_i * factor;
	return 0;
}

int clamp(int val, int lo, int hi) {
	if (val < lo) {
		return lo;
	} else if (val > hi) {
		return hi;
	} else {
		return val;
	}
}

void off(Motor_t* m1, Motor_t* m2, Motor_t* m3) {
	drive_motor(m1, OFF, 0);
	drive_motor(m2, OFF, 0);
	drive_motor(m3, OFF, 0);
}

void block(Motor_t* m1, Motor_t* m2, Motor_t* m3){
	drive_motor(m1, BLOCK, (uint16_t) ~0);
	drive_motor(m2, BLOCK, (uint16_t) ~0);
	drive_motor(m3, BLOCK, (uint16_t) ~0);
}

void update_state(PID_Controller_t* pid) {
	uint16_t raw_read = encoder_read_raw(pid->motor);
	int32_t prev = (int32_t)pid->last_encoder_pos;
	int32_t curr = (int32_t)raw_read;
	int32_t diff = curr - prev;

	if (abs(diff) > MAX_DIFF) {
		diff = 0;
	}

	pid->last_encoder_pos = raw_read;

	if (diff < -(1 << 15)) {
		// overflow in the positive direction
		diff = (1 << 16) + diff;
	} else if (diff > (1 << 15)) {
		// overflow in the positive direction
		diff = diff - (1 << 16);
	}

	uint32_t delta_t_millis = pid->t_millis - pid->last_t_millis;
	pid->last_t_millis = pid->t_millis;
	pid->t_millis = HAL_GetTick();

	float dpos = pid->E * (float) diff;
	float v = 1000.0 * dpos/(float) delta_t_millis;

	for (int i = 0; i < 4; i++) {
		pid->velocity_buf[i] = pid->velocity_buf[i+1];
	}
	pid->velocity_buf[4] = v;

	float sum = 0.0;
	for (int i = 0; i < 5; i++) {
		sum += pid->velocity_buf[i]/5.0;
	}
	pid->velocity = sum;

	pid->pos += dpos;
}

void tension(PID_Controller_t* p1, PID_Controller_t* p2, PID_Controller_t* p3) {
	float eps = 0.001; //mm/s
	drive_motor(p1->motor, BACK, (uint16_t) (~0)>>2);
	drive_motor(p2->motor, BACK, (uint16_t) (~0)>>2);
	drive_motor(p3->motor, BACK, (uint16_t) (~0)>>2);

	char MSG[50] = {'\0'};

	uint32_t t_start = HAL_GetTick();
	while (HAL_GetTick() - t_start < 1000) {
		update_state(p1);
		update_state(p2);
		update_state(p3);
		HAL_Delay(50);
	}

	while ((abs(p1->velocity) > eps) || (abs(p2->velocity) > eps) || (abs(p3->velocity) > eps)) {
		update_state(p1);
		update_state(p2);
		update_state(p3);
		HAL_Delay(20);
	}
	block(p1->motor, p2->motor, p3->motor);
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
