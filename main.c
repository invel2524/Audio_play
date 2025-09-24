/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wm8994.h"
#include "AUDIO.h"
#include "AUDIO_LINK.h"
#include "File_Handling.h"
#include "waveplayer.h"
#include "math.h"
#include "stdlib.h"
#include "stddef.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BUFFER_SIZE 512
#define WM8994_I2C_ADDR_7BIT  0x1A
WM8994_Object_t wm8994;
#define FREQUENCY 1000.0f   // 1kHz tone
#define SAMPLE_RATE 48000.0f     // Not directly used, but for reference
#define AMPLITUDE 32767.0f
#define M_PI 3.14159265358979323846

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_i2s3_ext_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int16_t audio_buffer[AUDIO_BUFFER_SIZE];
//AUDIO_PLAYBACK_StateTypeDef AudioState = AUDIO_STATE_STOP;
char msg[50];
char data[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void generate_sine_wave(int16_t *buffer, uint32_t buffer_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern ApplicationTypeDef Appli_state;
extern AUDIO_PLAYBACK_StateTypeDef AudioState;

int IsFinished = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_1)
	{
//		AudioState = AUDIO_STATE_NEXT;
		if (AudioState == AUDIO_STATE_PLAY)
		{
			AudioState = AUDIO_STATE_PAUSE;
		}

		if (AudioState == AUDIO_STATE_WAIT)
		{
			AudioState = AUDIO_STATE_RESUME;
		}
	}
}
static int32_t Codec_I2C_Write(uint16_t DevAddr, uint16_t Reg,
                               uint8_t *pData, uint16_t Length)
{
    return HAL_I2C_Mem_Write(&hi2c1, DevAddr, Reg,
                             I2C_MEMADD_SIZE_16BIT, pData, Length, 1000);
}
static int32_t Codec_I2C_Read(uint16_t DevAddr, uint16_t Reg,
                              uint8_t *pData, uint16_t Length)
{
    return HAL_I2C_Mem_Read(&hi2c1, DevAddr, Reg,
                            I2C_MEMADD_SIZE_16BIT, pData, Length, 1000);
}
static int32_t Codec_GetTick(void) { return (int32_t)HAL_GetTick(); }

static int32_t Codec_IO_Init(void)
{
    return WM8994_OK;
}
static int32_t Codec_IO_DeInit(void)
{
    return WM8994_OK;
}
void generate_sine_wave(int16_t *buffer, uint32_t buffer_size) {
    static float sample_idx = 0.0f;
    float phase_increment = 2.0f * M_PI * FREQUENCY / SAMPLE_RATE;  // Frequency to phase increment

    for (uint32_t i = 0; i < buffer_size; i+=2) {
        // Generate sine wave sample
        int16_t sample = (int16_t)(AMPLITUDE * sinf(phase_increment * sample_idx));
        buffer[i]     = sample;  // Left
        buffer[i + 1] = sample;
        sample_idx += 1.0f;

        // Handle overflow of sample index
        if (sample_idx >= SAMPLE_RATE) {
            sample_idx -= SAMPLE_RATE;
        }
    }
}
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    // Regenerate the second half of the audio buffer
    generate_sine_wave(&audio_buffer[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE / 2);
    const char *s = "HalfCplt\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 100);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    // Regenerate the first half of the audio buffer
    generate_sine_wave(&audio_buffer[0], AUDIO_BUFFER_SIZE / 2);
    const char *s = "Cplt\r\n";

    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 100);

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //WM8994_Init(&wm8994);
  if (HAL_I2C_IsDeviceReady(&hi2c1, (WM8994_I2C_ADDR_7BIT << 1), 3, 1000) != HAL_OK) {
        const char *msg="Codec not ready\r\n";
        HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),100);
    }
  WM8994_IO_t io = {
        .Init     = Codec_IO_Init,
        .DeInit   = Codec_IO_DeInit,
        .Address  = (uint16_t)(WM8994_I2C_ADDR_7BIT << 1), // HAL expects 8-bit addr
        .WriteReg = Codec_I2C_Write,
        .ReadReg  = Codec_I2C_Read,
        .GetTick  = Codec_GetTick
    };
    if (WM8994_RegisterBusIO(&wm8994, &io) != WM8994_OK) {
        const char *msg = "BusIO reg failed\r\n";
        HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),100);
        Error_Handler();
    }
    WM8994_Init_t cfg = {
          .InputDevice  = WM8994_IN_NONE,
          .OutputDevice = WM8994_OUT_HEADPHONE,
          .Frequency    = WM8994_FREQUENCY_48K,
          .Resolution   = WM8994_RESOLUTION_16b,
          .Volume       = 90
      };
  if (WM8994_Init(&wm8994, &cfg) == WM8994_OK) {
       const char *ok = "WM8994 init OK\r\n";
       HAL_UART_Transmit(&huart2,(uint8_t*)ok,strlen(ok),100);
   } else {
     const char *er = "WM8994 init ERROR\r\n";
     HAL_UART_Transmit(&huart2,(uint8_t*)er,strlen(er),100);
  }
  uint32_t id;
  if (WM8994_ReadID(&wm8994, &id) == WM8994_OK) {
       sprintf(msg, "WM8994 ID: 0x%04lX\r\n", id);
       HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
       } else {
         const char *er = "ReadID failed\r\n";
         HAL_UART_Transmit(&huart2,(uint8_t*)er,strlen(er),100);
   }
//  if (AUDIO_OUT_Init(WM8994_OUT_HEADPHONE, 70, 48000) != 0) {
//      // Handle initialization error
//      Error_Handler();
//  }
  int32_t ret = WM8994_SetOutputMode(&wm8994, WM8994_OUT_HEADPHONE);
  if(ret != WM8994_OK) {
      // Error handling
  }
  //AUDIO_OUT_Play((uint16_t*)audio_buffer, AUDIO_BUFFER_SIZE);
  generate_sine_wave(audio_buffer, AUDIO_BUFFER_SIZE);
  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)audio_buffer, AUDIO_BUFFER_SIZE);

//  for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
//      sprintf(msg, "%d\r\n", audio_buffer[i]);
//      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
//  }
  //WM8994_SetVolume(&wm8994, 70);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
	  HAL_Delay(1000);
//	  if(Appli_state == APPLICATION_READY)
//	  {
//		  Mount_USB();
//		  AUDIO_PLAYER_Start(0);
//		  while(!IsFinished)
//		  {
//			  AUDIO_PLAYER_Process(TRUE);
//			  if(AudioState == AUDIO_STATE_STOP)
//			  {
//				  IsFinished = 1;
//			  }
//		  }
//	  }


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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};


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
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
