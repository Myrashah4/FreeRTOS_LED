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
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLTAGE_THRESHOLD_ADC 3100  // ADC value for 2.5V (assuming 3.3V reference and 12-bit ADC: 2.5/3.3*4095 ≈ 3100)
#define DEFAULT_FREQUENCY 1
#define UART_RX_BUFFER_SIZE 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
osMessageQueueId_t freqQueueHandle;
osMutexId_t freqMutexHandle;
uint16_t frequency = 1;                // Current frequency (Hz)
const uint16_t freqMax = 100;          // Increased max frequency for UART control

// UART variables
uint8_t ledMode = 0;           // 0=normal, 1=pulse, 2=fade, 3=strobe, 4=sweep
uint32_t systemUptime = 0;     // System uptime in seconds
uint16_t commandCount = 0;     // Count of processed commands
uint8_t autoMode = 0;          // 0=manual, 1=auto increment, 2=auto random

uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
uint8_t uartRxChar;
volatile uint8_t uartRxIndex = 0;
volatile uint8_t uartCommandReady = 0;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for my_LED_Blinker */
osThreadId_t my_LED_BlinkerHandle;
const osThreadAttr_t my_LED_Blinker_attributes = {
  .name = "my_LED_Blinker",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ledBlink */
osThreadId_t ledBlinkHandle;
const osThreadAttr_t ledBlink_attributes = {
  .name = "ledBlink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for ButtonMonitorTa */
osThreadId_t ButtonMonitorTaHandle;
const osThreadAttr_t ButtonMonitorTa_attributes = {
  .name = "ButtonMonitorTa",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTCommandTask */
osThreadId_t UARTCommandTaskHandle;
const osThreadAttr_t UARTCommandTask_attributes = {
  .name = "UARTCommandTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void Start_LED_TASK(void *argument);
void StartLedBlinkTask(void *argument);
void StartButtonMonitor(void *argument);
void StartUARTCommandTask(void *argument);

/* USER CODE BEGIN PFP */
void UART_SendMessage(char* message);
void ProcessUARTCommand(void);
void VisualConfirm(uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        osDelay(100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        osDelay(100);
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  freqQueueHandle = osMessageQueueNew(1, sizeof(uint16_t), NULL);
  freqMutexHandle = osMutexNew(NULL);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of my_LED_Blinker */
  my_LED_BlinkerHandle = osThreadNew(Start_LED_TASK, NULL, &my_LED_Blinker_attributes);

  /* creation of ledBlink */
 // ledBlinkHandle = osThreadNew(StartLedBlinkTask, NULL, &ledBlink_attributes);

  /* creation of ButtonMonitorTa */
  ButtonMonitorTaHandle = osThreadNew(StartButtonMonitor, NULL, &ButtonMonitorTa_attributes);

  /* creation of UARTCommandTask */
  UARTCommandTaskHandle = osThreadNew(StartUARTCommandTask, NULL, &UARTCommandTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start UART interrupt for receiving */
  /* Start UART interrupt for receiving */
  HAL_StatusTypeDef uart_status = HAL_UART_Receive_IT(&huart2, &uartRxChar, 1);

  if (uart_status != HAL_OK) {
     char error_msg[50];
     sprintf(error_msg, "UART Init Error: %d\r\n", uart_status);
     HAL_UART_Transmit(&huart2, (uint8_t*)error_msg, strlen(error_msg), 1000);
   }
  /* Send welcome message */
  UART_SendMessage("\r\n\r\n");
  UART_SendMessage("╔══════════════════════════════════════════════════════════╗\r\n");
  UART_SendMessage("║          🎛️  STM32F4xx Advanced LED Controller           ║\r\n");
  UART_SendMessage("║                    FreeRTOS Project                      ║\r\n");
  UART_SendMessage("╠══════════════════════════════════════════════════════════╣\r\n");
  UART_SendMessage("║  Features:                                               ║\r\n");
  UART_SendMessage("║  ✓ Multi-threaded Real-time Operating System            ║\r\n");
  UART_SendMessage("║  ✓ Advanced UART Command Interface                       ║\r\n");
  UART_SendMessage("║  ✓ Multiple LED Control Modes                           ║\r\n");
  UART_SendMessage("║  ✓ Automatic Frequency Control                          ║\r\n");
  UART_SendMessage("║  ✓ Real-time System Monitoring                          ║\r\n");
  UART_SendMessage("║  ✓ Interactive Demonstration Modes                      ║\r\n");
  UART_SendMessage("╠══════════════════════════════════════════════════════════╣\r\n");
  UART_SendMessage("║  Quick Start:                                            ║\r\n");
  UART_SendMessage("║   • Type 'HELP' for all commands                        ║\r\n");
  UART_SendMessage("║   • Type 'DEMO' for demonstration                       ║\r\n");
  UART_SendMessage("║   • Type 'STATUS' for system info                       ║\r\n");
  UART_SendMessage("║   • Type any number (1-100) to set frequency            ║\r\n");
  UART_SendMessage("╚══════════════════════════════════════════════════════════╝\r\n");
  UART_SendMessage("Ready> ");
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Send message via UART
  * @param message: pointer to message string
  * @retval None
  */
void UART_SendMessage(char* message) {
  HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

/**
  * @brief Process received UART command
  * @retval None
  */


void ProcessUARTCommand(void) {
  commandCount++; // Increment command counter

  // Convert to uppercase for easier parsing
  for (int i = 0; uartRxBuffer[i]; i++) {
    if (uartRxBuffer[i] >= 'a' && uartRxBuffer[i] <= 'z') {
      uartRxBuffer[i] = uartRxBuffer[i] - 'a' + 'A';
    }
  }

  // STATUS command - Show detailed system information
  if (strncmp((char*)uartRxBuffer, "STATUS", 6) == 0) {
    osMutexAcquire(freqMutexHandle, osWaitForever);
    uint16_t currentFreq = frequency;
    osMutexRelease(freqMutexHandle);

    UART_SendMessage("\r\n╔═══════════════════════════════════╗\r\n");
    UART_SendMessage("║        SYSTEM STATUS              ║\r\n");
    UART_SendMessage("╠═══════════════════════════════════╣\r\n");

    char statusLine[50];
    sprintf(statusLine, "║ Frequency: %3u Hz                ║\r\n", currentFreq);
    UART_SendMessage(statusLine);

    sprintf(statusLine, "║ LED Mode: %s",
            ledMode == 0 ? "Normal     " :
            ledMode == 1 ? "Pulse      " :
            ledMode == 2 ? "Fade       " :
            ledMode == 3 ? "Strobe     " : "Sweep      ");
    UART_SendMessage(statusLine);
    UART_SendMessage("          ║\r\n");

    sprintf(statusLine, "║ Auto Mode: %s",
            autoMode == 0 ? "Manual    " :
            autoMode == 1 ? "Increment " : "Random    ");
    UART_SendMessage(statusLine);
    UART_SendMessage("          ║\r\n");

    sprintf(statusLine, "║ Uptime: %lu seconds              ║\r\n", systemUptime);
    UART_SendMessage(statusLine);

    sprintf(statusLine, "║ Commands: %u                     ║\r\n", commandCount);
    UART_SendMessage(statusLine);

    UART_SendMessage("╚═══════════════════════════════════╝\r\nReady> ");
    return;
  }

  // MODE command - Change LED behavior

  if (strncmp((char*)uartRxBuffer, "MODE ", 5) == 0) {
      // Find the start of the mode parameter (skip spaces after MODE)
      int start = 5;
      while (uartRxBuffer[start] == ' ' && start < UART_RX_BUFFER_SIZE) start++;

      // Find the end of the mode parameter
      int end = start;
      while (uartRxBuffer[end] != '\0' && uartRxBuffer[end] != '\r' &&
             uartRxBuffer[end] != '\n' && uartRxBuffer[end] != ' ' &&
             end < UART_RX_BUFFER_SIZE) {
          end++;
      }

      // Extract mode string
      char modeStr[20] = {0};
      int len = end - start;
      if (len > 0 && len < 19) {
          strncpy(modeStr, (char*)&uartRxBuffer[start], len);
          modeStr[len] = '\0';

          // Convert to uppercase
          for (int i = 0; modeStr[i]; i++) {
              if (modeStr[i] >= 'a' && modeStr[i] <= 'z') {
                  modeStr[i] = modeStr[i] - 'a' + 'A';
              }
          }

          // Match mode strings
          if (strcmp(modeStr, "NORMAL") == 0) ledMode = 0;
          else if (strcmp(modeStr, "PULSE") == 0) ledMode = 1;
          else if (strcmp(modeStr, "FADE") == 0) ledMode = 2;
          else if (strcmp(modeStr, "STROBE") == 0) ledMode = 3;
          else if (strcmp(modeStr, "SWEEP") == 0) ledMode = 4;
          else {
              char errorMsg[100];
              sprintf(errorMsg, "ERROR: '%s' invalid. Use: NORMAL, PULSE, FADE, STROBE, SWEEP\r\nReady> ", modeStr);
              UART_SendMessage(errorMsg);
              return;
          }

          char message[50];
          sprintf(message, "LED mode set to %s\r\nReady> ", modeStr);
          UART_SendMessage(message);
      } else {
          UART_SendMessage("ERROR: Invalid mode parameter\r\nReady> ");
      }
      return;
  }


  // AUTO command - Enable automatic frequency changes
  if (strncmp((char*)uartRxBuffer, "AUTO ", 5) == 0) {
      // Find the start of the auto parameter (skip spaces after AUTO)
      int start = 5;
      while (uartRxBuffer[start] == ' ' && start < UART_RX_BUFFER_SIZE) start++;

      // Find the end of the auto parameter
      int end = start;
      while (uartRxBuffer[end] != '\0' && uartRxBuffer[end] != '\r' &&
             uartRxBuffer[end] != '\n' && uartRxBuffer[end] != ' ' &&
             end < UART_RX_BUFFER_SIZE) {
          end++;
      }

      // Extract auto string
      char autoStr[20] = {0};
      int len = end - start;
      if (len > 0 && len < 19) {
          strncpy(autoStr, (char*)&uartRxBuffer[start], len);
          autoStr[len] = '\0';

          // Convert to uppercase
          for (int i = 0; autoStr[i]; i++) {
              if (autoStr[i] >= 'a' && autoStr[i] <= 'z') {
                  autoStr[i] = autoStr[i] - 'a' + 'A';
              }
          }

          // Match auto modes
          if (strcmp(autoStr, "OFF") == 0) autoMode = 0;
          else if (strcmp(autoStr, "INC") == 0) autoMode = 1;
          else if (strcmp(autoStr, "RANDOM") == 0) autoMode = 2;
          else {
              char errorMsg[80];
              sprintf(errorMsg, "ERROR: '%s' invalid. Use: OFF, INC, RANDOM\r\nReady> ", autoStr);
              UART_SendMessage(errorMsg);
              return;
          }

          char message[50];
          sprintf(message, "Auto mode set to %s\r\nReady> ", autoStr);
          UART_SendMessage(message);
      } else {
          UART_SendMessage("ERROR: Invalid auto parameter\r\nReady> ");
      }
      return;
  }

  // SWEEP command - Frequency sweep
  if (strncmp((char*)uartRxBuffer, "SWEEP", 5) == 0) {
    UART_SendMessage("Starting frequency sweep (1-20 Hz)...\r\n");

    for (uint16_t f = 1; f <= 20; f++) {
      osMutexAcquire(freqMutexHandle, osWaitForever);
      frequency = f;
      osMutexRelease(freqMutexHandle);

      osMessageQueuePut(freqQueueHandle, &f, 0, 0);

      char sweepMsg[30];
      sprintf(sweepMsg, "Frequency: %u Hz\r\n", f);
      UART_SendMessage(sweepMsg);

      osDelay(500); // 0.5 second per step
      VisualConfirm(1);
    }

    UART_SendMessage("Sweep completed!\r\nReady> ");
    return;
  }

  // DEMO command - Run a demonstration sequence
  if (strncmp((char*)uartRxBuffer, "DEMO", 4) == 0) {
    UART_SendMessage("\r\n🌟 STARTING DEMONSTRATION SEQUENCE 🌟\r\n");
    UART_SendMessage("Phase 1: Slow blink (1 Hz)\r\n");

    uint16_t demo_freq = 1;
    osMutexAcquire(freqMutexHandle, osWaitForever);
    frequency = demo_freq;
    osMutexRelease(freqMutexHandle);
    osMessageQueuePut(freqQueueHandle, &demo_freq, 0, 0);
    osDelay(3000);
    VisualConfirm(3);

    UART_SendMessage("Phase 2: Medium blink (5 Hz)\r\n");
    demo_freq = 5;
    osMutexAcquire(freqMutexHandle, osWaitForever);
    frequency = demo_freq;
    osMutexRelease(freqMutexHandle);
    osMessageQueuePut(freqQueueHandle, &demo_freq, 0, 0);
    osDelay(3000);
    VisualConfirm(3);

    UART_SendMessage("Phase 3: Fast blink (20 Hz)\r\n");
    demo_freq = 20;
    osMutexAcquire(freqMutexHandle, osWaitForever);
    frequency = demo_freq;
    osMutexRelease(freqMutexHandle);
    osMessageQueuePut(freqQueueHandle, &demo_freq, 0, 0);
    osDelay(3000);
    VisualConfirm(3);

    UART_SendMessage("🎉 DEMONSTRATION COMPLETE! 🎉\r\nReady> ");
    return;
  }

  // RESET command - System reset
  if (strncmp((char*)uartRxBuffer, "RESET", 5) == 0) {
    UART_SendMessage("⚠️  SYSTEM RESET - Returning to defaults...\r\n");

    osMutexAcquire(freqMutexHandle, osWaitForever);
    frequency = DEFAULT_FREQUENCY;
    osMutexRelease(freqMutexHandle);

    ledMode = 0;
    autoMode = 0;
    commandCount = 0;

    uint16_t resetFreq = DEFAULT_FREQUENCY;
    osMessageQueuePut(freqQueueHandle, &resetFreq, 0, 0);

    UART_SendMessage("✅ System reset complete!\r\nReady> ");
    return;
  }

  // ABOUT command - Show system information
  if (strncmp((char*)uartRxBuffer, "ABOUT", 5) == 0) {
    UART_SendMessage("\r\n┌─────────────────────────────────────┐\r\n");
    UART_SendMessage("│     STM32F4xx LED Controller       │\r\n");
    UART_SendMessage("│         FreeRTOS Project            │\r\n");
    UART_SendMessage("├─────────────────────────────────────┤\r\n");
    UART_SendMessage("│ Features:                           │\r\n");
    UART_SendMessage("│ ✓ Multi-threaded RTOS              │\r\n");
    UART_SendMessage("│ ✓ UART Command Interface            │\r\n");
    UART_SendMessage("│ ✓ Button & ADC Control             │\r\n");
    UART_SendMessage("│ ✓ Multiple LED Modes               │\r\n");
    UART_SendMessage("│ ✓ Auto Frequency Control           │\r\n");
    UART_SendMessage("│ ✓ Real-time Status Monitoring      │\r\n");
    UART_SendMessage("└─────────────────────────────────────┘\r\nReady> ");
    return;
  }

  // Enhanced HELP command
  if (strncmp((char*)uartRxBuffer, "HELP", 4) == 0) {
    UART_SendMessage("\r\n╔═══════════════════════════════════════════════════════╗\r\n");
    UART_SendMessage("║                 🎛️  COMMAND REFERENCE                  ║\r\n");
    UART_SendMessage("╠═══════════════════════════════════════════════════════╣\r\n");
    UART_SendMessage("║ BASIC COMMANDS:                                       ║\r\n");
    UART_SendMessage("║  <number>     - Set frequency (1-100 Hz)             ║\r\n");
    UART_SendMessage("║  SET <freq>   - Set frequency                         ║\r\n");
    UART_SendMessage("║  GET          - Show current frequency                ║\r\n");
    UART_SendMessage("║                                                       ║\r\n");
    UART_SendMessage("║ ADVANCED COMMANDS:                                    ║\r\n");
    UART_SendMessage("║  STATUS       - Show detailed system status          ║\r\n");
    UART_SendMessage("║  MODE <type>  - Change LED mode                       ║\r\n");
    UART_SendMessage("║                 (NORMAL/PULSE/FADE/STROBE/SWEEP)      ║\r\n");
    UART_SendMessage("║  AUTO <mode>  - Auto frequency (OFF/INC/RANDOM)      ║\r\n");
    UART_SendMessage("║  SWEEP        - Frequency sweep demonstration         ║\r\n");
    UART_SendMessage("║  DEMO         - Run full demonstration                ║\r\n");
    UART_SendMessage("║  RESET        - Reset system to defaults             ║\r\n");
    UART_SendMessage("║  ABOUT        - System information                    ║\r\n");
    UART_SendMessage("╚═══════════════════════════════════════════════════════╝\r\nReady> ");
    return;
  }

  // GET command (existing)
  if (strncmp((char*)uartRxBuffer, "GET", 3) == 0) {
    osMutexAcquire(freqMutexHandle, osWaitForever);
    uint16_t currentFreq = frequency;
    osMutexRelease(freqMutexHandle);

    char message[50];
    sprintf(message, "📊 Current frequency: %u Hz\r\nReady> ", currentFreq);
    UART_SendMessage(message);
    return;
  }

  // SET command (existing but enhanced)
  if (strncmp((char*)uartRxBuffer, "SET ", 4) == 0) {
    uint16_t newFreq = atoi((char*)&uartRxBuffer[4]);

    if (newFreq >= 1 && newFreq <= freqMax) {
      osMutexAcquire(freqMutexHandle, osWaitForever);
      frequency = newFreq;
      osMutexRelease(freqMutexHandle);

      osMessageQueuePut(freqQueueHandle, &newFreq, 0, 0);

      char message[50];
      sprintf(message, "✅ Frequency set to %u Hz\r\nReady> ", newFreq);
      UART_SendMessage(message);
    } else {
      char message[60];
      sprintf(message, "❌ ERROR: Invalid frequency. Range 1 to %u Hz\r\nReady> ", freqMax);
      UART_SendMessage(message);
    }
    return;
  }

  // Try to parse as direct number (existing functionality but enhanced)
  uint16_t newFreq = atoi((char*)uartRxBuffer);

  if (newFreq >= 1 && newFreq <= freqMax) {
    osMutexAcquire(freqMutexHandle, osWaitForever);
    frequency = newFreq;
    osMutexRelease(freqMutexHandle);

    osMessageQueuePut(freqQueueHandle, &newFreq, 0, 0);

    char message[50];
    sprintf(message, "✅ Frequency set to %u Hz\r\nReady> ", newFreq);
    UART_SendMessage(message);
  } else if (strlen((char*)uartRxBuffer) > 0) {
    UART_SendMessage("❌ Unknown command. Type HELP for available commands\r\nReady> ");
  } else {
    UART_SendMessage("Ready> ");
  }
}
/**
  * @brief UART Receive Complete Callback
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {

    if (uartRxChar == '\r' || uartRxChar == '\n') {
      if (uartRxIndex > 0) {
        uartRxBuffer[uartRxIndex] = '\0';

        // Process command immediately in interrupt
        ProcessUARTCommand();

        // Reset for next command
        uartRxIndex = 0;
      } else {
        UART_SendMessage("\r\nReady> ");
      }
    }
    else if (uartRxChar == '\b' || uartRxChar == 127) { // Backspace
      if (uartRxIndex > 0) {
        uartRxIndex--;
        UART_SendMessage("\b \b");
      }
    }
    else if (uartRxChar >= 32 && uartRxChar <= 126 && uartRxIndex < UART_RX_BUFFER_SIZE - 1) {
      // Store printable characters and echo them
      uartRxBuffer[uartRxIndex++] = uartRxChar;
      HAL_UART_Transmit(&huart2, &uartRxChar, 1, 10); // Echo character
    }

    // Continue receiving next character
    HAL_UART_Receive_IT(&huart2, &uartRxChar, 1);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_LED_TASK */
/**
* @brief Function implementing the my_LED_Blinker thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LED_TASK */
void Start_LED_TASK(void *argument) {
  uint16_t freqLocal = DEFAULT_FREQUENCY;
  uint32_t pulseCounter = 0;
  uint32_t fadeLevel = 0;
  uint8_t fadeDirection = 1; // 1 = up, 0 = down
  uint32_t strobeCounter = 0;
  uint32_t sweepCounter = 0;

  UART_SendMessage("DEBUG: Enhanced PA5 LED task started\r\n");

  for (;;) {
    // Check for frequency updates
    uint16_t newFreq;
    if (osMessageQueueGet(freqQueueHandle, &newFreq, NULL, 0) == osOK) {
      osMutexAcquire(freqMutexHandle, osWaitForever);
      freqLocal = newFreq;
      osMutexRelease(freqMutexHandle);

      char debug[60];
      sprintf(debug, "DEBUG: PA5 LED frequency updated to %u Hz\r\n", freqLocal);
      UART_SendMessage(debug);
    }

    // Handle different LED modes
    switch (ledMode) {
      case 0: // NORMAL - Simple blink
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        osDelay(1000 / (2 * freqLocal));
        break;

      case 1: // PULSE - Gradual on/off
        pulseCounter++;
        if (pulseCounter < 10) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        } else if (pulseCounter < 20) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        } else {
          pulseCounter = 0;
        }
        osDelay(1000 / (20 * freqLocal)); // Faster internal timing
        break;

      case 2: // FADE - Simulated PWM fade (using rapid on/off)
        fadeLevel += fadeDirection ? 1 : -1;
        if (fadeLevel >= 20) fadeDirection = 0;
        if (fadeLevel <= 0) fadeDirection = 1;

        // Simple PWM simulation
        for (int i = 0; i < 20; i++) {
          if (i < fadeLevel) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
          }
          osDelay(1); // Very short delay for PWM effect
        }
        osDelay(1000 / (10 * freqLocal));
        break;

      case 3: // STROBE - Fast flashes
        strobeCounter++;
        if (strobeCounter <= 2) {
          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
          osDelay(50); // Very fast strobe
        } else if (strobeCounter <= 10) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
          osDelay(100);
        } else {
          strobeCounter = 0;
        }
        break;

      case 4: // SWEEP - Frequency sweep pattern
        sweepCounter++;
        uint16_t sweepFreq = (sweepCounter % 10) + 1; // 1-10 Hz sweep
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        osDelay(1000 / (2 * sweepFreq));
        break;

      default:
        // Fallback to normal mode
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        osDelay(1000 / (2 * freqLocal));
        break;
    }
  }
}

/* USER CODE BEGIN Header_StartLedBlinkTask */
/**
* @brief Function implementing the ledBlink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedBlinkTask */
void StartLedBlinkTask(void *argument) {
  for (;;) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);  // Toggle LED
    osDelay(1000 / (2 * frequency));        // Delay depends on frequency
  }
}

/* USER CODE BEGIN Header_StartButtonMonitor */
/**
* @brief Function implementing the ButtonMonitorTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonMonitor */
void StartButtonMonitor(void *argument) {
  uint32_t adcValue;

  for (;;) {
    // Voltage reset check (existing code)
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
      adcValue = HAL_ADC_GetValue(&hadc1);

      if (adcValue >= VOLTAGE_THRESHOLD_ADC) {
        osMutexAcquire(freqMutexHandle, osWaitForever);
        frequency = DEFAULT_FREQUENCY;
        uint16_t resetFreq = frequency;
        osMutexRelease(freqMutexHandle);

        // Send reset frequency to LED task
        osMessageQueuePut(freqQueueHandle, &resetFreq, 0, 0);

        char message[50];
        sprintf(message, "RESET: Frequency reset to %u Hz\r\n", resetFreq);
        UART_SendMessage(message);
        osDelay(100);
      }
    }
    HAL_ADC_Stop(&hadc1);

    // Button press check
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
      osDelay(50);  // debounce
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
        osMutexAcquire(freqMutexHandle, osWaitForever);

        if (frequency < freqMax) {
          frequency++;
        } else {
          frequency = 1;
        }

        uint16_t newFreq = frequency;
        osMutexRelease(freqMutexHandle);

        // Send new frequency to LED task
        osMessageQueuePut(freqQueueHandle, &newFreq, 0, 0);

        char message[50];
        sprintf(message, "BUTTON: Frequency changed to %u Hz\r\n", newFreq);
        UART_SendMessage(message);

        // Wait for button release
        while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
          osDelay(10);
        }
      }
    }
    osDelay(10);
  }
}

/* USER CODE BEGIN Header_StartUARTCommandTask */
/**
* @brief Function implementing the UARTCommandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTCommandTask */


void StartUARTCommandTask(void *argument) {
  UART_SendMessage("🚀 Enhanced UART Command System started\r\n");

  uint32_t autoCounter = 0;

  for (;;) {
    systemUptime++; // Increment system uptime

    // Handle auto mode functionality
    if (autoMode == 1) { // Auto increment mode
      autoCounter++;
      if (autoCounter >= 5) { // Change frequency every 5 seconds
        autoCounter = 0;

        osMutexAcquire(freqMutexHandle, osWaitForever);
        frequency++;
        if (frequency > freqMax) frequency = 1;
        uint16_t newFreq = frequency;
        osMutexRelease(freqMutexHandle);

        osMessageQueuePut(freqQueueHandle, &newFreq, 0, 0);

        char message[50];
        sprintf(message, "🔄 Auto: Frequency changed to %u Hz\r\n", newFreq);
        UART_SendMessage(message);
      }
    }
    else if (autoMode == 2) { // Auto random mode
      autoCounter++;
      if (autoCounter >= 3) { // Change frequency every 3 seconds
        autoCounter = 0;

        // Generate pseudo-random frequency (1-20 Hz for demo)
        uint16_t randomFreq = (systemUptime % 20) + 1;

        osMutexAcquire(freqMutexHandle, osWaitForever);
        frequency = randomFreq;
        osMutexRelease(freqMutexHandle);

        osMessageQueuePut(freqQueueHandle, &randomFreq, 0, 0);

        char message[50];
        sprintf(message, "🎲 Random: Frequency changed to %u Hz\r\n", randomFreq);
        UART_SendMessage(message);
      }
    }

    // Optional: Send periodic heartbeat (every 60 seconds)
    if (systemUptime % 60 == 0 && systemUptime > 0) {
      char heartbeat[60];
      sprintf(heartbeat, "💓 System heartbeat - Uptime: %lu seconds\r\n", systemUptime);
      UART_SendMessage(heartbeat);
    }

    osDelay(1000); // 1 second delay
  }
}/**
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
  * @param  file: pointer to source file name
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




// [User Types Character]
//         ↓
// [HAL_UART_RxCpltCallback() Triggered]
//         ↓
// [Store Character in uartRxBuffer]
//         ↓
// [Echo Character Back to Terminal]
//         ↓
// [Wait for Next Character]
//         ↓
// ┌─────────────────────────────────────┐
// │  [Is Enter Key Pressed? (\r or \n)] │
// └─────────────────────────────────────┘
//         ↓ Yes
// [Null-Terminate Buffer (uartRxBuffer[uartRxIndex] = '\0')]
//         ↓
// [Call ProcessUARTCommand()]
//         ↓
// [Reset uartRxIndex = 0]
//         ↓
// ┌───────────────────────────────────────────────┐
// │         Inside ProcessUARTCommand()           │
// ├───────────────────────────────────────────────┤
// │ Convert input to uppercase                    │
// │ Match with known commands via strncmp()       │
// │ Parse parameters (e.g., SET 50, MODE PULSE)   │
// │ Execute corresponding action                  │
// │ Send response over UART                       │
// │ Increment command counter                     │
// └───────────────────────────────────────────────┘
//         ↓
// [Command Affects System State?]
//         ↓ Yes
// ┌───────────────────────────────────────┐
// │ Frequency change?                     │
// │   → Send to LED thread (message queue)│
// │ Mode change?                          │
// │   → Update global ledMode variable    │
// │ Status request?                       │
// │   → Read system state + send response │
// └───────────────────────────────────────┘


