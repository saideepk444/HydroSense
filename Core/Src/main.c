/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - Tap-triggered 3-second audio recording
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//==water levels===
// #define CMD_LEVEL_QUARTER   0x21  // red
// #define CMD_LEVEL_HALF      0x22  // yellow
// #define CMD_LEVEL_THREEQ    0x23  // green
//==water levels=== (match these to Python CMD_LED_* bytes)
#define CMD_LEVEL_EMPTY 0x20
#define CMD_LEVEL_100ML 0x21
#define CMD_LEVEL_300ML 0x22
#define CMD_LEVEL_500ML 0x23
#define CMD_LEVEL_700ML 0x24
#define CMD_LEVEL_900ML 0x25
// ===== Audio DMA Configuration =====
#define PCM_BLOCK_SAMPLES 1024                    // per half-buffer
#define DMA_TOTAL_SAMPLES (PCM_BLOCK_SAMPLES * 2) // double buffer

// ===== Recording Configuration =====
#define SAMPLE_RATE 44117 // 48MHz / 17 / 64
#define RECORD_DURATION_SEC 3
#define SAMPLES_TO_RECORD (SAMPLE_RATE * RECORD_DURATION_SEC) // ~132,351

// ===== Protocol Markers =====
#define SYNC_HEADER_0 0xAA
#define SYNC_HEADER_1 0x55
#define START_MARKER_0 0xBB
#define START_MARKER_1 0x66
#define STOP_MARKER_0 0xCC
#define STOP_MARKER_1 0x77

// ===== IMU Configuration =====
#define LSM6DSO32_I2C_ADDR (0x6A << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// ===== WS2812 CONFIG (TIM2 @ 48 MHz, PSC=0, ARR=59) =====
#define LED_COUNT 15
#define LED_BITS 24
#define RESET_SLOTS 80
#define BUF_LEN (LED_COUNT * LED_BITS + RESET_SLOTS)

// Timing: 48 MHz → 1 tick ≈ 0.02083 us
// 60 ticks → 1.25 us total bit time
// "0" ≈ 0.4 us  → 0.4 / 1.25 * 60 ≈ 19
// "1" ≈ 0.8 us  → 0.8 / 1.25 * 60 ≈ 38
#define WS_T0H 19
#define WS_T1H 38

static uint32_t ws_buf[BUF_LEN];
volatile uint8_t ws_busy = 0;

// === water level ===
// typedef enum {
//  LED_STATE_IDLE = 0,
//  LED_STATE_RECORDING,     // blink orange
//  LED_STATE_LEVEL_QUARTER, // solid red (quarter full)
//  LED_STATE_LEVEL_HALF,    // solid yellow (half full)
//  LED_STATE_LEVEL_THREEQ   // solid green (3/4 full)
//} LedState_t;
typedef enum
{
  LED_STATE_IDLE = 0,
  LED_STATE_RECORDING, // blink orange

  LED_STATE_EMPTY, // 0 mL
  LED_STATE_100ML,
  LED_STATE_300ML,
  LED_STATE_500ML,
  LED_STATE_700ML,
  LED_STATE_900ML
} LedState_t;

volatile LedState_t g_led_state = LED_STATE_IDLE;
static uint32_t g_led_last_toggle_ms = 0;
static uint8_t g_led_on = 0;

// ===== Audio DMA buffers =====
static int32_t dfsdm_dma_buf[DMA_TOTAL_SAMPLES];

// Double buffer for PCM output
static int16_t pcm_buffer_A[PCM_BLOCK_SAMPLES];
static int16_t pcm_buffer_B[PCM_BLOCK_SAMPLES];

// Flags to signal main loop that buffer is ready
volatile uint8_t g_buffer_A_ready = 0;
volatile uint8_t g_buffer_B_ready = 0;

// UART DMA TX busy flag
volatile uint8_t g_uart_tx_busy = 0;

// ===== Recording state =====
volatile uint8_t g_recording = 0;         // 0 = idle, 1 = recording
volatile uint32_t g_samples_sent = 0;     // Counter for recording duration
volatile uint8_t g_waiting_to_record = 0; // 1 = waiting for delay before recording
volatile uint32_t g_tap_time = 0;         // Timestamp of tap

#define PRE_RECORD_DELAY_MS 1000
// ===== Cooldown after recording =====
volatile uint8_t g_cooldown_active = 0;
volatile uint32_t g_cooldown_start = 0; // 0.5 second delay before recording
#define POST_RECORD_COOLDOWN_MS 100     // 2 second cooldown

// ===== IMU flags =====
volatile uint8_t g_single_tap_flag = 0;

// ===== USART3 RX =====
uint8_t usart3_rx_byte;
volatile uint8_t usart3_received_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void LSM6DSO32_Init_SingleTap(I2C_HandleTypeDef *hi2c);
void LSM6DSO32_Debug_ReadStatus(I2C_HandleTypeDef *hi2c);
void Send_PCM_Via_UART_DMA(int16_t *pData, uint16_t numSamples);
void Send_Start_Marker(void);
void Send_Stop_Marker(void);
static void LED_Service(void);

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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_DFSDM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, &usart3_rx_byte, 1);

  printf("\r\n========================================\r\n");
  printf("  Tap-Triggered Audio Recorder\r\n");
  printf("========================================\r\n");

  // Initialize IMU for single tap detection
  LSM6DSO32_Init_SingleTap(&hi2c1);
  printf("IMU Initialized\r\n");
  LSM6DSO32_Debug_ReadStatus(&hi2c1);

  // Display configuration
  printf("\r\n--- Configuration ---\r\n");
  printf("Sample Rate: %d Hz\r\n", SAMPLE_RATE);
  printf("Record Duration: %d seconds\r\n", RECORD_DURATION_SEC);
  printf("Samples per recording: %d\r\n", SAMPLES_TO_RECORD);
  printf("Block Size: %d samples\r\n", PCM_BLOCK_SAMPLES);
  printf("---------------------\r\n\r\n");

  // Start DFSDM DMA (runs continuously, but we only send when recording)
  printf("Starting DFSDM DMA...\r\n");
  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0,
                                       (int32_t *)dfsdm_dma_buf,
                                       DMA_TOTAL_SAMPLES) != HAL_OK)
  {
    printf("ERROR: DFSDM DMA start FAILED!\r\n");
    Error_Handler();
  }
  printf("DFSDM DMA started.\r\n");
  printf("\r\n>>> TAP TO START RECORDING <<<\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_status_time = 0;
  while (1)
  {

    // === LED ===
    LED_Service();
    //  // ==== SIMPLE TEST: TOGGLE LED ON/OFF EVERY 1s ====
    //	  // (Comment this block out once you're done testing.)
    //	  static uint32_t test_last_toggle = 0;
    //	  static uint8_t  test_led_on = 0;
    //
    //	  if (HAL_GetTick() - test_last_toggle >= 1000)  // every 1000 ms
    //	  {
    //		test_last_toggle = HAL_GetTick();
    //		test_led_on = !test_led_on;
    //
    //		if (test_led_on)
    //		{
    //		  // Turn LEDs ON with some color, e.g. yellow (half level)
    //		  g_led_state = LED_STATE_LEVEL_HALF;   // solid yellow
    //		}
    //		else
    //		{
    //		  // Turn LEDs OFF
    //		  g_led_state = LED_STATE_IDLE;         // LED_Service() will call LED_AllOff()
    //		}
    //	  }

    // ========== Handle Cooldown Expiry ==========
    if (g_cooldown_active && (HAL_GetTick() - g_cooldown_start >= POST_RECORD_COOLDOWN_MS))
    {
      g_cooldown_active = 0;
      printf("Cooldown ended - ready for next tap\r\n");
    }
    // ========== Handle Tap Detection ==========
    if (g_single_tap_flag)
    {
      g_single_tap_flag = 0;

      //      // Only start if not already recording or waiting (ignore taps during recording)
      //      if (!g_recording && !g_waiting_to_record)
      //      {
      //        printf("\r\n*** TAP DETECTED - Recording in 0.5 sec ***\r\n");
      //
      //        // Start delay timer
      //        g_tap_time = HAL_GetTick();
      //        g_waiting_to_record = 1;
      //      }
      //      else
      //      {
      //        printf("(tap ignored - busy)\r\n");
      //      }
      // Only start if not recording, waiting, OR in cooldown
      if (!g_recording && !g_waiting_to_record && !g_cooldown_active) // <-- ADD !g_cooldown_active
      {
        printf("\r\n*** TAP DETECTED - Recording in 0.5 sec ***\r\n");
        g_tap_time = HAL_GetTick();
        g_waiting_to_record = 1;
      }
      else
      {
        printf("(tap ignored - busy or cooldown)\r\n");
      }
    }

    // ========== Handle Pre-Record Delay ==========
    if (g_waiting_to_record)
    {
      if (HAL_GetTick() - g_tap_time >= PRE_RECORD_DELAY_MS)
      {
        printf("*** Starting %d sec recording ***\r\n", RECORD_DURATION_SEC);

        // Send START marker to laptop
        Send_Start_Marker();

        // Reset counter and start recording
        g_samples_sent = 0;
        g_recording = 1;
        g_waiting_to_record = 0;

        //  start blinking orange while recording
        g_led_state = LED_STATE_RECORDING;
        g_led_last_toggle_ms = HAL_GetTick();
        g_led_on = 0;

        // Clear any stale buffer flags
        g_buffer_A_ready = 0;
        g_buffer_B_ready = 0;
      }
    }

    // ========== Recording State Machine ==========
    if (g_recording)
    {
      // Send buffer A if ready
      if (g_buffer_A_ready && !g_uart_tx_busy)
      {
        Send_PCM_Via_UART_DMA(pcm_buffer_A, PCM_BLOCK_SAMPLES);
        g_buffer_A_ready = 0;
        g_samples_sent += PCM_BLOCK_SAMPLES;
      }

      // Send buffer B if ready
      if (g_buffer_B_ready && !g_uart_tx_busy)
      {
        Send_PCM_Via_UART_DMA(pcm_buffer_B, PCM_BLOCK_SAMPLES);
        g_buffer_B_ready = 0;
        g_samples_sent += PCM_BLOCK_SAMPLES;
      }

      // Check if recording is complete
      if (g_samples_sent >= SAMPLES_TO_RECORD)
      {
        // Wait for last DMA transfer to complete
        while (g_uart_tx_busy)
        {
        }

        // Send STOP marker
        Send_Stop_Marker();

        // Back to idle
        g_recording = 0;

        g_cooldown_active = 1;
        g_cooldown_start = HAL_GetTick();
        //        g_led_state = LED_STATE_IDLE;
        //         g_led_on = 0;
        printf("*** Recording complete! Sent %lu samples ***\r\n", g_samples_sent);
        printf("\r\n>>> TAP TO START ANOTHER RECORDING <<<\r\n\r\n");
      }
    }
    else
    {
      // Not recording - clear buffer flags to keep them fresh
      // (DMA keeps running, we just don't send the data)
      g_buffer_A_ready = 0;
      g_buffer_B_ready = 0;
    }

    // ========== Status Print (only when idle) ==========
    if (!g_recording && !g_waiting_to_record && (HAL_GetTick() - last_status_time > 5000))
    {
      last_status_time = HAL_GetTick();
      printf("Idle - waiting for tap...\r\n");
    }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief DFSDM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 64;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 17;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0;
  hdfsdm1_channel0.Init.RightBitShift = 0x07;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */
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
  hi2c1.Init.Timing = 0x0090194B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */
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
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC3 PC4
                           PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_10 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE9 PE11 PE13
                           PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE10 PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // WS2812 data pin: PA0 -> TIM2_CH1
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PC2: DFSDM1_CKOUT */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PD3: DFSDM1_DATIN0 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* PC10 (TX) and PC11 (RX) as USART3 */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ========================================================================== */
/*                           WS2812 LOW-LEVEL                                */
/* ========================================================================== */
// Set a single LED's color (n = index, 0-based). GRB order for WS2812.
static void ws2812_set_pixel(uint32_t n, uint8_t r, uint8_t g, uint8_t b)
{
  if (n >= LED_COUNT)
    return;

  uint32_t idx = n * LED_BITS;
  uint8_t colors[3] = {g, r, b}; // WS2812 expects G, then R, then B

  for (int c = 0; c < 3; c++)
  {
    for (int bit = 0; bit < 8; bit++)
    {
      if (colors[c] & (1 << (7 - bit)))
      {
        ws_buf[idx] = WS_T1H; // "1" bit
      }
      else
      {
        ws_buf[idx] = WS_T0H; // "0" bit
      }
      idx++;
    }
  }
}

// Fill whole strip with one color
static void ws2812_fill(uint8_t r, uint8_t g, uint8_t b)
{
  for (uint32_t i = 0; i < LED_COUNT; i++)
  {
    ws2812_set_pixel(i, r, g, b);
  }
}

// Send buffer with TIM2 + DMA
static void ws2812_show(void)
{
  // Reset slots at the end (line low for >50us)
  for (uint32_t i = LED_COUNT * LED_BITS; i < BUF_LEN; i++)
  {
    ws_buf[i] = 0;
  }

  ws_busy = 1;

  if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)ws_buf, BUF_LEN) != HAL_OK)
  {
    Error_Handler();
  }

  // Block until DMA is done
  while (ws_busy)
  {
  }

  // Extra reset time (safety)
  HAL_Delay(1);
}

// TIM2 PWM DMA complete callback to stop the transfer
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
    //    ws2812_busy = 0;
    ws_busy = 0;
  }
}

// === HIGH-LEVEL LED HELPERS ================================================

void LED_AllOff(void)
{
  ws2812_fill(0, 0, 0);
  ws2812_show();
}

void LED_AllColor(uint8_t r, uint8_t g, uint8_t b)
{
  ws2812_fill(r, g, b);
  ws2812_show();
}
// Show a vertical "bar": first num_on LEDs ON with (r,g,b), rest OFF
void LED_ShowBar(uint8_t num_on, uint8_t r, uint8_t g, uint8_t b)
{
  if (num_on > LED_COUNT)
    num_on = LED_COUNT;

  for (uint8_t i = 0; i < LED_COUNT; i++)
  {
    if (i < num_on)
      ws2812_set_pixel(i, r, g, b);
    else
      ws2812_set_pixel(i, 0, 0, 0);
  }
  ws2812_show();
}
static void LED_Service(void)
{
  uint32_t now = HAL_GetTick();

  switch (g_led_state)
  {
  case LED_STATE_IDLE:
    // Idle: keep simple solid blue (all 15 LEDs)
    LED_AllColor(0, 0, 255);
    break;

  case LED_STATE_RECORDING:
    // blink orange ~every 100 ms (same as before)
    if (now - g_led_last_toggle_ms >= 100)
    {
      g_led_last_toggle_ms = now;
      g_led_on = !g_led_on;

      if (g_led_on)
      {
        // All LEDs orange while "on" phase
        LED_AllColor(255, 80, 0);
      }
      else
      {
        LED_AllOff();
      }
    }
    break;

  // ----------- BAR GRAPH LEVELS -----------
  // Red for empty + 100 mL
  case LED_STATE_EMPTY:
    // 2 red LEDs ON
    LED_ShowBar(2, 255, 0, 0);
    break;

  case LED_STATE_100ML:
    // 4 red LEDs ON
    LED_ShowBar(4, 255, 0, 0);
    break;

  // Yellow for 300 + 500 mL
  case LED_STATE_300ML:
    // 6 yellow LEDs ON
    LED_ShowBar(6, 255, 255, 0);
    break;

  case LED_STATE_500ML:
    // 8 yellow LEDs ON
    LED_ShowBar(8, 255, 255, 0);
    break;

  // Green for 700 + 900 mL
  case LED_STATE_700ML:
    // 10 green LEDs ON
    LED_ShowBar(10, 0, 255, 0);
    break;

  case LED_STATE_900ML:
    // All 15 green
    LED_ShowBar(15, 0, 255, 0);
    break;

  default:
    break;
  }
}

// static void LED_Service(void)
//{
//	uint32_t now = HAL_GetTick();
//
//	  switch (g_led_state)
//	  {
//	    case LED_STATE_IDLE:
//	      // default idle color (blue)
//	      LED_AllColor(0, 0, 255);
//	      break;
//
//	    case LED_STATE_RECORDING:
//	      // blink orange ~every 100 ms (same as before)
//	      if (now - g_led_last_toggle_ms >= 100)
//	      {
//	        g_led_last_toggle_ms = now;
//	        g_led_on = !g_led_on;
//
//	        if (g_led_on)
//	        {
//	          LED_AllColor(255, 80, 0);   // orange-ish
//	        }
//	        else
//	        {
//	          LED_AllOff();
//	        }
//	      }
//	      break;
//
//	    // --------- GROUPED COLORS ----------
//	    // Red for empty + 100 mL
//	    case LED_STATE_EMPTY:
//	    case LED_STATE_100ML:
//	      LED_AllColor(255, 0, 0);       // red
//	      break;
//
//	    // Yellow for 300 + 500 mL
//	    case LED_STATE_300ML:
//	    case LED_STATE_500ML:
//	      LED_AllColor(255, 255, 0);     // yellow
//	      break;
//
//	    // Green for 700 + 900 mL
//	    case LED_STATE_700ML:
//	    case LED_STATE_900ML:
//	      LED_AllColor(0, 255, 0);       // green
//	      break;
//
//	    default:
//	      break;
//	  }
//   uint32_t now = HAL_GetTick();
//
//   switch (g_led_state)
//   {
//     case LED_STATE_IDLE:
//       // up to you: keep last color, or turn off
//         LED_AllColor(0, 0, 255);   // R,G,B -> blue
//       break;
//
//     case LED_STATE_RECORDING:
//       // blink orange ~every 100 ms
//       if (now - g_led_last_toggle_ms >= 100)
//       {
//         g_led_last_toggle_ms = now;
//         g_led_on = !g_led_on;
//
//         if (g_led_on)
//         {
//           // orange-ish
//           LED_AllColor(255, 80, 0);
//         }
//         else
//         {
//           LED_AllOff();
//         }
//       }
//       break;
//
//     case LED_STATE_LEVEL_QUARTER:
//       // quarter full = red
//       LED_AllColor(255, 0, 0);
//       break;
//
//     case LED_STATE_LEVEL_HALF:
//       // half full = yellow
//       LED_AllColor(255, 255, 0);
//       break;
//
//     case LED_STATE_LEVEL_THREEQ:
//       // 3/4 full = green
//       LED_AllColor(0, 255, 0);
//       break;
//
//     default:
//       break;
//   }
// }

/* ========================================================================== */
/*                           PRINTF REDIRECT                                  */
/* ========================================================================== */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* ========================================================================== */
/*                           AUDIO FUNCTIONS                                  */
/* ========================================================================== */

/**
 * @brief  Send START marker to signal recording beginning
 */
void Send_Start_Marker(void)
{
  uint8_t marker[2] = {START_MARKER_0, START_MARKER_1};
  HAL_UART_Transmit(&huart3, marker, 2, 10);
}

/**
 * @brief  Send STOP marker to signal recording end
 */
void Send_Stop_Marker(void)
{
  uint8_t marker[2] = {STOP_MARKER_0, STOP_MARKER_1};
  HAL_UART_Transmit(&huart3, marker, 2, 10);
}

/**
 * @brief  Send PCM buffer over UART3 using DMA
 */
void Send_PCM_Via_UART_DMA(int16_t *pData, uint16_t numSamples)
{
  static uint8_t header[4];

  header[0] = SYNC_HEADER_0;
  header[1] = SYNC_HEADER_1;
  header[2] = (uint8_t)(numSamples & 0xFF);
  header[3] = (uint8_t)((numSamples >> 8) & 0xFF);
  HAL_UART_Transmit(&huart3, header, 4, 10);

  g_uart_tx_busy = 1;
  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)pData, numSamples * 2);
}

/**
 * @brief  Convert 32-bit DFSDM to 16-bit PCM
 */
static void Safe_Convert_To_16Bit(int32_t *src, int16_t *dest, uint16_t samples)
{
  for (uint16_t i = 0; i < samples; ++i)
  {
    // Hardware RightBitShift = 5, Software shift = 3
    int32_t val = src[i] >> 5;

    if (val > 32767)
      val = 32767;
    else if (val < -32768)
      val = -32768;

    dest[i] = (int16_t)val;
  }
}

/* ========================================================================== */
/*                           DFSDM CALLBACKS                                  */
/* ========================================================================== */

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
  Safe_Convert_To_16Bit(&dfsdm_dma_buf[0], pcm_buffer_A, PCM_BLOCK_SAMPLES);
  g_buffer_A_ready = 1;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *h)
{
  Safe_Convert_To_16Bit(&dfsdm_dma_buf[PCM_BLOCK_SAMPLES], pcm_buffer_B, PCM_BLOCK_SAMPLES);
  g_buffer_B_ready = 1;
}

/* ========================================================================== */
/*                           IMU FUNCTIONS                                    */
/* ========================================================================== */

void LSM6DSO32_Debug_ReadStatus(I2C_HandleTypeDef *hi2c)
{
  uint8_t whoami = 0;
  uint8_t tap_src = 0;
  uint8_t all_int_src = 0;
  uint8_t status_reg = 0;

  HAL_I2C_Mem_Read(hi2c, LSM6DSO32_I2C_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT, &whoami, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(hi2c, LSM6DSO32_I2C_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &tap_src, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(hi2c, LSM6DSO32_I2C_ADDR, 0x1A, I2C_MEMADD_SIZE_8BIT, &all_int_src, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(hi2c, LSM6DSO32_I2C_ADDR, 0x1E, I2C_MEMADD_SIZE_8BIT, &status_reg, 1, HAL_MAX_DELAY);

  printf("WHO_AM_I: 0x%02X (expect 0x6C)\r\n", whoami);
  printf("TAP_SRC: 0x%02X\r\n", tap_src);
  printf("ALL_INT: 0x%02X\r\n", all_int_src);
  printf("STATUS: 0x%02X\r\n", status_reg);
}

void LSM6DSO32_Init_SingleTap(I2C_HandleTypeDef *hi2c)
{
  uint8_t reg_data;
  HAL_StatusTypeDef ret;

  //  reg_data = 0x60;
  reg_data = 0x6C;
  ret = HAL_I2C_Mem_Write(hi2c, LSM6DSO32_I2C_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
  if (ret != HAL_OK)
  {
    printf("IMU Init Step 1 FAILED\r\n");
    return;
  }

  reg_data = 0x0E;
  ret = HAL_I2C_Mem_Write(hi2c, LSM6DSO32_I2C_ADDR, 0x56, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
  if (ret != HAL_OK)
  {
    printf("IMU Init Step 2 FAILED\r\n");
    return;
  }

  reg_data = 0x04;
  ret = HAL_I2C_Mem_Write(hi2c, LSM6DSO32_I2C_ADDR, 0x57, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
  if (ret != HAL_OK)
  {
    printf("IMU Init Step 3 FAILED\r\n");
    return;
  }

  reg_data = 0x84;
  ret = HAL_I2C_Mem_Write(hi2c, LSM6DSO32_I2C_ADDR, 0x58, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
  if (ret != HAL_OK)
  {
    printf("IMU Init Step 4 FAILED\r\n");
    return;
  }

  reg_data = 0x04;
  ret = HAL_I2C_Mem_Write(hi2c, LSM6DSO32_I2C_ADDR, 0x59, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
  if (ret != HAL_OK)
  {
    printf("IMU Init Step 5 FAILED\r\n");
    return;
  }

  reg_data = 0x06;
  ret = HAL_I2C_Mem_Write(hi2c, LSM6DSO32_I2C_ADDR, 0x5A, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
  if (ret != HAL_OK)
  {
    printf("IMU Init Step 6 FAILED\r\n");
    return;
  }

  reg_data = 0x00;
  ret = HAL_I2C_Mem_Write(hi2c, LSM6DSO32_I2C_ADDR, 0x5B, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
  if (ret != HAL_OK)
  {
    printf("IMU Init Step 7 FAILED\r\n");
    return;
  }

  reg_data = 0x40;
  ret = HAL_I2C_Mem_Write(hi2c, LSM6DSO32_I2C_ADDR, 0x5E, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
  if (ret != HAL_OK)
  {
    printf("IMU Init Step 8 FAILED\r\n");
    return;
  }

  printf("IMU tap detection configured.\r\n");
}

/* ========================================================================== */
/*                           HAL CALLBACKS                                    */
/* ========================================================================== */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    g_single_tap_flag = 1;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    g_uart_tx_busy = 0;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //  if (huart->Instance == USART3)
  //  {
  //	  // Decode one-byte command from Python
  //	  switch (usart3_rx_byte) {
  //		case CMD_LEVEL_QUARTER:
  //		  g_led_state = LED_STATE_LEVEL_QUARTER;
  //		  break;
  //
  //		case CMD_LEVEL_HALF:
  //		  g_led_state = LED_STATE_LEVEL_HALF;
  //		  break;
  //
  //		case CMD_LEVEL_THREEQ:
  //		  g_led_state = LED_STATE_LEVEL_THREEQ;
  //		  break;
  //
  //		default:
  //		  // ignore unknown commands
  //		  break;
  //	  }
  //    HAL_UART_Receive_IT(&huart3, &usart3_rx_byte, 1);
  //  }
  if (huart->Instance == USART3)
  {
    switch (usart3_rx_byte)
    {
    case CMD_LEVEL_EMPTY:
      g_led_state = LED_STATE_EMPTY;
      break;

    case CMD_LEVEL_100ML:
      g_led_state = LED_STATE_100ML;
      break;

    case CMD_LEVEL_300ML:
      g_led_state = LED_STATE_300ML;
      break;

    case CMD_LEVEL_500ML:
      g_led_state = LED_STATE_500ML;
      break;

    case CMD_LEVEL_700ML:
      g_led_state = LED_STATE_700ML;
      break;

    case CMD_LEVEL_900ML:
      g_led_state = LED_STATE_900ML;
      break;

    default:
      // ignore unknown commands
      break;
    }

    HAL_UART_Receive_IT(&huart3, &usart3_rx_byte, 1);
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