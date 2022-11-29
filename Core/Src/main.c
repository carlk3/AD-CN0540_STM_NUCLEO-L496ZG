/* USER CODE BEGIN Header */
/**
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

#include <assert.h>
#include <stdlib.h>
#include <stdatomic.h>
//
#include "cn0540_app_config.h"
#include "cn0540_init_params.h"
#include "platform_drivers.h"
//extern "C"{
#include "AD77681/ad77681.h"
#include "CN0540_FFT/cn0540_adi_fft.h"
#include "LTC26X6/ltc26x6.h"
//}
#include "console.h"
//
#include "printf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define wait_ms HAL_Delay

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

// Descriptor of the main device - the ADC AD7768-1
struct ad77681_dev *device_adc;
// Structure carying measured data, sampled by the ADC
struct adc_data measured_data;
// AD7768-1's status register structure, carying all the error flags
struct ad77681_status_registers *current_status;
// Initialize the interrupt event variable
volatile bool int_event= false;

// Descriptor of the DAC LTC2606
struct ltc26x6_dev *device_dac;

// Structure carying data, the FFT module works with
struct fft_entry *FFT_data;
// Structure carying measuremtns from the FFT module
struct fft_measurements *FFT_meas;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */


void drdy_interrupt();
static int32_t getUserInput(uint32_t *UserInput);
static void go_to_error();
static void print_title();
static void print_prompt();
//int32_t static getMenuSelect(uint16_t *menuSelect);
//int32_t static getLargeMenuSelect(uint32_t *largeSelect);
static void print_binary(uint8_t number, char *binary_number);
static void menu_1_set_adc_powermode(void);
static void menu_2_set_adc_clock_divider(void);
static void menu_3_set_adc_filter_type(void);
static void set_adc_FIR_filter(void);
static void set_adc_SINC5_filter(void);
static void set_adc_SINC3_filter(void);
static void set_adc_50HZ_rej(void);
static void set_adc_user_defined_FIR(void);
static void menu_4_adc_buffers_controll(void);
static void menu_5_set_default_settings(void);
static void menu_6_set_adc_vcm(void);
static void menu_7_adc_read_register(void);
static void menu_8_adc_cont_read_data(void);
//static void adc_data_read(void);
static void cont_sampling();
static void menu_9_reset_ADC(void);
static void menu_10_power_down(void);
static void menu_11_ADC_GPIO(void);
static void adc_GPIO_write(void);
static void adc_GPIO_inout(void);
static void adc_GPIO_settings(void);
static void menu_12_read_master_status(void);
static void menu_13_mclk_vref(void);
static void menu_14_print_measured_data(void);
static void menu_15_set_adc_data_output_mode(void);
static void menu_16_set_adc_diagnostic_mode(void);
static void menu_17_do_the_fft(void);
static void menu_18_fft_settings(void);
static void menu_19_gains_offsets(void);
static void menu_20_check_scratchpad(void);
static void menu_21_piezo_offset(void);
static void menu_22_set_DAC_output(void);
static void get_mean_voltage(struct adc_data *measured_data, double *mean_voltage);
static void adc_hard_reset(void);
static void sdpk1_gpio_setup(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * ADC data recteption interrupt from DRDY
 *
 * Data reception from the ADC using interrupt generated by the ADC's DRDY (Data Ready) pin
 * Interrupt triggers falling edge of the active-high DRDY pulse
 * DRDY pulse is generated by the ADC and frequency of the DRDY pulse depends on the ADC settings:
 *
 * DRDY frequency = MCLK / ( MCLK_DIV * FILTER_OSR )
 */
void drdy_interrupt() {
	int_event = true;

	if (measured_data.count == measured_data.samples) { // Desired numer of samples has been taken, set everything back
//		drdy.disable_irq();                             // Disable interrupt on DRDY pin
		LL_EXTI_DisableIT_0_31(DRDY_Pin);
		measured_data.finish = true;                    // Desired number of samples has been taken
		measured_data.count = 0;                        // Set measured data counter to 0
	}
}
/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (DRDY_Pin == GPIO_Pin)
		drdy_interrupt();
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
  MX_LPUART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  int32_t connected = FAILURE;
  uint32_t menu;

  sdpk1_gpio_setup();                                                     // Setup SDP-K1 GPIOs
  adc_hard_reset();                                                       // Perform ADC hard reset

  ad77681_soft_reset(device_adc);

  connected = ad77681_setup(&device_adc, init_params, &current_status);   // SETUP and check connection
  if(connected == FAILURE)
      go_to_error();
  ltc26x6_init(&device_dac, init_params_dac);                             // Initialize DAC

  #ifdef CN0540_ADI_FFT_H_                                                // if the adi_fft.h is included , initialize it
  FFT_init_params(&FFT_data, &FFT_meas);                                  // Initialize FFT structure and allocate space
  update_FFT_enviroment(device_adc->vref, device_adc->mclk, device_adc->sample_rate, FFT_data);   // Update the Vref, Mclk, Sampling rate
  measured_data.samples = 4096;                                           // Initialize FFT with 4096 samples
  FFT_init(measured_data.samples, FFT_data);                              // Update sample count for FFT
  #endif // CN0540_ADI_FFT_H_
  // FFT module is initializedd with 4096 samples and 7-term BH window

  print_title();
  print_prompt();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	for (;;) {
		if (SUCCESS == getUserInput(&menu)) { // Settings menu SWITCH
			switch (menu) {
			case 1:
				menu_1_set_adc_powermode();     // Set ADC power mode
				break;
			case 2:
				menu_2_set_adc_clock_divider(); // Set ADC clock divider
				break;
			case 3:
				menu_3_set_adc_filter_type();   // Set ad7768-1 filter type
				break;
			case 4:
				menu_4_adc_buffers_controll();  // Set ADC AIN & Reference buffers
				break;
			case 5:
				menu_5_set_default_settings();  // Set ADC default value
				break;
			case 6:
				menu_6_set_adc_vcm();           // Set ADC VCM
				break;
			case 7:
				menu_7_adc_read_register();     // Read ADC register
				break;
			case 8:
				menu_8_adc_cont_read_data();    // Perform continuous ADC read
				break;
			case 9:
				menu_9_reset_ADC();             // Reset ADC
				break;
			case 10:
				menu_10_power_down();           // ADC Wake up and sleep
				break;
			case 11:
				menu_11_ADC_GPIO();             // Set ADC GPIOs
				break;
			case 12:
				menu_12_read_master_status();   // Read ADC master status
				break;
			case 13:
				menu_13_mclk_vref();            // Set ADC MCLK / Vref
				break;
			case 14:
				menu_14_print_measured_data();  // Print continouos ADC read data
				break;
			case 15:
				menu_15_set_adc_data_output_mode(); // Set ADC data output mode
				break;
			case 16:
				menu_16_set_adc_diagnostic_mode();  // Set ADC to diagnostic mode
				break;
			case 17:
				menu_17_do_the_fft();               // Perform FFT
				break;
			case 18:
				menu_18_fft_settings();             // Change FFT settins
				break;
			case 19:
				menu_19_gains_offsets();            // Set ADC gain and offset
				break;
			case 20:
				menu_20_check_scratchpad();         // Perform scratchpad check
				break;
			case 21:
				menu_21_piezo_offset();             // Compensate piezo offset
				break;
			case 22:
				menu_22_set_DAC_output();           // Set DAC output mode
				break;
			default:
				printf("Invalid option");
				print_prompt();
				break;
			}
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00909BEB;
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

  LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPUART1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOG);
  HAL_PWREx_EnableVddIO2();
  /**LPUART1 GPIO Configuration
  PG7   ------> LPUART1_TX
  PG8   ------> LPUART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7|LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* LPUART1 interrupt Init */
  NVIC_SetPriority(LPUART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(LPUART1_IRQn);

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  LPUART_InitStruct.BaudRate = 115200;
  LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
  LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
  LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
  LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
  LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
  LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
  LL_LPUART_Enable(LPUART1);
  /* USER CODE BEGIN LPUART1_Init 2 */

  LL_LPUART_ClearFlag_PE(LPUART1);
  LL_LPUART_ClearFlag_FE(LPUART1);
  LL_LPUART_ClearFlag_NE(LPUART1);
  LL_LPUART_ClearFlag_ORE(LPUART1);
  LL_LPUART_EnableIT_RXNE(LPUART1);

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_ADC_GPIO_Port, RESET_ADC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSB_AUX_GPIO_Port, CSB_AUX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CS_ADC_Pin|SHUTDOWN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE10
                           PE12 PE14 PE15 PE0
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF6 PF7
                           PF8 PF9 PF10 PF11
                           PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : IO4_Pin PC1 PC2 IO3_Pin
                           PC4 IO0_Pin PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = IO4_Pin|GPIO_PIN_1|GPIO_PIN_2|IO3_Pin
                          |GPIO_PIN_4|IO0_Pin|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 IO5_Pin
                           PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|IO5_Pin
                          |GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB15
                           PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_ADC_Pin */
  GPIO_InitStruct.Pin = RESET_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_ADC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_FF_Pin */
  GPIO_InitStruct.Pin = SW_FF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_FF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG3
                           PG4 PG9 PG10 PG11
                           PG12 PG13 PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SYNC_IN_Pin DRDY_AUX_Pin */
  GPIO_InitStruct.Pin = SYNC_IN_Pin|DRDY_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CSB_AUX_Pin */
  GPIO_InitStruct.Pin = CSB_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSB_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin CS_ADC_Pin SHUTDOWN_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|CS_ADC_Pin|SHUTDOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD11 PD12 PD13
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * Error warning, in case of unsuccessfull SPI connection
 *
 */
static void go_to_error()
{
    int32_t connected = FAILURE;
//    uint8_t scratchpad_sequence = 0xAD;
    while (1) {
        printf_("ERROR: NOT CONNECTED\nCHECK YOUR PHYSICAL CONNECTION\n\n");  // When not connected, keep showing error message
//        wait(5);
        HAL_Delay(5000);
        connected = ad77681_setup(&device_adc, init_params, &current_status);   // Keep trying to connect
        if (connected == SUCCESS) {
            printf_("SUCCESSFULLY RECONNECTED\n\n");                          // If successfull reading from scratchpad, init the ADC and go back
            break;
        }
    }
}

/**
 * Print title
 *
 */
static void print_title() {
    printf_("\n\r");
    printf_("****************************************************************\n");
    printf_("*      EVAL-CN0540-PMDZ Demonstration Program -- (mbed)        *\n");
    printf_("*                                                              *\n");
    printf_("*   This program demonstrates IEPE / ICP piezo accelerometer   *\n");
    printf_("*        interfacing and FFT measurements using AD7768-1       *\n");
    printf_("*           Precision 24-bit sigma-delta AD converter          *\n");
    printf_("*                                                              *\n");
    printf_("* Set the baud rate to 115200 select the newline terminator.   *\n");
    printf_("****************************************************************\n");
}

/**
 * Print main menu to console
 *
 */
static void print_prompt() {
    printf_("\n\nCommand Summary:\n\n");
    printf_("  1  - Set ADC power mode\n");
    printf_("  2  - Set ADC MCLK divider\n");
    printf_("  3  - Set ADC filter type\n");
    printf_("  4  - Set ADC AIN and REF buffers\n");
    printf_("  5  - Set ADC to default config\n");
    printf_("  6  - Set ADC VCM output\n");
    printf_("  7  - Read desired ADC register\n");
    printf_("  8  - Read continuous ADC data\n");
    printf_("  9  - Reset ADC\n");
    printf_("  10 - ADC Power-down\n");
    printf_("  11 - Set ADC GPIOs\n");
    printf_("  12 - Read ADC master status\n");
    printf_("  13 - Set ADC Vref and MCLK\n");
    printf_("  14 - Print ADC measured data\n");
    printf_("  15 - Set ADC data output mode\n");
    printf_("  16 - Set ADC diagnostic mode\n");
    printf_("  17 - Do the FFT\n");
    printf_("  18 - FFT settings\n");
    printf_("  19 - Set ADC Gains, Offsets\n");
    printf_("  20 - ADC Scratchpad Check\n");
    printf_("  21 - Compenzate Piezo sensor offset\n");
    printf_("  22 - Set DAC output\n");
    printf_("\n\r");
}

/**
 * Read user input from uart
 * *UserInput = 0 if failure
 *
 */
static int32_t getUserInput(uint32_t *UserInput)
{
    long uart_val;
    int32_t ret;

    while (!cnsl_buf_full);
    ret = sscanf((char *)cnsl_buf, "%ld", &uart_val);       // Return 1 = OK, Return 0 = Fail
    cnsl_buf_clear();
    if((ret == 0) || (uart_val < 0)) {      // Failure if uart_val is negative, or non-digit
        *UserInput = 0;
        return FAILURE;
    }
    *UserInput = (uint32_t)(uart_val);
    return SUCCESS;
}

/**
 * Set power mode
 *
 */
 static void menu_1_set_adc_powermode(void)
 {
    uint32_t new_pwr_mode;

    printf_(" Avaliable power modes: \n");
    printf_("  1 - Low power mode\n");
    printf_("  2 - Median power mode\n");
    printf_("  3 - Fast power mode\n");
    printf_(" Select an option: \n");

    getUserInput(&new_pwr_mode);
    putchar('\n');

    switch (new_pwr_mode) {
    case 1:
        ad77681_set_power_mode(device_adc, AD77681_ECO);
        printf_(" Low power mode selected\n");
        break;
    case 2:
        ad77681_set_power_mode(device_adc, AD77681_MEDIAN);
        printf_(" Median power mode selected\n");
        break;
    case 3:
        ad77681_set_power_mode(device_adc, AD77681_FAST);
        printf_(" Fast power mode selected\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Set clock divider
 *
 */
 static void menu_2_set_adc_clock_divider(void)
 {
    uint32_t new_mclk_div;

    printf_(" Avaliable MCLK divide options: \n");
    printf_("  1 - MCLK/16\n");
    printf_("  2 - MCLK/8\n");
    printf_("  3 - MCLK/4\n");
    printf_("  4 - MCLK/2\n");
    printf_(" Select an option: \n");

    getUserInput(&new_mclk_div);
    putchar('\n');

    switch (new_mclk_div) {
    case 1:
        ad77681_set_mclk_div(device_adc, AD77681_MCLK_DIV_16);
        printf_(" MCLK/16 selected\n");
        break;
    case 2:
        ad77681_set_mclk_div(device_adc, AD77681_MCLK_DIV_8);
        printf_(" MCLK/8 selected\n");
        break;
    case 3:
        ad77681_set_mclk_div(device_adc, AD77681_MCLK_DIV_4);
        printf_(" MCLK/4 selected\n");
        break;
    case 4:
        ad77681_set_mclk_div(device_adc, AD77681_MCLK_DIV_2);
        printf_(" MCLK/2 selected\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    // Update the sample rate after changing the MCLK divider
    ad77681_update_sample_rate(device_adc);
    print_prompt();
}

/**
 * Set filter type
 *
 */
 static void menu_3_set_adc_filter_type(void)
{
    uint32_t new_filter = 0;
//    int32_t ret;

    printf_(" Avaliable clock divide options: \n");
    printf_(" 1 - SINC3 Fileter\n");
    printf_(" 2 - SINC5 Filter\n");
    printf_(" 3 - Low ripple FIR Filter\n");
    printf_(" 4 - SINC3 50/60Hz rejection\n");
    printf_(" 5 - User-defined FIR filter\n");
    printf_(" Select an option: \n");

    getUserInput(&new_filter);
    putchar('\n');

    switch (new_filter) {
    case 1:
        set_adc_SINC3_filter();
        break;
    case 2:
        set_adc_SINC5_filter();
        break;
    case 3:
        set_adc_FIR_filter();
        break;
    case 4:
        set_adc_50HZ_rej();
        break;
    case 5:
        set_adc_user_defined_FIR();
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    // Update the sample rate after changing the Filter type
    ad77681_update_sample_rate(device_adc);
    print_prompt();
}

/**
 * Set SINC3 filter
 *
 */
static void set_adc_SINC3_filter(void)
{
    uint32_t new_sinc3 = 0; //, new_sinc5 = 0;
    int32_t ret;

    printf_(" SINC3 filter Oversampling ratios: \n");
    printf_(" OSR is calculated as (x + 1)*32\n");
    printf_(" x is SINC3 OSR register value\n");
    printf_(" Please input a value from 0 to 8192 = 2^13\n  :");

    ret = getUserInput(&new_sinc3);

//    if ((new_sinc3 >= 0) && (new_sinc3 <= 8192) && (ret == SUCCESS)) {
	if ((new_sinc3 <= 8192) && (ret == SUCCESS)) {
        printf_("%lu\n", new_sinc3);
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx32, AD77681_SINC3, new_sinc3);
        printf_(" SINC3 OSR is set to %lu\n", (new_sinc3 + 1) * 32);
    } else {
        printf_("%lu\n", new_sinc3);
        printf_(" Invalid option - too large number\n");
    }
}

/**
 * Set SINC5 filter
 *
 */
static void set_adc_SINC5_filter(void)
{
    uint32_t new_sinc5;

    printf_(" SINC5 filter Oversampling ratios: \n");
    printf_("  1 - Oversampled by 8\n");
    printf_("  2 - Oversampled by 16\n");
    printf_("  3 - Oversampled by 32\n");
    printf_("  4 - Oversampled by 64\n");
    printf_("  5 - Oversampled by 128\n");
    printf_("  6 - Oversampled by 256\n");
    printf_("  7 - Oversampled by 512\n");
    printf_("  8 - Oversampled by 1024\n");
    printf_(" Select an option: \n");

    getUserInput(&new_sinc5);

    putchar('\n');

    switch (new_sinc5) {
    case 1:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx32, AD77681_SINC5_DECx8, 0);
        printf_(" SINC5 with OSRx8 set\n");
        break;
    case 2:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx32, AD77681_SINC5_DECx16, 0);
        printf_(" SINC5 with OSRx16 set\n");
        break;
    case 3:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx32, AD77681_SINC5, 0);
        printf_(" SINC5 with OSRx32 set\n");
        break;
    case 4:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx64, AD77681_SINC5, 0);
        printf_(" SINC5 with OSRx64 set\n");
        break;
    case 5:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx128, AD77681_SINC5, 0);
        printf_(" SINC5 with OSRx128 set\n");
        break;
    case 6:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx256, AD77681_SINC5, 0);
        printf_(" SINC5 with OSRx256 set\n");
        break;
    case 7:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx512, AD77681_SINC5, 0);
        printf_(" SINC5 with OSRx512 set\n");
        break;
    case 8:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx1024, AD77681_SINC5, 0);
        printf_(" SINC5 with OSRx1024 set\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
}

/**
 * Set FIR filter
 *
 */
static void set_adc_FIR_filter(void)
{
    uint32_t new_fir;

    printf_(" FIR filter Oversampling ratios: \n");
    printf_("  1 - Oversampled by 32\n");
    printf_("  2 - Oversampled by 64\n");
    printf_("  3 - Oversampled by 128\n");
    printf_("  4 - Oversampled by 256\n");
    printf_("  5 - Oversampled by 512\n");
    printf_("  6 - Oversampled by 1024\n");
    printf_(" Select an option: \n");

    getUserInput(&new_fir);

    putchar('\n');

    switch (new_fir) {
    case 1:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx32, AD77681_FIR, 0);
        printf_(" FIR with OSRx32 set\n");
        break;
    case 2:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx64, AD77681_FIR, 0);
        printf_(" FIR with OSRx64 set\n");
        break;
    case 3:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx128, AD77681_FIR, 0);
        printf_(" FIR with OSRx128 set\n");
        break;
    case 4:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx256, AD77681_FIR, 0);
        printf_(" FIR with OSRx256 set\n");
        break;
    case 5:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx512, AD77681_FIR, 0);
        printf_(" FIR with OSRx512 set\n");
        break;
    case 6:
        ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx1024, AD77681_FIR, 0);
        printf_(" FIR with OSRx1024 set\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
}

/**
 * Set 50HZ rejection bit when SINC3 is being used
 *
 */
static void set_adc_50HZ_rej(void)
{
    uint32_t new_50Hz;

    printf_(" SINC3 50/60Hz rejection: \n");
    printf_("  1 - 50/60Hz rejection enable \n");
    printf_("  2 - 50/60Hz rejection disable \n");
    printf_(" Select an option: \n");

    getUserInput(&new_50Hz);

    putchar('\n');

    switch (new_50Hz)
    {
    case 1:
        ad77681_set_50HZ_rejection(device_adc, ENABLE);
        printf_(" SINC3 50/60Hz rejection enabled\n");
        break;
    case 2:
        ad77681_set_50HZ_rejection(device_adc, DISABLE);
        printf_(" SINC3 50/60Hz rejection disabled\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
}

/**
 * Insert user-defined FIR filter coeffs
 *
 */
static void set_adc_user_defined_FIR(void)
{
    const uint8_t coeff_reg_length = 56; // Maximum allowed number of coefficients in the coeff register

    printf_(" User Defined FIR filter\n");

    if ((ARRAY_SIZE(programmable_FIR) <= coeff_reg_length) && (count_of_active_coeffs <= coeff_reg_length)) {
        printf_("  Aplying user-defined FIR filter coefficients from 'FIR_user_coeffs.h'\n");
        ad77681_programmable_filter(device_adc, programmable_FIR, count_of_active_coeffs);
        printf_(" Coeffs inserted successfully\n");
    } else
        printf_("  Incorrect count of coefficients in 'FIR_user_coeffs.h'\n");
}

/**
 * AIN and REF buffers controll
 *
 */
 static void menu_4_adc_buffers_controll(void)
 {
    uint32_t new_AIN_buffer = 0, new_REF_buffer = 0, new_buffer = 0;

    printf_(" Buffers settings: \n");
    printf_("  1 - Set AIN precharge buffers\n");
    printf_("  2 - Set REF buffers\n");
    printf_(" Select an option: \n");

    getUserInput(&new_buffer);
    putchar('\n');

    switch (new_buffer) {
    case 1:
        printf_(" Analog IN precharge buffers settings: \n");
        printf_("  1 - Turn ON  both precharge buffers\n");
        printf_("  2 - Turn OFF both precharge buffers\n");
        printf_("  3 - Turn ON  AIN- precharge buffer\n");
        printf_("  4 - Turn OFF AIN- precharge buffer\n");
        printf_("  5 - Turn ON  AIN+ precharge buffer\n");
        printf_("  6 - Turn OFF AIN+ precharge buffer\n");
        printf_(" Select an option: \n");

        getUserInput(&new_AIN_buffer);
        putchar('\n');

        switch (new_AIN_buffer) {
        case 1:
            ad77681_set_AINn_buffer(device_adc, AD77681_AINn_ENABLED);
            ad77681_set_AINp_buffer(device_adc, AD77681_AINp_ENABLED);
            printf_(" AIN+ and AIN- enabled\n");
            break;
        case 2:
            ad77681_set_AINn_buffer(device_adc, AD77681_AINn_DISABLED);
            ad77681_set_AINp_buffer(device_adc, AD77681_AINp_DISABLED);
            printf_(" AIN+ and AIN- disabled\n");
            break;
        case 3:
            ad77681_set_AINn_buffer(device_adc, AD77681_AINn_ENABLED);
            printf_(" AIN- Enabled\n");
            break;
        case 4:
            ad77681_set_AINn_buffer(device_adc, AD77681_AINn_DISABLED);
            printf_(" AIN- Disabled\n");
            break;
        case 5:
            ad77681_set_AINp_buffer(device_adc, AD77681_AINp_ENABLED);
            printf_(" AIN+ Enabled\n");
            break;
        case 6:
            ad77681_set_AINp_buffer(device_adc, AD77681_AINp_DISABLED);
            printf_(" AIN+ Disabled\n");
            break;
        default:
            printf_(" Invalid option\n");
            break;
        }
        break;
    case 2:
        printf_(" REF buffers settings: \n");
        printf_("  1 - Full REF- reference buffer\n");
        printf_("  2 - Full REF+ reference buffer\n");
        printf_("  3 - Unbuffered REF- reference buffer\n");
        printf_("  4 - Unbuffered REF+ reference buffer\n");
        printf_("  5 - Precharge  REF- reference buffer\n");
        printf_("  6 - Precharge  REF+ reference buffer\n");
        printf_(" Select an option: \n");

        getUserInput(&new_REF_buffer);
        putchar('\n');

        switch (new_REF_buffer) {
        case 1:
            ad77681_set_REFn_buffer(device_adc, AD77681_BUFn_FULL_BUFFER_ON);
            printf_(" Fully buffered REF-\n");
            break;
        case 2:
            ad77681_set_REFp_buffer(device_adc, AD77681_BUFp_FULL_BUFFER_ON);
            printf_(" Fully buffered REF+\n");
            break;
        case 3:
            ad77681_set_REFn_buffer(device_adc, AD77681_BUFn_DISABLED);
            printf_(" Unbuffered REF-\n");
            break;
        case 4:
            ad77681_set_REFp_buffer(device_adc, AD77681_BUFp_DISABLED);
            printf_(" Unbuffered REF+\n");
            break;
        case 5:
            ad77681_set_REFn_buffer(device_adc, AD77681_BUFn_ENABLED);
            printf_(" Precharge buffer on REF-\n");
            break;
        case 6:
            ad77681_set_REFp_buffer(device_adc, AD77681_BUFp_ENABLED);
            printf_(" Precharge buffer on REF+\n");
            break;
        default:
            printf_(" Invalid option\n");
            break;
        }
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Default ADC Settings
 *
 */
 static void menu_5_set_default_settings(void)
 {
    int32_t default_settings_flag  = ad77681_setup(&device_adc, init_params, &current_status);

    if (default_settings_flag == SUCCESS)
        printf_("\n Default ADC settings successfull\n");
    else
        printf_("\n Error in settings, please reset the ADC\n");
    print_prompt();
}

/**
 * VCM output controll
 *
 */
 static void menu_6_set_adc_vcm(void)
 {
    uint32_t new_vcm = 0;

    printf_(" Avaliable VCM output voltage levels: \n");
    printf_("  1 - VCM = (AVDD1-AVSS)/2\n");
    printf_("  2 - VCM = 2.5V\n");
    printf_("  3 - VCM = 2.05V\n");
    printf_("  4 - VCM = 1.9V\n");
    printf_("  5 - VCM = 1.65V\n");
    printf_("  6 - VCM = 1.1V\n");
    printf_("  7 - VCM = 0.9V\n");
    printf_("  8 - VCM off\n");
    printf_(" Select an option: \n");

    getUserInput(&new_vcm);
    putchar('\n');

    switch (new_vcm) {

    case 1:
        ad77681_set_VCM_output(device_adc, AD77681_VCM_HALF_VCC);
        printf_(" VCM set to half of the Vcc\n");
        break;
    case 2:
        ad77681_set_VCM_output(device_adc, AD77681_VCM_2_5V);
        printf_(" VCM set to 2.5V\n");
        break;
    case 3:
        ad77681_set_VCM_output(device_adc, AD77681_VCM_2_05V);
        printf_(" VCM set to 2.05V\n");
        break;
    case 4:
        ad77681_set_VCM_output(device_adc, AD77681_VCM_1_9V);
        printf_(" VCM set to 1.9V\n");
        break;
    case 5:
        ad77681_set_VCM_output(device_adc, AD77681_VCM_1_65V);
        printf_(" VCM set to 1.65V\n");
        break;
    case 6:
        ad77681_set_VCM_output(device_adc, AD77681_VCM_1_1V);
        printf_(" VCM set to 1.1V\n");
        break;
    case 7:
        ad77681_set_VCM_output(device_adc, AD77681_VCM_0_9V);
        printf_(" VCM set to 0.9V\n");
        break;
    case 8:
        ad77681_set_VCM_output(device_adc, AD77681_VCM_OFF);
        printf_(" VCM OFF\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Register read
 *
 */
 static void menu_7_adc_read_register(void)
 {
    uint32_t new_reg_to_read = 0;
    uint8_t reg_read_buf[3], read_adc_data[6]; //, hex_number = 0;
    uint8_t HI = 0, MID = 0, LO = 0;
    char binary_number[9] = {0}; //, other_register[2] = "";
//    double voltage;

    printf_(" Read desired register: \n");
    printf_("  1 - 0x03        - Chip type\n");
    printf_("  2 - 0x14        - Interface format\n");
    printf_("  3 - 0x15        - Power clock\n");
    printf_("  4 - 0x16        - Analog\n");
    printf_("  5 - 0x17        - Analog2\n");
    printf_("  6 - 0x18        - Conversion\n");
    printf_("  7 - 0x19        - Digital filter\n");
    printf_("  8 - 0x1A        - SINC3 Dec. rate MSB\n");
    printf_("  9 - 0x1B        - SINC3 Dec. rate LSB\n");
    printf_(" 10 - 0x1C        - Duty cycle ratio\n");
    printf_(" 11 - 0x1D        - Sync, Reset\n");
    printf_(" 12 - 0x1E        - GPIO Control\n");
    printf_(" 13 - 0x1F        - GPIO Write\n");
    printf_(" 14 - 0x20        - GPIO Read\n");
    printf_(" 15 - 0x21 - 0x23 - Offset register\n");
    printf_(" 16 - 0x24 - 0x26 - Gain register\n");
    printf_(" 17 - 0x2C        - ADC Data\n");
    printf_(" Select an option: \n");

    getUserInput(&new_reg_to_read);
    putchar('\n');

    switch (new_reg_to_read) {
    case 1:
        ad77681_spi_reg_read(device_adc, AD77681_REG_CHIP_TYPE, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x03 - Chip type register is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 2:
        ad77681_spi_reg_read(device_adc, AD77681_REG_INTERFACE_FORMAT, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x14 - Interface format register is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 3:
        ad77681_spi_reg_read(device_adc, AD77681_REG_POWER_CLOCK, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x15 - Power clock register is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 4:
        ad77681_spi_reg_read(device_adc, AD77681_REG_ANALOG, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x16 - Anlaog register is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 5:
        ad77681_spi_reg_read(device_adc, AD77681_REG_ANALOG2, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x17 - Analog2 regster is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 6:
        ad77681_spi_reg_read(device_adc, AD77681_REG_CONVERSION, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x18 - Conversion register is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 7:
        ad77681_spi_reg_read(device_adc, AD77681_REG_DIGITAL_FILTER, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x19 - Digital filter register is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 8:
        ad77681_spi_reg_read(device_adc, AD77681_REG_SINC3_DEC_RATE_MSB, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x1A - SINC3 Dec. rate MSB is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 9:
        ad77681_spi_reg_read(device_adc, AD77681_REG_SINC3_DEC_RATE_LSB, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x1B - SINC3 Dec. rate LSB is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 10:
        ad77681_spi_reg_read(device_adc, AD77681_REG_DUTY_CYCLE_RATIO, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x1C - Duty cycle ratio 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 11:
        ad77681_spi_reg_read(device_adc, AD77681_REG_SYNC_RESET, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x1D - Sync, Reset 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 12:
        ad77681_spi_reg_read(device_adc, AD77681_REG_GPIO_CONTROL, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x1E - GPIO Control is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 13:
        ad77681_spi_reg_read(device_adc, AD77681_REG_GPIO_WRITE, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x1F - GPIO Write is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 14:
        ad77681_spi_reg_read(device_adc, AD77681_REG_GPIO_READ, reg_read_buf);
        print_binary(reg_read_buf[1], binary_number);
        printf_(" Value of 0x20 - GPIO Read is: 0x%x  0b%s\n", reg_read_buf[1], binary_number);
        break;
    case 15:
        ad77681_spi_reg_read(device_adc, AD77681_REG_OFFSET_HI, reg_read_buf);
        HI = reg_read_buf[1];

        ad77681_spi_reg_read(device_adc, AD77681_REG_OFFSET_MID, reg_read_buf);
        MID = reg_read_buf[1];

        ad77681_spi_reg_read(device_adc, AD77681_REG_OFFSET_LO, reg_read_buf);
        LO = reg_read_buf[1];

        printf_(" Value of 0x21 - 0x23 - Offset register is: 0x%x %x %x\n", HI, MID, LO);
        break;

    case 16:
        ad77681_spi_reg_read(device_adc, AD77681_REG_GAIN_HI, reg_read_buf);
        HI = reg_read_buf[1];

        ad77681_spi_reg_read(device_adc, AD77681_REG_GAIN_MID, reg_read_buf);
        MID = reg_read_buf[1];

        ad77681_spi_reg_read(device_adc, AD77681_REG_GAIN_LO, reg_read_buf);
        LO = reg_read_buf[1];

        printf_(" Value of 0x24 - 0x26 - Gain register is: 0x%x %x %x\n", HI, MID, LO);
        break;
    case 17:
        ad77681_spi_read_adc_data(device_adc, read_adc_data, AD77681_REGISTER_DATA_READ);
        printf_(" Value of 0x2C - ADC data is: 0x%x 0x%x 0x%x\n", read_adc_data[1], read_adc_data[2], read_adc_data[3]);
        break;

    default :
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Read ADC data
 *
 */
static void menu_8_adc_cont_read_data(void)
{
    uint32_t new_sample_count = 0;
    int32_t ret;

    printf_(" Read Continuous ADC Data");
    printf_("  Input number of samples (1 to 4096): \n");
    ret = getUserInput(&new_sample_count);                  // Get user input

    if ((new_sample_count <= 4096) && (ret == SUCCESS) ) {
        printf_("\n%lu of samples\n", new_sample_count);   // Print Desired Measurement Count
        measured_data.samples = (uint16_t)(new_sample_count);
        measured_data.finish = false;
        measured_data.count = 0;
        printf_("Sampling....\n");
        cont_sampling();
        printf_("Done Sampling....\n");
    } else {
        printf_(" Invalid option\n");
    }
    print_prompt();
}

/**
 * ADC Continuous read
 *
 */
static void cont_sampling() {
	uint8_t buf[6];

	ad77681_set_continuos_read(device_adc, AD77681_CONTINUOUS_READ_DISABLE);    // Disable continuous ADC read

	__enable_irq();			// Enable all interupts
//    drdy.enable_irq();	// Enable interrupt on DRDY pin
	LL_EXTI_EnableIT_0_31(DRDY_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(DRDY_Pin);
	asm volatile("" ::: "memory");

	ad77681_set_continuos_read(device_adc, AD77681_CONTINUOUS_READ_ENABLE);
//	drdy.fall(&drdy_interrupt);	// Interrupt on falling edne of DRDY

	while (!measured_data.finish) { // While loop. Waiting for the measurements to be completed
		if (int_event == true) {      // Checks if Interrupt Occurred
			ad77681_spi_read_adc_data(device_adc, buf, AD77681_CONTINUOUS_DATA_READ);    // Read the continuous read data
			if (device_adc->conv_len == AD77681_CONV_24BIT)                                                 // 24bit format
				measured_data.raw_data[measured_data.count] = (buf[0] << 16 | buf[1] << 8 | buf[2] << 0); // Combining the SPI buffers
			else
				// 16bit format
				measured_data.raw_data[measured_data.count] = (buf[0] << 8 | buf[1] << 0);          // Combining the SPI buffers
			measured_data.count++;  // Increment Measured Data Counter
			int_event = false;        // Set int event flag after reading the Data
		}
	}
	ad77681_set_continuos_read(device_adc, AD77681_CONTINUOUS_READ_DISABLE);    // Disable continuous ADC read
}

/**
 * Reset ADC
 *
 */
 static void menu_9_reset_ADC(void)
 {
    uint32_t new_reset_option = 0;

    printf_(" ADC reset opportunities: \n");
    printf_("  1 - Soft reset - over SPI\n");
    printf_("  2 - Hard reset - uing RESET pin\n");
    printf_(" Select an option: \n");

    getUserInput(&new_reset_option);
    putchar('\n');

    switch (new_reset_option) {
    case 1:
        ad77681_soft_reset(device_adc);             // Perform soft reset thru SPI write
        printf_(" ADC after soft reset\n");
        break;
    case 2:
        adc_hard_reset();
        printf_(" ADC after hard reset\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Reset ADC thru SDP-K1 GPIO
 *
 *
 */
static void adc_hard_reset() {
//    adc_reset = 0;  // Set ADC reset pin to Low
	HAL_GPIO_WritePin(RESET_ADC_GPIO_Port, RESET_ADC_Pin, GPIO_PIN_RESET);
//	mdelay(100);    // Delay 100ms
	HAL_Delay(100);
//    adc_reset = 1;  // Set ADC reset pin to High
	HAL_GPIO_WritePin(RESET_ADC_GPIO_Port, RESET_ADC_Pin, GPIO_PIN_SET);
//	mdelay(100);    // Delay 100ms
	HAL_Delay(100);
}

/**
 * Sleep mode / Wake up ADC
 *
 */
 static void menu_10_power_down(void)
 {
    uint32_t new_sleep = 0;

    printf_(" Control sleep mode of the ADC: \n");
    printf_("  1 - Put ADC to sleep mode\n");
    printf_("  2 - Wake up ADC\n");
    printf_(" Select an option: \n");

    getUserInput(&new_sleep);
    putchar('\n');

    switch (new_sleep) {
    case 1:
        ad77681_power_down(device_adc, AD77681_SLEEP);
        printf_(" ADC put to sleep mode\n");
        break;
    case 2:
        ad77681_power_down(device_adc, AD77681_WAKE);
        printf_(" ADC powered\n");
        break;
    default:
        printf_("Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * ADC's GPIO Control
 *
 */
 static void menu_11_ADC_GPIO(void)
 {
    uint8_t GPIO_state;
    uint32_t new_gpio_sel = 0;
    char binary_number[8];
//    int32_t ret_val = FAILURE, ret;

    printf_(" ADC GPIO Control: \n");
    printf_("  1 - Read from GPIO\n");
    printf_("  2 - Write to  GPIO\n");
    printf_("  3 - Set GPIO as input / output\n");
    printf_("  4 - Change GPIO settings\n");
    printf_(" Select an option: \n");

    getUserInput(&new_gpio_sel);
    putchar('\n');

    switch (new_gpio_sel) {
    case 1:
        ad77681_gpio_read(device_adc, &GPIO_state, AD77681_ALL_GPIOS);
        print_binary(GPIO_state, binary_number);
        printf_(" Current GPIO Values:\n GPIO0: %c\n GPIO1: %c\n GPIO2: %c\n GPIO3: %c\n", binary_number[7], binary_number[6], binary_number[5], binary_number[4]);
        break;
    case 2:
        adc_GPIO_write();
        break;
    case 3:
        adc_GPIO_inout();
        break;
    case 4:
        adc_GPIO_settings();
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Write to GPIOs, part of the ADC_GPIO function
 *
 */
static void adc_GPIO_write(void)
{
    uint32_t new_gpio_write = 0, new_value = 0;
    int32_t ret, ret_val __attribute__ ((unused));

    printf_(" Write to GPIO: \n");
    printf_("  1 - Write to all GPIOs\n");
    printf_("  2 - Write to GPIO0\n");
    printf_("  3 - Write to GPIO1\n");
    printf_("  4 - Write to GPIO2\n");
    printf_("  5 - Write to GPIO3\n");
    printf_(" Select an option: \n");

    getUserInput(&new_gpio_write);
    putchar('\n');

    switch (new_gpio_write)
    {
    case 1:
        printf_("Insert value to be writen into all GPIOs, same value for all GPIOs: ");
        ret = getUserInput(&new_value);

        if (((new_value == GPIO_HIGH) || (new_value == GPIO_LOW)) && (ret == SUCCESS)) {
            new_value *= 0xF;
            ret_val = ad77681_gpio_write(device_adc, new_value, AD77681_ALL_GPIOS);
            printf_("\n Value %lu successully written to all GPOIs\n", new_value);
        } else
            printf_("\nInvalid value\n");
        break;
    case 2:
        printf_("Insert value to be written into GPIO0: ");
        ret = getUserInput(&new_value);

        if (((new_value == GPIO_HIGH) || (new_value == GPIO_LOW)) && (ret == SUCCESS)) {
            ret_val = ad77681_gpio_write(device_adc, new_value, AD77681_GPIO0);
            printf_("\n Value %lu successully written to GPIO0\n", new_value);
        } else
            printf_("\nInvalid value\n");
        break;
    case 3:
        printf_("Insert value to be written into GPIO1: ");
        ret = getUserInput(&new_value);

        if (((new_value == GPIO_HIGH) || (new_value == GPIO_LOW)) && (ret == SUCCESS)) {
            ret_val = ad77681_gpio_write(device_adc, new_value, AD77681_GPIO1);
            printf_("\n Value %lu successully written to GPIO1\n", new_value);
        } else
            printf_("\nInvalid value\n");
        break;
    case 4:
        printf_("Insert value to be written into GPIO2: ");
        ret = getUserInput(&new_value);

        if (((new_value == GPIO_HIGH) || (new_value == GPIO_LOW)) && (ret == SUCCESS)) {
            ret_val = ad77681_gpio_write(device_adc, new_value, AD77681_GPIO2);
            printf_("\n Value %lu successully written to GPIO2\n", new_value);
        } else
            printf_("\nInvalid value\n");
        break;
    case 5:
        printf_("Insert value to be written into GPIO3: ");
        ret = getUserInput(&new_value);

        if (((new_value == GPIO_HIGH) || (new_value == GPIO_LOW)) && (ret == SUCCESS)) {
            ret_val = ad77681_gpio_write(device_adc, new_value, AD77681_GPIO3);
            printf_("\n Value %lu successully written to GPIO3\n", new_value);
        } else
            printf_("\nInvalid value\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
}

/**
 * GPIO direction, part of the ADC_GPIO function
 *
 */
static void adc_GPIO_inout(void)
{
    uint32_t new_gpio_inout = 0, new_gpio_inout_set = 0;
    int32_t ret_val  __attribute__ ((unused));

    printf_(" Set GPIOs as input or output: \n");
    printf_("  1 - Set all GPIOs\n");
    printf_("  2 - Set GPIO0\n");
    printf_("  3 - Set GPIO1\n");
    printf_("  4 - Set GIPO2\n");
    printf_("  5 - Set GPIO3\n");
    printf_(" Select an option: \n");

    getUserInput(&new_gpio_inout);
    putchar('\n');

    switch (new_gpio_inout) {
    case 1:
        printf_("   1 - Set all GPIOS as inputs\n");
        printf_("   2 - Set all GPIOS as outputs\n");

        getUserInput(&new_gpio_inout_set);
        putchar('\n');

        if ((new_gpio_inout_set == 1) || (new_gpio_inout_set == 2)) {
            new_gpio_inout_set -= 1;
            new_gpio_inout_set *= 0xF;
            ret_val = ad77681_gpio_inout(device_adc, new_gpio_inout_set, AD77681_ALL_GPIOS);
            printf_("All GPIOs successfully set");
        } else
            printf_("\nInvalid value\n");
        break;
    case 2:
        printf_("   1 - Set GPIO0 as input\n");
        printf_("   2 - Set GPIO0 as output\n");

        getUserInput(&new_gpio_inout_set);
        putchar('\n');

        if ((new_gpio_inout_set == 1) || (new_gpio_inout_set == 2)) {
            new_gpio_inout_set -= 1;
            ret_val = ad77681_gpio_inout(device_adc, new_gpio_inout_set, AD77681_GPIO0);
            printf_("GPIO0 successfully set");
        } else
            printf_("\nInvalid value\n");
        break;
    case 3:
        printf_("   1 - Set GPIO1 as input\n");
        printf_("   2 - Set GPIO1 as output\n");

        getUserInput(&new_gpio_inout_set);
        putchar('\n');

        if ((new_gpio_inout_set == 1) || (new_gpio_inout_set == 2)) {
            new_gpio_inout_set -= 1;
            ret_val = ad77681_gpio_inout(device_adc, new_gpio_inout_set, AD77681_GPIO1);
            printf_("GPIO1 successfully set");
        } else
            printf_("\nInvalid value\n");
        break;
    case 4:
        printf_("   1 - Set GPIO2 as input\n");
        printf_("   2 - Set GPIO2 as output\n");

        getUserInput(&new_gpio_inout_set);
        putchar('\n');

        if ((new_gpio_inout_set == 1) || (new_gpio_inout_set == 2)) {
            new_gpio_inout_set -= 1;
            ret_val = ad77681_gpio_inout(device_adc, new_gpio_inout_set, AD77681_GPIO2);
            printf_("GPIO2 successfully set");
        } else
            printf_("\nInvalid value\n");
        break;
    case 5:
        printf_("   1 - Set GPIO3 as input\n");
        printf_("   2 - Set GPIO3 as output\n");

        getUserInput(&new_gpio_inout_set);
        putchar('\n');

        if ((new_gpio_inout_set == 1) || (new_gpio_inout_set == 2)) {
            new_gpio_inout_set -= 1;
            ret_val = ad77681_gpio_inout(device_adc, new_gpio_inout_set, AD77681_GPIO3);
            printf_("GPIO3 successfully set");
        } else
            printf_("\nInvalid value\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
}

/**
 * Additional GPIO settings, part of the ADC_GPIO function
 *
 */
static void adc_GPIO_settings(void)
{
    uint32_t new_gpio_settings = 0;

    printf_(" GPIO Settings: \n");
    printf_("  1 - Enable  all GPIOs (Global enble)\n");
    printf_("  2 - Disable all GPIOs (Global disable)\n");
    printf_("  3 - Set GPIO0 - GPIO2 as open drain\n");
    printf_("  4 - Set GPIO0 - GPIO2 as strong driver\n");
    printf_(" Select an option: \n");

    getUserInput(&new_gpio_settings);
    putchar('\n');

    switch (new_gpio_settings) {
    case 1:
        ad77681_global_gpio(device_adc, AD77681_GLOBAL_GPIO_ENABLE);
        printf_(" Global GPIO enalbe bit enabled");
        break;
    case 2:
        ad77681_global_gpio(device_adc, AD77681_GLOBAL_GPIO_DISABLE);
        printf_(" Global GPIO enalbe bit disabled");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
}

/**
 * Read ADC status from status registers
 *
 */
 static void menu_12_read_master_status(void)
 {
//    uint8_t reg_read_buf[3];
//    char binary_number[8];
    char ok[3] = { 'O', 'K' }, fault[6] = { 'F', 'A', 'U', 'L', 'T' };

    ad77681_status(device_adc, current_status);
    putchar('\n');
    printf_("== MASTER STATUS REGISER\n");
    printf_("Master error:          %s\n", ((current_status->master_error == 0) ? (ok) : (fault)));
    printf_("ADC error:             %s\n", ((current_status->adc_error == 0) ? (ok) : (fault)));
    printf_("Dig error:             %s\n", ((current_status->dig_error == 0) ? (ok) : (fault)));
    printf_("Ext. clock:            %s\n", ((current_status->adc_err_ext_clk_qual == 0) ? (ok) : (fault)));
    printf_("Filter saturated:      %s\n", ((current_status->adc_filt_saturated == 0) ? (ok) : (fault)));
    printf_("Filter not settled:    %s\n", ((current_status->adc_filt_not_settled == 0) ? (ok) : (fault)));
    printf_("SPI error:             %s\n", ((current_status->spi_error == 0) ? (ok) : (fault)));
    printf_("POR Flag:              %s\n", ((current_status->por_flag == 0) ? (ok) : (fault)));

    if (current_status->spi_error == 1) {
        printf_("\n== SPI DIAG STATUS REGISER\n");
        printf_("SPI ignore error:      %s\n", ((current_status->spi_ignore == 0) ? (ok) : (fault)));
        printf_("SPI clock count error: %s\n", ((current_status->spi_clock_count == 0) ? (ok) : (fault)));
        printf_("SPI read error:        %s\n", ((current_status->spi_read_error == 0) ? (ok) : (fault)));
        printf_("SPI write error:       %s\n", ((current_status->spi_write_error == 0) ? (ok) : (fault)));
        printf_("SPI CRC error:         %s\n", ((current_status->spi_crc_error == 0) ? (ok) : (fault)));
    }
    if (current_status->adc_error == 1) {
        printf_("\n== ADC DIAG STATUS REGISER\n");
        printf_("DLDO PSM error:        %s\n", ((current_status->dldo_psm_error == 0) ? (ok) : (fault)));
        printf_("ALDO PSM error:        %s\n", ((current_status->aldo_psm_error == 0) ? (ok) : (fault)));
        printf_("REF DET error:         %s\n", ((current_status->ref_det_error == 0) ? (ok) : (fault)));
        printf_("FILT SAT error:        %s\n", ((current_status->filt_sat_error == 0) ? (ok) : (fault)));
        printf_("FILT NOT SET error:    %s\n", ((current_status->filt_not_set_error == 0) ? (ok) : (fault)));
        printf_("EXT CLK QUAL error:    %s\n", ((current_status->ext_clk_qual_error == 0) ? (ok) : (fault)));
    }
    if (current_status->dig_error == 1) {
        printf_("\n== DIGITAL DIAG STATUS REGISER\n");
        printf_("Memory map CRC error:  %s\n", ((current_status->memoy_map_crc_error == 0) ? (ok) : (fault)));
        printf_("RAM CRC error:         %s\n", ((current_status->ram_crc_error == 0) ? (ok) : (fault)));
        printf_("FUSE CRC error:        %s\n", ((current_status->fuse_crc_error == 0) ? (ok) : (fault)));
    }
    putchar('\n');
    print_prompt();
}

/**
 * Set Vref anc MCLK as "exteranl" values, depending on you setup
 *
 */
 static void menu_13_mclk_vref(void)
 {
    uint32_t input = 0, new_settings = 0;
    int32_t ret;

    printf_(" Set Vref and Mclk: \n");
    printf_("  1 - Change Vref\n");
    printf_("  2 - Change MCLK\n");
    printf_(" Select an option: \n");

    getUserInput(&new_settings);
    putchar('\n');

    switch (new_settings) {
    case 1:
        printf_(" Change Vref from %d mV to [mV]: ", device_adc->vref);       // Vref change
        ret = getUserInput(&input);

        if ((input >= 1000) && (input <= 5000) && (ret == SUCCESS)) {
            printf_("\n New Vref value is %lu mV", input);
            device_adc->vref = input;

            #ifdef CN0540_ADI_FFT_H_
            // Update the Vref, Mclk and sampling rate
            update_FFT_enviroment(device_adc->vref, device_adc->mclk, device_adc->sample_rate, FFT_data);
            #endif //CN0540_ADI_FFT_H_
        } else
            printf_(" Invalid option\n");
        putchar('\n');
        break;
    case 2:
        printf_(" Change MCLK from %d kHz to [kHz]: ", device_adc->mclk);     // MCLK change
        ret = getUserInput(&input);

        if ((input >= 10000) && (input <= 50000) && (ret == SUCCESS)){
            printf_("\n New MCLK value is %lu kHz\n", input);
            device_adc->vref = input;
            ad77681_update_sample_rate(device_adc);                             // Update the sample rate after changinig the MCLK

            #ifdef CN0540_ADI_FFT_H_
            // Update the Vref, Mclk and sampling rate
            update_FFT_enviroment(device_adc->vref, device_adc->mclk, device_adc->sample_rate, FFT_data);
            #endif //CN0540_ADI_FFT_H_
        } else
            printf_(" Invalid option\n");
        putchar('\n');
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Print measured data and transfered to voltage
 *
 */
 static void menu_14_print_measured_data(void)
 {
    double voltage;
    int32_t shifted_data;
    uint16_t i;
//    char buf[15];

    if (measured_data.finish) {
        // Printing Voltage
        printf_("\n\nVoltage\n");
        for ( i = 0; i < measured_data.samples; i++) {
            ad77681_data_to_voltage(device_adc, &measured_data.raw_data[i], &voltage);
            printf_("%.9f \n",voltage);
        }
        // Printing Codes
        printf_("\n\nCodes\n");
        for(i = 0 ; i < measured_data.samples ; i++) {
            if (measured_data.raw_data[i] & 0x800000)
                shifted_data = (int32_t)((0xFF << 24) | measured_data.raw_data[i]);
            else
                shifted_data = (int32_t)((0x00 << 24) | measured_data.raw_data[i]);
            printf_("%ld\n", shifted_data + AD7768_HALF_SCALE);
        }
        // Printing Raw Date
        printf_("\n\nRaw data\n");
        for (i = 0; i < measured_data.samples; i++)
            printf_("%lu\n", measured_data.raw_data[i]);
        // Set  measured_data.finish to false after Printing
        measured_data.finish = false;
    } else
        printf_("Data not prepared\n");
    print_prompt();
}

/**
 * Set data output mode
 *
 */
static void menu_15_set_adc_data_output_mode(void)
{
    uint32_t new_data_mode = 0, new_length = 0, new_status = 0, new_crc = 0; //, ret;

    printf_(" ADC data outpup modes: \n");
    printf_("  1 - Continuous: waiting for DRDY\n");
    printf_("  2 - Continuous one shot: waiting for SYNC_IN\n");
    printf_("  3 - Single-conversion standby\n");
    printf_("  4 - Periodic standby\n");
    printf_("  5 - Standby mode\n");
    printf_("  6 - 16bit or 24bit data format\n");
    printf_("  7 - Status bit output\n");
    printf_("  8 - Switch form diag mode to measure\n");
    printf_("  9 - Switch form measure to diag mode\n");
    printf_(" 10 - Set CRC type\n");
    printf_(" Select an option: \n");

    getUserInput(&new_data_mode);
    putchar('\n');

    switch (new_data_mode) {
    case 1:
        ad77681_set_conv_mode(device_adc, AD77681_CONV_CONTINUOUS, device_adc->diag_mux_sel, device_adc->conv_diag_sel);// DIAG MUX NOT SELECTED
        printf_(" Continuous mode set\n");
        break;
    case 2:
        ad77681_set_conv_mode(device_adc, AD77681_CONV_ONE_SHOT, device_adc->diag_mux_sel, device_adc->conv_diag_sel);
        printf_(" Continuous one shot conversion set\n");
        break;
    case 3:
        ad77681_set_conv_mode(device_adc, AD77681_CONV_SINGLE, device_adc->diag_mux_sel, device_adc->conv_diag_sel);
        printf_(" Single conversion standby mode set\n");
        break;
    case 4:
        ad77681_set_conv_mode(device_adc, AD77681_CONV_PERIODIC, device_adc->diag_mux_sel, device_adc->conv_diag_sel);
        printf_(" Periodiec standby mode set\n");
        break;
    case 5:
        ad77681_set_conv_mode(device_adc, AD77681_CONV_STANDBY, device_adc->diag_mux_sel, device_adc->conv_diag_sel);
        printf_(" Standby mode set\n");
        break;
    case 6:
        printf_(" Conversion length select: \n");
        printf_("  1 - 24bit length\n");
        printf_("  2 - 16bit length\n");

        getUserInput(&new_length);
        putchar('\n');

        switch (new_length) {
        case 1:
            ad77681_set_convlen(device_adc, AD77681_CONV_24BIT);
            printf_(" 24bit data output format selected\n");
            break;
        case 2:
            ad77681_set_convlen(device_adc, AD77681_CONV_16BIT);
            printf_(" 16bit data output format selected\n");
            break;
        default:
            printf_(" Invalid option\n");
            break;
        }
        break;
    case 7:
        printf_(" Status bit output: \n");
        printf_("  1 - Enable status bit after each ADC conversion\n");
        printf_("  2 - Disable status bit after each ADC conversion\n");

        getUserInput(&new_status);
        putchar('\n');

        switch (new_status) {
        case 1:
            ad77681_set_status_bit(device_adc, true);
            printf_(" Status bit enabled\n");
            break;
        case 2:
            ad77681_set_status_bit(device_adc, false);
            printf_(" Status bit disabled\n");
            break;
        default:
            printf_(" Invalid option\n");
            break;
        }
        break;
    case 8:
        ad77681_set_conv_mode(device_adc, device_adc->conv_mode, device_adc->diag_mux_sel, false);// DIAG MUX NOT SELECTED
        printf_(" Measure mode selected\n");
        break;
    case 9:
        ad77681_set_conv_mode(device_adc, device_adc->conv_mode, device_adc->diag_mux_sel, true); // DIAG MUX SELECTED
        printf_(" Diagnostic mode selected\n");
        break;
    case 10:
        printf_(" CRC settings \n");
        printf_("  1 - Disable CRC\n");
        printf_("  2 - 8-bit polynomial CRC\n");
        printf_("  3 - XOR based CRC\n");

        getUserInput(&new_crc);
        putchar('\n');

        switch (new_crc) {
        case 1:
            ad77681_set_crc_sel(device_adc, AD77681_NO_CRC);
            printf_(" CRC disabled\n");
            break;
        case 2:
            ad77681_set_crc_sel(device_adc, AD77681_CRC);
            printf_("  8-bit polynomial CRC method selected\n");
            break;
        case 3:
            ad77681_set_crc_sel(device_adc, AD77681_XOR);
            printf_("  XOR based CRC method selected\n");
            break;
        default:
            printf_(" Invalid option\n");
            break;
        }
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Set diagnostic mode
 *
 */
static void menu_16_set_adc_diagnostic_mode(void)
{
    uint32_t new_diag_mode = 0;

    printf_(" ADC diagnostic modes: \n");
    printf_("  1 - Internal temperature sensor\n");
    printf_("  2 - AIN shorted\n");
    printf_("  3 - Positive full-scale\n");
    printf_("  4 - Negative full-scale\n");
    printf_(" Select an option: \n");

    getUserInput(&new_diag_mode);
    putchar('\n');

    switch (new_diag_mode) {
    case 1:
        ad77681_set_conv_mode(device_adc, device_adc->conv_mode, AD77681_TEMP_SENSOR, true);
        printf_(" Diagnostic mode: Internal temperature sensor selected\n");
        break;
    case 2:
        ad77681_set_conv_mode(device_adc, device_adc->conv_mode, AD77681_AIN_SHORT, true);
        printf_(" Diagnostic mode: AIN shorted selected\n");
        break;
    case 3:
        ad77681_set_conv_mode(device_adc, device_adc->conv_mode, AD77681_POSITIVE_FS, true);
        printf_(" Diagnostic mode: Positive full-scale selected\n");
        break;
    case 4:
        ad77681_set_conv_mode(device_adc, device_adc->conv_mode, AD77681_NEGATIVE_FS, true);
        printf_(" Diagnostic mode: Negative full-scale selected\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Do the FFT
 *
 */
static void menu_17_do_the_fft(void)
{
    printf_(" FFT in progress...\n");
    measured_data.samples = FFT_data->fft_length * 2;
    measured_data.samples = 4096;
    measured_data.finish = false;
    measured_data.count = 0;
    printf_("Sampling....\n");
    cont_sampling();
    perform_FFT(measured_data.raw_data, FFT_data, FFT_meas, device_adc->sample_rate);
    printf_(" FFT Done!\n");
    measured_data.finish = false;

    printf_("\n THD:\t\t%.3f dB", FFT_meas->THD);
    printf_("\n SNR:\t\t%.3f dB", FFT_meas->SNR);
    printf_("\n DR:\t\t%.3f dB", FFT_meas->DR);
    printf_("\n Fundamental:\t%.3f dBFS", FFT_meas->harmonics_mag_dbfs[0]);
    printf_("\n Fundamental:\t%.3f Hz", FFT_meas->harmonics_freq[0]*FFT_data->bin_width);
    printf_("\n RMS noise:\t%.6f uV", FFT_meas->RMS_noise * 1000000.0);
    printf_("\n LSB noise:\t%.3f", FFT_meas->transition_noise_LSB);

    print_prompt();
}

/**
 * Setting of the FFT module
 *
 */
static void menu_18_fft_settings(void)
{
    uint32_t new_menu_select, new_window, new_sample_count;

    printf_(" FFT settings: \n");
    printf_("  1 - Set window type\n");
    printf_("  2 - Set sample count\n");
    printf_("  3 - Print FFT plot\n");
    printf_(" Select an option: \n\n");

    getUserInput(&new_menu_select);

    switch (new_menu_select) {
    case 1:
        printf_(" Choose window type:\n");
        printf_("  1 - 7-term Blackman-Harris\n");
        printf_("  2 - Rectangular\n");

        getUserInput(&new_window);

        switch (new_window) {
        case 1:
            printf_("  7-7-term Blackman-Harris window selected\n");
            FFT_data->window = BLACKMAN_HARRIS_7TERM;
            break;
        case 2:
            printf_("  Rectalngular window selected\n");
            FFT_data->window = RECTANGULAR;
            break;
        default:
            printf_(" Invalid option\n");
            break;
        }
        break;
    case 2:
        printf_(" Set sample count:\n");
        printf_("  1 - 4096 samples\n");
        printf_("  2 - 1024 samples\n");
        printf_("  3 - 256  samples\n");
        printf_("  4 - 64   samples\n");
        printf_("  5 - 16   samples\n");

        getUserInput(&new_sample_count);

        switch (new_sample_count) {
        case 1:
            printf_(" 4096 samples selected\n");
            FFT_init(4096, FFT_data);   // Update the FFT module with a new sample count
            break;
        case 2:
            printf_(" 1024 samples selected\n");
            FFT_init(1024, FFT_data);
            break;
        case 3:
            printf_(" 256 samples selected\n");
            FFT_init(256, FFT_data);
            break;
        case 4:
            printf_(" 64 samples selected\n");
            FFT_init(64, FFT_data);
            break;
        case 5:
            printf_(" 16 samples selected\n");
            FFT_init(16, FFT_data);
            break;
        default:
            printf_(" Invalid option\n");
            break;
        }
        break;
    case 3:
        if (FFT_data->fft_done == true) {
            printf_(" Printing FFT plot in dB:\n");

            for (uint16_t i = 0; i < FFT_data->fft_length; i++)
                printf_("%.4f\n", FFT_data->fft_dB[i]);
        }
        else
            printf_(" Data not prepared\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Set Gains and Offsets
 *
 */
static void menu_19_gains_offsets(void)
{
    uint32_t gain_offset, new_menu_select;
    int32_t ret;

    printf_(" Gains and Offsets settings: \n");
    printf_("  1 - Set gain\n");
    printf_("  2 - Set offset\n");
    printf_(" Select an option: \n");

    getUserInput(&new_menu_select);

    switch (new_menu_select) {
    case 1:
        printf_(" Insert new Gain value in decimal form\n");
        ret = getUserInput(&gain_offset);

        if ((gain_offset <= 0xFFFFFF) && (ret == SUCCESS)) {
            ad77681_apply_gain(device_adc, gain_offset);
            printf_(" Value %lu has been successfully inserted to the Gain register\n", gain_offset);
        } else
            printf_(" Invalid value\n");
        break;
    case 2:
        printf_(" Insert new Offset value in decimal form\n");
        ret = getUserInput(&gain_offset);
        if ((gain_offset <= 0xFFFFFF) && (ret == SUCCESS)) {
            ad77681_apply_offset(device_adc, gain_offset);
            printf_(" Value %lu has been successfully inserted to the Offset register\n", gain_offset);
        } else
            printf_(" Invalid value\n");
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Chceck read and write functionaity by writing to and reading from scratchpad register
 *
 */
static void menu_20_check_scratchpad(void)
{
    int32_t ret;
    uint32_t ret_val;
    uint32_t new_menu_select;
    uint8_t chceck_sequence;

    printf_(" Scratchpad check\n");
    printf_("  Insert 8bit number for scratchpad check: \n");

    ret = getUserInput(&new_menu_select);

//    if ((new_menu_select <= 0xFF) && (new_menu_select >= 0) && (ret == SUCCESS)) {
    if ((new_menu_select <= 0xFF) && (ret == SUCCESS)) {
        chceck_sequence = (uint8_t)(new_menu_select);
        ret_val = ad77681_scratchpad(device_adc, &chceck_sequence);
        printf_("  Insered sequence:  %lu\n  Returned sequence: %d\n", new_menu_select, chceck_sequence);
        if (ret_val == SUCCESS)
            printf_("  SUCCESS!\n");
        else
            printf_("  FAILURE!\n");
    } else
        printf_("  Invalid value\n");
    print_prompt();
}

/**
 * Start with the piezo accelerometer offset compensation
 * The offset compenzation process uses a successive approximation model
 * There is lot of averaging going on, because of quite noisy piezo accelerometer
 * It will take some time
 */
static void menu_21_piezo_offset(void)
{
    uint8_t ltc2606_res = 16;
    uint32_t dac_code = 0;
    uint32_t dac_code_arr[16];
    double mean_voltage = 0.0, min_voltage;
    double mean_voltage_arr[16];
    int8_t sar_loop, min_find, min_index;
    uint16_t SINC3_odr;

    // Low power mode and MCLK/16
    ad77681_set_power_mode(device_adc, AD77681_ECO);
    ad77681_set_mclk_div(device_adc, AD77681_MCLK_DIV_16);

    // 4SPS = 7999 SINC3, 10SPS = 3199 SINC3, 50SPS = 639 SINC3
    ad77681_SINC3_ODR(device_adc, &SINC3_odr, 4);
    // Set the oversamplig ratio to high value, to extract DC
    ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx32, AD77681_SINC3, SINC3_odr);

    /*
	On power-up, the user must apply a soft or hard reset to the device when using either control mode.
	A SYNC_IN pulse is also recommended after the reset or after any change to the device configuration.

	SYNC_IN receives the synchronous signal from SYNC_OUT or from the main controller.
	SYNC_IN enables synchronization of multiple AD7768-1 devices that require simultaneous sampling.
	A SYNC_IN pulse is always required when the device configuration changes in any way
	(for example, if the filter decimation rate changes).
	Apply SYNC_IN pulses directly after a DRDY pulse occurs.
	*/
	ad77681_initiate_sync(device_adc);

    // successive approximation algorithm
    printf_("\nInitialize SAR loop (DAC MSB set to high)\n");
    // Set DAC code to half scale
    dac_code = (1 << (ltc2606_res - 1 ));
    // Update output of the DAC
    ltc26x6_write_code(device_dac, write_update_command, dac_code);
    // Wait for DAC output to settle
    wait_ms(500);

    // Set desired number of samples for every iteration
    measured_data.samples = 100;
    measured_data.finish = false;
    measured_data.count = 0;
    // Take X number of samples
    cont_sampling();
    // Get the mean voltage of taken samples stroed in the measured_data strucutre
    get_mean_voltage(&measured_data, &mean_voltage);
    // Print the mean ADC read voltage for a given DAC code
    printf_("DAC code:%lx\t\tMean Voltage: %.6f\n", dac_code, mean_voltage);
    // Store the initial DAC code in the array
    dac_code_arr[ltc2606_res - 1] = dac_code;
    // Store the initial mean voltage in the array
    mean_voltage_arr[ltc2606_res - 1] = mean_voltage;

    for ( sar_loop = ltc2606_res - 1; sar_loop > 0; sar_loop--) {
        // Check if the mean voltage is positive or negative
        if (mean_voltage > 0) {
            dac_code = dac_code + (1 << (sar_loop - 1));
            printf_("UP\n\n");
        } else {
            dac_code = dac_code - (1 << (sar_loop)) + (1 << (sar_loop-1));
            printf_("DOWN\n\n");
        }
        // Print loop coard
        printf_("SAR loop #: %d\n",sar_loop);
        // Update output of the DAC
        ltc26x6_write_code(device_dac, write_update_command, dac_code);
        // Wait for DAC output to settle
        wait_ms(500);
        // Clear data finish flag
        measured_data.finish = false;
        measured_data.count = 0;
        // Take X number of samples
        cont_sampling();
        // Get the mean voltage of taken samples stroed in the measured_data strucutre
        get_mean_voltage(&measured_data, &mean_voltage);
        printf_("DAC code:%lx\t\tMean Voltage: %.6f\n", dac_code, mean_voltage);
        dac_code_arr[sar_loop - 1] = dac_code;
        mean_voltage_arr[sar_loop - 1] = mean_voltage;
    }
//    min_voltage = abs(mean_voltage_arr[0]);
    min_voltage = fabs(mean_voltage_arr[0]);
    for (min_find = 0;  min_find < 16; min_find++) {
        if (min_voltage > fabs(mean_voltage_arr[min_find])) {
            min_voltage = fabs(mean_voltage_arr[min_find]);
            min_index = min_find;
        }
    }
    ltc26x6_write_code(device_dac, write_update_command, dac_code_arr[min_index]);
    // Wait for DAC output to settle
    wait_ms(500);
    // Print the final DAC code
    printf_("\nFinal DAC code set to:%lx\t\tFinal Mean Voltage: %.6f\n", dac_code_arr[min_index], mean_voltage_arr[min_index]);
    // Set to original filter
    ad77681_set_filter_type(device_adc, AD77681_SINC5_FIR_DECx32, AD77681_FIR, 0);
    ad77681_update_sample_rate(device_adc);
    printf_("\nOffset compenzation done!\n");
    print_prompt();
}

/**
 * Get mean from sampled data
 * @param mean_voltage      Mean Voltage
 * @param measured_data     The structure carying measured data
 */
static void get_mean_voltage(struct adc_data *measured_data, double *mean_voltage)
{
//    int32_t shifted_data;
    double sum = 0, voltage = 0;
    uint16_t i;

    for ( i = 0; i < measured_data->samples; i++) {
        ad77681_data_to_voltage(device_adc, &measured_data->raw_data[i], &voltage);
        sum += voltage;
    }
    *mean_voltage = (double)(sum / (double)(measured_data->samples));
}

/**
 * Set output of the on-board DAC in codes or in voltage
 *
 */
static void menu_22_set_DAC_output(void)
{
    int16_t dac_status = SUCCESS;
    uint16_t  code ;
    uint32_t new_menu_select, new_dac;
    float dac_voltage;
    // Gain factor of the on-board DAC buffer, to have full 5V range(ADA4807-1ARJZ)
    // Non-inverting op-amp resistor ratio => 1 + (2.7 k ohm / 2.7 k ohm)
    float buffer_gain =  2;

    printf_(" Set DAC output: \n");
    printf_("  1 - Voltage\n");
    printf_("  2 - Codes\n");
    printf_(" Select an option: \n");

    getUserInput(&new_menu_select);

    switch (new_menu_select) {
    case 1:
        printf_(" Set DAC output in mV: ");
        getUserInput(&new_dac);

        dac_voltage = ((float)(new_dac) / 1000.0) / buffer_gain;
        ltc26x6_voltage_to_code(device_dac, dac_voltage, &code);
        ltc26x6_write_code(device_dac, write_update_command, code);
        if (dac_status == SUCCESS)
            printf_("%.3f V at Shift output\n\n", dac_voltage * buffer_gain);
        else if (dac_status == LTC26X6_CODE_OVERFLOW)
            printf_("%.3f V at Shift output, OVERFLOW!\n\n", dac_voltage * buffer_gain);
        else if (dac_status == LTC26X6_CODE_UNDERFLOW)
            printf_("%.3f V at Shift output, UNDERFLOW!\n\n", dac_voltage * buffer_gain);
        break;
    case 2:
        printf_(" Set DAC codes in decimal form: ");
        getUserInput(&new_dac);
        ltc26x6_write_code(device_dac, write_update_command, new_dac);
        printf_("%lx at DAC output\n\n", new_dac);
        break;
    default:
        printf_(" Invalid option\n");
        break;
    }
    print_prompt();
}

/**
 * Prints out an array in binary form
 *
 */
static void print_binary(uint8_t number, char *binary_number)
{
    for (int8_t i = 7; i >= 0; i--) {
        if (number & 1)
            binary_number[i] = '1';
        else
            binary_number[i] = '0';
        number >>= 1;
    }
}

/**
 * Setup SDP-K1 GPIOs
 *
 *
 */
static void sdpk1_gpio_setup(void) {
	// Enable DAC buffer & other buffer
//    buffer_en = GPIO_HIGH;
	HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);
	// Turn on onboard red LED
//    led_red = GPIO_HIGH;
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	// Turn on onboard blue LED
//    led_blue = GPIO_HIGH;
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
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
     ex: printf_("Wrong parameters value: file %s on line %d\r\n", file, line) */
	(void)file;
	(void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
