/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "api.hpp"
#include "bmi08x.h"
#include <stdio.h>
#include <cstring>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
//#define DUAL_CORE_BOOT_SYNC_SEQUENCE
//! NOT Using Both Cores for H7 Test

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* Definitions for testsTask */
osThreadId_t testsTaskHandle;
const osThreadAttr_t testsTask_attributes = {
  .name = "testsTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for imuDataTask */
osThreadId_t imuDataTaskHandle;
const osThreadAttr_t imuDataTask_attributes = {
  .name = "imuDataTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for stackMonitor */
osThreadId_t stackMonitorHandle;
const osThreadAttr_t stackMonitor_attributes = {
  .name = "stackMonitor",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UartPrintTask */
osThreadId_t UartPrintTaskHandle;
const osThreadAttr_t UartPrintTask_attributes = {
  .name = "UartPrintTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuDataQueue */
osMessageQueueId_t imuDataQueueHandle;
const osMessageQueueAttr_t imuDataQueue_attributes = {
  .name = "imuDataQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void StartTestsTask(void *argument);
void StartImuTask(void *argument);
void StartStackMonitor(void *argument);
void StartUartPrintTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
} cs_pin_t;

typedef struct {
	float out;
	float alpha;
} low_pass_filt_t;
void CS_low(cs_pin_t* intf_ptr);
void CS_high(cs_pin_t* intf_ptr);

int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void delay_us(uint32_t us, void *intf_ptr);
void DWT_Init();

void imu_init(struct bmi08_dev* rdev, bool useSPI);

void process_accel_measurements(const bmi08_sensor_data* dat_in, Vector<3>& dat_out);
void process_gryo_measurements(const bmi08_sensor_data* dat_in, Vector<3>& dat_out);

void low_pass_init(low_pass_filt_t* filt, float alpha);
void low_pass_setAlpha(low_pass_filt_t* filt, float alpha);
void low_pass(low_pass_filt_t* filt, float inp);

struct bmi08_dev rdev;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
	DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of imuDataQueue */
  imuDataQueueHandle = osMessageQueueNew (8, sizeof(Vector<3>), &imuDataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of testsTask */
  testsTaskHandle = osThreadNew(StartTestsTask, NULL, &testsTask_attributes);

  /* creation of imuDataTask */
  imuDataTaskHandle = osThreadNew(StartImuTask, NULL, &imuDataTask_attributes);

  /* creation of stackMonitor */
  stackMonitorHandle = osThreadNew(StartStackMonitor, NULL, &stackMonitor_attributes);

  /* creation of UartPrintTask */
  UartPrintTaskHandle = osThreadNew(StartUartPrintTask, NULL, &UartPrintTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10C0ECFF;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CS_low(cs_pin_t* intf_ptr) {
	HAL_GPIO_WritePin(intf_ptr->port, intf_ptr->pin, GPIO_PIN_RESET);
}
void CS_high(cs_pin_t* intf_ptr) {
	HAL_GPIO_WritePin(intf_ptr->port, intf_ptr->pin, GPIO_PIN_SET);
}
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*) intf_ptr;
    HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 50U);
    return 0;
}
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *(uint8_t*) intf_ptr;
    HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*) reg_data, len, 50U);
    return 0;
}
void delay_us(uint32_t us, void *intf_ptr) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}
void DWT_Init() {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
// Device address for I2C
uint8_t acc_dev_addr = BMI08_ACCEL_I2C_ADDR_SECONDARY;   // 0x19
uint8_t gyro_dev_addr = BMI08_GYRO_I2C_ADDR_SECONDARY;   // 0x69
void imu_init(struct bmi08_dev* rdev, bool useSPI) {
	rdev->delay_us = delay_us;
	rdev->variant = BMI088_VARIANT;
	if (useSPI) {
		// SPI dev setup
	} else {
		rdev->intf_ptr_accel = &acc_dev_addr;
		rdev->intf_ptr_gyro = &gyro_dev_addr;
		rdev->intf = BMI08_I2C_INTF;
		rdev->read = i2c_read;
		rdev->write = i2c_write;
	}
	bmi08a_init(rdev);
	bmi08g_init(rdev);
	HAL_Delay(50);

	// Wake accelerometer from suspend
	rdev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
	bmi08a_set_power_mode(rdev);
	HAL_Delay(50);

	// Set gyro to normal mode
	rdev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
	bmi08g_set_power_mode(rdev);
	HAL_Delay(50);

	rdev->accel_cfg.range = BMI088_ACCEL_RANGE_6G;
	rdev->gyro_cfg.range = BMI08_GYRO_RANGE_500_DPS;
	rdev->gyro_cfg.odr = BMI08_GYRO_BW_32_ODR_100_HZ;
	rdev->accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;
	rdev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
	bmi08a_set_meas_conf(rdev);
	bmi08g_set_meas_conf(rdev);
	HAL_Delay(50);
}
EKF::IMU::IMU_calibs_t calib_params;
void process_accel_measurements(const bmi08_sensor_data* dat_in, Vector<3>& dat_out) {
	float ax = (float)dat_in->x / 32768.0f * 6.0f * 9.81f;
	float az = (float)dat_in->z / 32768.0f * 6.0f * 9.81f;
	float ay = (float)dat_in->y / 32768.0f * 6.0f * 9.81f;
	dat_out.data[0]=ax; dat_out.data[1]=ay; dat_out.data[2]=az;
	EKF::IMU::correct_accel(dat_out.data, &calib_params);
}
void process_gryo_measurements(const bmi08_sensor_data* dat_in, Vector<3>& dat_out){
	float gx = (float)dat_in->x / 32768.0f * 500.0f / (180.0f / PI);
	float gy = (float)dat_in->y / 32768.0f * 500.0f / (180.0f / PI);
	float gz = (float)dat_in->z / 32768.0f * 500.0f / (180.0f / PI);
	dat_out.data[0]=gx; dat_out.data[1]=gy; dat_out.data[2]=gz;
	EKF::IMU::correct_gyro(dat_out.data, &calib_params);
}


void low_pass_init(low_pass_filt_t* filt, float alpha) {
	low_pass_setAlpha(filt, alpha);
	filt->out = 0.0f;
}
void low_pass_setAlpha(low_pass_filt_t* filt, float alpha) {
	if (alpha > 1.0f) {
		alpha = 1.0f;
	} else if (alpha < 0.0f) {
		alpha = 0.0f;
	}
	filt->alpha = alpha;
}
void low_pass(low_pass_filt_t* filt, float inp) {
	filt->out = filt->alpha*inp + (1-filt->alpha)*filt->out;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTestsTask */
/**
  * @brief  Function implementing the testsTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTestsTask */
void StartTestsTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	EKF::State_t state = {
			.position_x = 0.0f,
			.position_y = 0.0f,
			.velocity_u = 0.0f,
			.velocity_v = 0.0f,
			.angle = 3.14159265359f/2.0f,
			.angular_vel = 0
	};
	EKF::SquareMatrix<6> certainty = EKF::Zero<6,6>;
	uint32_t timestamp = osKernelGetTickCount();
	EKF::EK_filter filter(state, certainty, timestamp);
	EKF::IMU::IMU_variances_t variances_inp = {
			.variance_ax=2.5e-4,
			.variance_ay=2.5e-4,
			.variance_angular_rate=4e-4
	};
  /* Infinite loop */
  for(;;)
  {
	  Vector<3> imu_dat;
	  osMessageQueueGet(imuDataQueueHandle, &imu_dat, 0U, osWaitForever);
	  timestamp = osKernelGetTickCount();
	  EKF::IMU::predict(&filter, imu_dat(0,0), imu_dat(1,0), imu_dat(2,0), variances_inp, timestamp);
	  EKF::Pose_t pose = filter.get_pose();

	  char msg[64U];
	  sprintf(msg, "%02f,%02f,%02f,%i\r\n", pose.position_x, pose.position_y, pose.heading, timestamp);
	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000U);
    osDelay(0U);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the imuDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
	imu_init(&rdev, false);

	float correction_mat[9U] = {
		1.00473113f, -0.00660536f, -0.01280495f,
		-0.00660536f,  1.00394358f,  0.00342996f,
		-0.01280495f,  0.00342996f,  1.0051751f
	};
	float accel_biases[3U] = { 0.10940603f,  0.10872448f, -0.08515973f };
	float gyro_biases[3U] = { 0.0008648f,  -0.00058626f, -0.0014762f };
	calib_params = EKF::IMU::create_calib_params(correction_mat, accel_biases, gyro_biases);

	low_pass_filt_t ax_filt, ay_filt, az_filt;
	low_pass_filt_t gx_filt, gy_filt, gz_filt;
	low_pass_init(&ax_filt, 0.5);
	low_pass_init(&ay_filt, 0.5);
	low_pass_init(&az_filt, 0.5);
	low_pass_init(&gx_filt, 0.5);
	low_pass_init(&gy_filt, 0.5);
	low_pass_init(&gz_filt, 0.5);

	Vector<3> accel_vec, gyro_vec;
  /* Infinite loop */
  for(;;)
  {
	  struct bmi08_sensor_data accel_dat, gyro_dat;

	  bmi08a_get_data(&accel_dat, &rdev);
	  bmi08g_get_data(&gyro_dat, &rdev);

	  process_accel_measurements(&accel_dat, accel_vec);
	  process_gryo_measurements(&gyro_dat, gyro_vec);

	  low_pass(&ax_filt, accel_vec(0,0)); accel_vec(0,0)=ax_filt.out;
	  low_pass(&ay_filt, accel_vec(1,0)); accel_vec(1,0)=ay_filt.out;
	  low_pass(&az_filt, accel_vec(2,0)); accel_vec(2,0)=az_filt.out;
	  low_pass(&gx_filt, gyro_vec(0,0)); gyro_vec(0,0)=gx_filt.out;
	  low_pass(&gy_filt, gyro_vec(1,0)); gyro_vec(1,0)=gy_filt.out;
	  low_pass(&gz_filt, gyro_vec(2,0)); gyro_vec(2,0)=gz_filt.out;

	  Vector<3> useable_data = {
			  .data = {
				accel_vec(0,0),
				accel_vec(1,0),
				gyro_vec(2,0)
			  }
	  };

	  osMessageQueuePut(imuDataQueueHandle, &useable_data, 0U, osWaitForever);

	  /*
	  char msg[256U];
	  sprintf(msg, "%f,%f,%f,%f,%f,%f,%i\r\n", accel_vec(0,0),accel_vec(1,0),accel_vec(2,0),gyro_vec(0,0),gyro_vec(1,0),gyro_vec(2,0),osKernelGetTickCount());
	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), osWaitForever);*/

    osDelay(50U);
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartStackMonitor */
/**
* @brief Function implementing the stackMonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStackMonitor */
void StartStackMonitor(void *argument)
{
  /* USER CODE BEGIN StartStackMonitor */
  /* Infinite loop */
  for(;;)
  {
	  UBaseType_t tests_hwm = uxTaskGetStackHighWaterMark(
	      (TaskHandle_t)testsTaskHandle);
	  UBaseType_t imu_hwm = uxTaskGetStackHighWaterMark(
	      (TaskHandle_t)imuDataTaskHandle);
	  UBaseType_t monitor_hwm = uxTaskGetStackHighWaterMark(
	      (TaskHandle_t)stackMonitorHandle);

	  char msg[80];
	  snprintf(msg, sizeof(msg), "STK tst:%lu imu:%lu mon:%lu\r\n",
			   (unsigned long)tests_hwm,
			   (unsigned long)imu_hwm,
			   (unsigned long)monitor_hwm);
	  //HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000U);

	  osDelay(100U);
  }
  /* USER CODE END StartStackMonitor */
}

/* USER CODE BEGIN Header_StartUartPrintTask */
/**
* @brief Function implementing the UartPrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartPrintTask */
void StartUartPrintTask(void *argument)
{
  /* USER CODE BEGIN StartUartPrintTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUartPrintTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
