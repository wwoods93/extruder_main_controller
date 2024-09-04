///* USER CODE BEGIN Header */
/////////////////////////////////////////////////////////////////**
////////////////////////////////////////////////////////////////  ******************************************************************************
////////////////////////////////////////////////////////////////  * @file           : main.c
////////////////////////////////////////////////////////////////  * @brief          : Main program body
////////////////////////////////////////////////////////////////  ******************************************************************************
////////////////////////////////////////////////////////////////  * @attention
////////////////////////////////////////////////////////////////  *
////////////////////////////////////////////////////////////////  * Copyright (c) 2022 STMicroelectronics.
////////////////////////////////////////////////////////////////  * All rights reserved.
////////////////////////////////////////////////////////////////  *
////////////////////////////////////////////////////////////////  * This software is licensed under terms that can be found in the LICENSE file
////////////////////////////////////////////////////////////////  * in the root directory of this software component.
////////////////////////////////////////////////////////////////  * If no LICENSE file comes with this software, it is provided AS-IS.
////////////////////////////////////////////////////////////////  *
////////////////////////////////////////////////////////////////  ******************************************************************************
////////////////////////////////////////////////////////////////  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "cmsis_os.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
////////////////////////////////////////////////////////////////
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
////////////////////////////////////////////////////////////////
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
////////////////////////////////////////////////////////////////
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
// CAN_HandleTypeDef hcan1;
//
//I2C_HandleTypeDef hi2c1;
//I2C_HandleTypeDef hi2c2;
//
//RTC_HandleTypeDef hrtc;
//
//SPI_HandleTypeDef hspi2;
//
//TIM_HandleTypeDef htim1;
//TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim6;
//TIM_HandleTypeDef htim7;
//TIM_HandleTypeDef htim10;
//TIM_HandleTypeDef htim13;
//TIM_HandleTypeDef htim14;
//
//UART_HandleTypeDef huart2;
//
//WWDG_HandleTypeDef hwwdg;
//
///* Definitions for initialization_task */
//osThreadId_t initialization_taskHandle;
//const osThreadAttr_t initialization_task_attributes = {
//  .name = "initialization_task",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for preparation_process_task */
//osThreadId_t preparation_process_taskHandle;
//const osThreadAttr_t preparation_process_task_attributes = {
//  .name = "preparation_process_task",
//  .stack_size = 320 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for extrusion_process_task */
//osThreadId_t extrusion_process_taskHandle;
//const osThreadAttr_t extrusion_process_task_attributes = {
//  .name = "extrusion_process_task",
//  .stack_size = 320 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for spooling_process_task */
//osThreadId_t spooling_process_taskHandle;
//const osThreadAttr_t spooling_process_task_attributes = {
//  .name = "spooling_process_task",
//  .stack_size = 320 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for comms_handler_task */
//osThreadId_t comms_handler_taskHandle;
//const osThreadAttr_t comms_handler_task_attributes = {
//  .name = "comms_handler_task",
//  .stack_size = 512 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for heartbeat_task */
//osThreadId_t heartbeat_taskHandle;
//const osThreadAttr_t heartbeat_task_attributes = {
//  .name = "heartbeat_task",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
///* Definitions for comms_handler_tick */
//osTimerId_t comms_handler_tickHandle;
//const osTimerAttr_t comms_handler_tick_attributes = {
//  .name = "comms_handler_tick"
//};
///* Definitions for spi_tx_data_buffer_mutex */
//osMutexId_t spi_tx_data_buffer_mutexHandle;
//const osMutexAttr_t spi_tx_data_buffer_mutex_attributes = {
//  .name = "spi_tx_data_buffer_mutex"
//};
///* Definitions for spi_rx_data_buffer_mutex */
//osMutexId_t spi_rx_data_buffer_mutexHandle;
//const osMutexAttr_t spi_rx_data_buffer_mutex_attributes = {
//  .name = "spi_rx_data_buffer_mutex"
//};
///* Definitions for i2c_tx_data_buffer_mutex */
//osMutexId_t i2c_tx_data_buffer_mutexHandle;
//const osMutexAttr_t i2c_tx_data_buffer_mutex_attributes = {
//  .name = "i2c_tx_data_buffer_mutex"
//};
///* Definitions for i2c_rx_data_buffer_mutex */
//osMutexId_t i2c_rx_data_buffer_mutexHandle;
//const osMutexAttr_t i2c_rx_data_buffer_mutex_attributes = {
//  .name = "i2c_rx_data_buffer_mutex"
//};
///* USER CODE BEGIN PV */
////////////////////////////////////////////////////////////////
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_CAN1_Init(void);
//static void MX_I2C2_Init(void);
//static void MX_SPI2_Init(void);
//static void MX_TIM6_Init(void);
//static void MX_TIM7_Init(void);
//static void MX_TIM10_Init(void);
//static void MX_TIM13_Init(void);
//static void MX_TIM14_Init(void);
//static void MX_WWDG_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_I2C1_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_RTC_Init(void);
//void start_initialization_task(void *argument);
//void start_preparation_process_task(void *argument);
//void start_extrusion_process_task(void *argument);
//void start_spooling_process_task(void *argument);
//void start_comms_updater_task(void *argument);
//void start_heartbeat_task(void *argument);
//void comms_handler_tick_callback(void *argument);
//
///* USER CODE BEGIN PFP */
////////////////////////////////////////////////////////////////
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
////////////////////////////////////////////////////////////////
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
////////////////////////////////////////////////////////////////
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
////////////////////////////////////////////////////////////////
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_CAN1_Init();
//  MX_I2C2_Init();
//  MX_SPI2_Init();
//  MX_TIM6_Init();
//  MX_TIM7_Init();
//  MX_TIM10_Init();
//  MX_TIM13_Init();
//  MX_TIM14_Init();
//  MX_WWDG_Init();
//  MX_USART2_UART_Init();
//  MX_I2C1_Init();
//  MX_TIM1_Init();
//  MX_TIM2_Init();
//  MX_RTC_Init();
//  /* USER CODE BEGIN 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END 2 */
//
//  /* Init scheduler */
//  osKernelInitialize();
//  /* Create the mutex(es) */
//  /* creation of spi_tx_data_buffer_mutex */
//  spi_tx_data_buffer_mutexHandle = osMutexNew(&spi_tx_data_buffer_mutex_attributes);
//
//  /* creation of spi_rx_data_buffer_mutex */
//  spi_rx_data_buffer_mutexHandle = osMutexNew(&spi_rx_data_buffer_mutex_attributes);
//
//  /* creation of i2c_tx_data_buffer_mutex */
//  i2c_tx_data_buffer_mutexHandle = osMutexNew(&i2c_tx_data_buffer_mutex_attributes);
//
//  /* creation of i2c_rx_data_buffer_mutex */
//  i2c_rx_data_buffer_mutexHandle = osMutexNew(&i2c_rx_data_buffer_mutex_attributes);
//
//  /* USER CODE BEGIN RTOS_MUTEX */
////////////////////////////////////////////////////////////////  /* add mutexes, ... */
//  /* USER CODE END RTOS_MUTEX */
//
//  /* USER CODE BEGIN RTOS_SEMAPHORES */
////////////////////////////////////////////////////////////////  /* add semaphores, ... */
//  /* USER CODE END RTOS_SEMAPHORES */
//
//  /* Create the timer(s) */
//  /* creation of comms_handler_tick */
//  comms_handler_tickHandle = osTimerNew(comms_handler_tick_callback, osTimerPeriodic, NULL, &comms_handler_tick_attributes);
//
//  /* USER CODE BEGIN RTOS_TIMERS */
////////////////////////////////////////////////////////////////  /* start timers, add new ones, ... */
//  /* USER CODE END RTOS_TIMERS */
//
//  /* USER CODE BEGIN RTOS_QUEUES */
////////////////////////////////////////////////////////////////  /* add queues, ... */
//  /* USER CODE END RTOS_QUEUES */
//
//  /* Create the thread(s) */
//  /* creation of initialization_task */
//  initialization_taskHandle = osThreadNew(start_initialization_task, NULL, &initialization_task_attributes);
//
//  /* creation of preparation_process_task */
//  preparation_process_taskHandle = osThreadNew(start_preparation_process_task, NULL, &preparation_process_task_attributes);
//
//  /* creation of extrusion_process_task */
//  extrusion_process_taskHandle = osThreadNew(start_extrusion_process_task, NULL, &extrusion_process_task_attributes);
//
//  /* creation of spooling_process_task */
//  spooling_process_taskHandle = osThreadNew(start_spooling_process_task, NULL, &spooling_process_task_attributes);
//
//  /* creation of comms_handler_task */
//  comms_handler_taskHandle = osThreadNew(start_comms_updater_task, NULL, &comms_handler_task_attributes);
//
//  /* creation of heartbeat_task */
//  heartbeat_taskHandle = osThreadNew(start_heartbeat_task, NULL, &heartbeat_task_attributes);
//
//  /* USER CODE BEGIN RTOS_THREADS */
////////////////////////////////////////////////////////////////  /* add threads, ... */
//  /* USER CODE END RTOS_THREADS */
//
//  /* USER CODE BEGIN RTOS_EVENTS */
////////////////////////////////////////////////////////////////  /* add events, ... */
//  /* USER CODE END RTOS_EVENTS */
//
//  /* Start scheduler */
//  osKernelStart();
//
//  /* We should never get here as control is now taken by the scheduler */
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
////////////////////////////////////////////////////////////////  while (1)
////////////////////////////////////////////////////////////////  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
////////////////////////////////////////////////////////////////  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 128;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
//  RCC_OscInitStruct.PLL.PLLQ = 2;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
//}
//
///**
//  * @brief CAN1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_CAN1_Init(void)
//{
//
//  /* USER CODE BEGIN CAN1_Init 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END CAN1_Init 0 */
//
//  /* USER CODE BEGIN CAN1_Init 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END CAN1_Init 1 */
//  hcan1.Instance = CAN1;
//  hcan1.Init.Prescaler = 16;
//  hcan1.Init.Mode = CAN_MODE_NORMAL;
//  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
//  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
//  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
//  hcan1.Init.TimeTriggeredMode = DISABLE;
//  hcan1.Init.AutoBusOff = DISABLE;
//  hcan1.Init.AutoWakeUp = DISABLE;
//  hcan1.Init.AutoRetransmission = DISABLE;
//  hcan1.Init.ReceiveFifoLocked = DISABLE;
//  hcan1.Init.TransmitFifoPriority = DISABLE;
//  if (HAL_CAN_Init(&hcan1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN CAN1_Init 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END CAN1_Init 2 */
//
//}
//
///**
//  * @brief I2C1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_I2C1_Init(void)
//{
//
//  /* USER CODE BEGIN I2C1_Init 0 */
////////////////////
//  /* USER CODE END I2C1_Init 0 */
//
//  /* USER CODE BEGIN I2C1_Init 1 */
////////////////////
//  /* USER CODE END I2C1_Init 1 */
//  hi2c1.Instance = I2C1;
//  hi2c1.Init.ClockSpeed = 100000;
//  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c1.Init.OwnAddress1 = 0;
//  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c1.Init.OwnAddress2 = 0;
//  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C1_Init 2 */
////////////////////
//  /* USER CODE END I2C1_Init 2 */
//
//}
//
///**
//  * @brief I2C2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_I2C2_Init(void)
//{
//
//  /* USER CODE BEGIN I2C2_Init 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END I2C2_Init 0 */
//
//  /* USER CODE BEGIN I2C2_Init 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END I2C2_Init 1 */
//  hi2c2.Instance = I2C2;
//  hi2c2.Init.ClockSpeed = 100000;
//  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c2.Init.OwnAddress1 = 0;
//  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c2.Init.OwnAddress2 = 0;
//  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C2_Init 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END I2C2_Init 2 */
//
//}
//
///**
//  * @brief RTC Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_RTC_Init(void)
//{
//
//  /* USER CODE BEGIN RTC_Init 0 */
//
//  /* USER CODE END RTC_Init 0 */
//
//  RTC_TimeTypeDef sTime = {0};
//  RTC_DateTypeDef sDate = {0};
//
//  /* USER CODE BEGIN RTC_Init 1 */
//
//  /* USER CODE END RTC_Init 1 */
//
//  /** Initialize RTC Only
//  */
//  hrtc.Instance = RTC;
//  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//  hrtc.Init.AsynchPrediv = 127;
//  hrtc.Init.SynchPrediv = 255;
//  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//  if (HAL_RTC_Init(&hrtc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* USER CODE BEGIN Check_RTC_BKUP */
//
//  /* USER CODE END Check_RTC_BKUP */
//
//  /** Initialize RTC and set the Time and Date
//  */
//  sTime.Hours = 0x0;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//  sDate.Month = RTC_MONTH_JANUARY;
//  sDate.Date = 0x1;
//  sDate.Year = 0x24;
//
//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN RTC_Init 2 */
//
//  /* USER CODE END RTC_Init 2 */
//
//}
//
///**
//  * @brief SPI2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_SPI2_Init(void)
//{
//
//  /* USER CODE BEGIN SPI2_Init 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END SPI2_Init 0 */
//
//  /* USER CODE BEGIN SPI2_Init 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END SPI2_Init 1 */
//  /* SPI2 parameter configuration*/
//  hspi2.Instance = SPI2;
//  hspi2.Init.Mode = SPI_MODE_MASTER;
//  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi2.Init.NSS = SPI_NSS_SOFT;
//  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi2.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI2_Init 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END SPI2_Init 2 */
//
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//////////////
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_IC_InitTypeDef sConfigIC = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//////////////
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 64-1;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 10000-1;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
//  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
//  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
//  sConfigIC.ICFilter = 0;
//  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//////////////
//  /* USER CODE END TIM1_Init 2 */
//
//}
//
///**
//  * @brief TIM2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
////
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
////
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 32-1;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 4294967295;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */
////
//  /* USER CODE END TIM2_Init 2 */
//
//}
//
///**
//  * @brief TIM6 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM6_Init(void)
//{
//
//  /* USER CODE BEGIN TIM6_Init 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM6_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM6_Init 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM6_Init 1 */
//  htim6.Instance = TIM6;
//  htim6.Init.Prescaler = 0;
//  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim6.Init.Period = 65535;
//  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM6_Init 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM6_Init 2 */
//
//}
//
///**
//  * @brief TIM7 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM7_Init(void)
//{
//
//  /* USER CODE BEGIN TIM7_Init 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM7_Init 0 */
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM7_Init 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM7_Init 1 */
//  htim7.Instance = TIM7;
//  htim7.Init.Prescaler = 32;
//  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim7.Init.Period = 65535;
//  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM7_Init 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM7_Init 2 */
//
//}
//
///**
//  * @brief TIM10 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM10_Init(void)
//{
//
//  /* USER CODE BEGIN TIM10_Init 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM10_Init 0 */
//
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM10_Init 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM10_Init 1 */
//  htim10.Instance = TIM10;
//  htim10.Init.Prescaler = 64-1;
//  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim10.Init.Period = 1000;
//  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_OnePulse_Init(&htim10, TIM_OPMODE_SINGLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_TIMING;
//  sConfigOC.Pulse = 750;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM10_Init 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM10_Init 2 */
//  HAL_TIM_MspPostInit(&htim10);
//
//}
//
///**
//  * @brief TIM13 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM13_Init(void)
//{
//
//  /* USER CODE BEGIN TIM13_Init 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM13_Init 0 */
//
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM13_Init 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM13_Init 1 */
//  htim13.Instance = TIM13;
//  htim13.Init.Prescaler = 32-1;
//  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim13.Init.Period = 1000;
//  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_OnePulse_Init(&htim13, TIM_OPMODE_SINGLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_TIMING;
//  sConfigOC.Pulse = 750;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM13_Init 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM13_Init 2 */
//  HAL_TIM_MspPostInit(&htim13);
//
//}
//
///**
//  * @brief TIM14 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM14_Init(void)
//{
//
//  /* USER CODE BEGIN TIM14_Init 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM14_Init 0 */
//
//  TIM_OC_InitTypeDef sConfigOC = {0};
//
//  /* USER CODE BEGIN TIM14_Init 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM14_Init 1 */
//  htim14.Instance = TIM14;
//  htim14.Init.Prescaler = 32-1;
//  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim14.Init.Period = 1000;
//  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_TIM_OnePulse_Init(&htim14, TIM_OPMODE_SINGLE) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_TIMING;
//  sConfigOC.Pulse = 750;
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM14_Init 2 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END TIM14_Init 2 */
//  HAL_TIM_MspPostInit(&htim14);
//
//}
//
///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
////////////////////////
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
////////////////////////
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate = 115200;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
////////////////////////
//  /* USER CODE END USART2_Init 2 */
//
//}
//
///**
//  * @brief WWDG Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_WWDG_Init(void)
//{
//
//  /* USER CODE BEGIN WWDG_Init 0 */
//////////////////////////////////////////
//  /* USER CODE END WWDG_Init 0 */
//
//  /* USER CODE BEGIN WWDG_Init 1 */
//////////////////////////////////////////
//  /* USER CODE END WWDG_Init 1 */
//  hwwdg.Instance = WWDG;
//  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
//  hwwdg.Init.Window = 64;
//  hwwdg.Init.Counter = 64;
//  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
//  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN WWDG_Init 2 */
//////////////////////////////////////////
//  /* USER CODE END WWDG_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, STEPPER_PULSE_Pin|STEPPER_DIR_Pin, GPIO_PIN_SET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOA, STEPPER_ENABLE_Pin|LD2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, SPI2_CS3_Pin|GPIO_PIN_14|SPI2_CS2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : STEPPER_PULSE_Pin */
//  GPIO_InitStruct.Pin = STEPPER_PULSE_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  HAL_GPIO_Init(STEPPER_PULSE_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : STEPPER_DIR_Pin LD2_Pin */
//  GPIO_InitStruct.Pin = STEPPER_DIR_Pin|LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : STEPPER_ENABLE_Pin */
//  GPIO_InitStruct.Pin = STEPPER_ENABLE_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(STEPPER_ENABLE_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PC4 PC5 PC6 */
//  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : STEPPER_ALARM_Pin PB9 */
//  GPIO_InitStruct.Pin = STEPPER_ALARM_Pin|GPIO_PIN_9;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : SPI2_CS3_Pin SPI2_CS2_Pin */
//  GPIO_InitStruct.Pin = SPI2_CS3_Pin|SPI2_CS2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PB14 */
//  GPIO_InitStruct.Pin = GPIO_PIN_14;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /*Configure GPIO pins : PC7 PC8 */
//  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PA8 */
//  GPIO_InitStruct.Pin = GPIO_PIN_8;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//}
//
///* USER CODE BEGIN 4 */
////////////////////////////////////////////////////////////////
///* USER CODE END 4 */
//
///* USER CODE BEGIN Header_start_initialization_task */
/////////////////////////////////////////////////////////////////**
////////////////////////////////////////////////////////////////  * @brief  Function implementing the initialization_task thread.
////////////////////////////////////////////////////////////////  * @param  argument: Not used
////////////////////////////////////////////////////////////////  * @retval None
////////////////////////////////////////////////////////////////  */
///* USER CODE END Header_start_initialization_task */
//void start_initialization_task(void *argument)
//{
//  /* USER CODE BEGIN 5 */
////////////////////////////////////////////////////////////////  /* Infinite loop */
////////////////////////////////////////////////////////////////  for(;;)
////////////////////////////////////////////////////////////////  {
////////////////////////////////////////////////////////////////    osDelay(1);
////////////////////////////////////////////////////////////////  }
//  /* USER CODE END 5 */
//}
//
///* USER CODE BEGIN Header_start_preparation_process_task */
/////////////////////////////////////////////////////////////////**
////////////////////////////////////////////////////////////////* @brief Function implementing the preparation_process_task thread.
////////////////////////////////////////////////////////////////* @param argument: Not used
////////////////////////////////////////////////////////////////* @retval None
////////////////////////////////////////////////////////////////*/
///* USER CODE END Header_start_preparation_process_task */
//void start_preparation_process_task(void *argument)
//{
//  /* USER CODE BEGIN start_preparation_process_task */
////////////////////////////////////////////////////////////////  /* Infinite loop */
////////////////////////////////////////////////////////////////  for(;;)
////////////////////////////////////////////////////////////////  {
////////////////////////////////////////////////////////////////    osDelay(1);
////////////////////////////////////////////////////////////////  }
//  /* USER CODE END start_preparation_process_task */
//}
//
///* USER CODE BEGIN Header_start_extrusion_process_task */
/////////////////////////////////////////////////////////////////**
////////////////////////////////////////////////////////////////* @brief Function implementing the extrusion_process_task thread.
////////////////////////////////////////////////////////////////* @param argument: Not used
////////////////////////////////////////////////////////////////* @retval None
////////////////////////////////////////////////////////////////*/
///* USER CODE END Header_start_extrusion_process_task */
//void start_extrusion_process_task(void *argument)
//{
//  /* USER CODE BEGIN start_extrusion_process_task */
////////////////////////////////////////////////////////////////  /* Infinite loop */
////////////////////////////////////////////////////////////////  for(;;)
////////////////////////////////////////////////////////////////  {
////////////////////////////////////////////////////////////////    osDelay(1);
////////////////////////////////////////////////////////////////  }
//  /* USER CODE END start_extrusion_process_task */
//}
//
///* USER CODE BEGIN Header_start_spooling_process_task */
/////////////////////////////////////////////////////////////////**
////////////////////////////////////////////////////////////////* @brief Function implementing the spooling_process_task thread.
////////////////////////////////////////////////////////////////* @param argument: Not used
////////////////////////////////////////////////////////////////* @retval None
////////////////////////////////////////////////////////////////*/
///* USER CODE END Header_start_spooling_process_task */
//void start_spooling_process_task(void *argument)
//{
//  /* USER CODE BEGIN start_spooling_process_task */
////////////////////////////////////////////////////////////////  /* Infinite loop */
////////////////////////////////////////////////////////////////  for(;;)
////////////////////////////////////////////////////////////////  {
////////////////////////////////////////////////////////////////    osDelay(1);
////////////////////////////////////////////////////////////////  }
//  /* USER CODE END start_spooling_process_task */
//}
//
///* USER CODE BEGIN Header_start_comms_updater_task */
/////////////////////////////////////////////////////////////////**
////////////////////////////////////////////////////////////////* @brief Function implementing the comms_updater_task thread.
////////////////////////////////////////////////////////////////* @param argument: Not used
////////////////////////////////////////////////////////////////* @retval None
////////////////////////////////////////////////////////////////*/
///* USER CODE END Header_start_comms_updater_task */
//void start_comms_updater_task(void *argument)
//{
//  /* USER CODE BEGIN start_comms_updater_task */
////////////////////////////////////////////////////////////////  /* Infinite loop */
////////////////////////////////////////////////////////////////  for(;;)
////////////////////////////////////////////////////////////////  {
////////////////////////////////////////////////////////////////    osDelay(1);
////////////////////////////////////////////////////////////////  }
//  /* USER CODE END start_comms_updater_task */
//}
//
///* USER CODE BEGIN Header_start_heartbeat_task */
///////////////////////////**
//////////////////////////* @brief Function implementing the heartbeat_task thread.
//////////////////////////* @param argument: Not used
//////////////////////////* @retval None
//////////////////////////*/
///* USER CODE END Header_start_heartbeat_task */
//void start_heartbeat_task(void *argument)
//{
//  /* USER CODE BEGIN start_heartbeat_task */
//////////////////////////  /* Infinite loop */
//////////////////////////  for(;;)
//////////////////////////  {
//////////////////////////    osDelay(1);
//////////////////////////  }
//  /* USER CODE END start_heartbeat_task */
//}
//
///* comms_handler_tick_callback function */
//void comms_handler_tick_callback(void *argument)
//{
//  /* USER CODE BEGIN comms_handler_tick_callback */
//////////////////////////////////////////////////
//  /* USER CODE END comms_handler_tick_callback */
//}
//
///**
//  * @brief  Period elapsed callback in non blocking mode
//  * @note   This function is called  when TIM4 interrupt took place, inside
//  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
//  * a global variable "uwTick" used as application time base.
//  * @param  htim : TIM handle
//  * @retval None
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* USER CODE BEGIN Callback 0 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM4) {
//    HAL_IncTick();
//  }
//  /* USER CODE BEGIN Callback 1 */
////////////////////////////////////////////////////////////////
//  /* USER CODE END Callback 1 */
//}
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
////////////////////////////////////////////////////////////////  /* User can add his own implementation to report the HAL error return state */
////////////////////////////////////////////////////////////////  __disable_irq();
////////////////////////////////////////////////////////////////  while (1)
////////////////////////////////////////////////////////////////  {
////////////////////////////////////////////////////////////////  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
////////////////////////////////////////////////////////////////  /* User can add his own implementation to report the file name and line number,
////////////////////////////////////////////////////////////////     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
