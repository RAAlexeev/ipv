/* USER CODE BEGIN Header */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
//#define STM32F405xx
//static void do_abort() {  }


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "menu.hpp"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FFT.h"   
#include "modbus.h"
#include "EEPROM.hpp"
#include "integrator.hpp"
#include "myUtils.hpp"
#include "SC39-11driver.h"
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

IWDG_HandleTypeDef hiwdg;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim8_up;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

WWDG_HandleTypeDef hwwdg;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 256 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId myTask_S1Handle;
uint32_t myTask_S1Buffer[ 500 ];
osStaticThreadDef_t myTask_S1ControlBlock;
osThreadId myTask_S2Handle;
uint32_t myTask_S2Buffer[ 500 ];
osStaticThreadDef_t myTask_S2ControlBlock;
osThreadId myTaskModbusHandle;
uint32_t myTaskModbusBuffer[ 2048 ];
osStaticThreadDef_t myTaskModbusControlBlock;
osTimerId myTimerBUT2Handle;
osStaticTimerDef_t myTimerBUT2ControlBlock;
osTimerId myTimerBUT1Handle;
osStaticTimerDef_t myTimerBUT1ControlBlock;
osMutexId myMutex_SPI1Handle;
osStaticMutexDef_t myMutexSPI1ControlBlock;
osMutexId myMutexI2C1Handle;
osStaticMutexDef_t myMutexI2C1ControlBlock;
osSemaphoreId myCountingSem_S01Handle;
osStaticSemaphoreDef_t myCountingSem_S01ControlBlock;
osSemaphoreId myCountingSem_S02Handle;
osStaticSemaphoreDef_t myCountingSem_S05;
osSemaphoreId myCountingSemBUT1Handle;
osStaticSemaphoreDef_t myCountingSemBUT1ControlBlock;
osSemaphoreId myCountingSemBUT2Handle;
osStaticSemaphoreDef_t myCountingSemBUT2ControlBlock;
osSemaphoreId myCountingSemMBhandle;
osStaticSemaphoreDef_t myCountingSemMBcontrolBlock;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t	but1pressed = 0, but2pressed = 0,  service = 0;
 SignalChenal SignalChenal::instances[]={SignalChenal(),SignalChenal()};
 EEPROM_t EEPROM=EEPROM_t();
 void PWM(float  velocity, float min, float max);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RNG_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_IWDG_Init(void);
static void MX_WWDG_Init(void);
void StartDefaultTask(void const * argument);
void StartTask_S1(void const * argument);
void StartTask_S2(void const * argument);
void StartTaskModbus(void const * argument);
void myTimeCallbackBUT2(void const * argument);
void myTimerCalbakBUT1(void const * argument);
//int __io_putchar(int ch) __attribute__((weak));

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void delay(uint32_t ms){
	osDelay(ms);
}

void _Error_Handler(const char *f, const uint16_t l){
	 for(;;);
}

int __io_putchar(int ch){
	return ITM_SendChar((ch));
}

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){


 	  __HAL_UART_CLEAR_OREFLAG(huart);


 	  __HAL_UART_CLEAR_NEFLAG(huart);


 	  __HAL_UART_CLEAR_FEFLAG(huart);
 	  /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */


 		if (osSemaphoreRelease(myCountingSemMBhandle)!= osOK){
 			  Error_Handler();
 		}



 //	__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
 //	if (osSemaphoreRelease(myBinSemMBhandle)!= osOK){
 //		  Error_Handler();
 //	}
 }




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */
	__HAL_DBGMCU_FREEZE_TIM2();
	__HAL_DBGMCU_FREEZE_IWDG();

	//uint32_t *ACTLR = (uint32_t *)0xE000E008;
	//*ACTLR |= 2;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();

 // MX_USB_OTG_FS_PCD_Init();
 // MX_IWDG_Init();
  //MX_WWDG_Init();
  /* USER CODE BEGIN 2 */
osDelay(1000);
  EEPROM.init();
  MX_USART1_UART_Init();

  if (( GPIO_PIN_RESET == HAL_GPIO_ReadPin( BUT1_GPIO_Port, BUT1_Pin ) )&&( GPIO_PIN_RESET == HAL_GPIO_ReadPin( BUT2_GPIO_Port, BUT2_Pin ) ))
	  service++;
	//setCoef(1,0x0);

//	setCoef(1,0x100);
//	setCoef(1,0x200);
	//  HAL_TIM_Base_Start(&htim3);

	//uint8_t uartBuf;

//	 uint8_t  *buf =(uint8_t*)"qqqqqqqqqqqqqqqqqqq";
//	HAL_UART_Transmit( &huart1,buf,10,100 );

/*

	PWM(0,0,20);
	PWM(5,0,20);
	PWM(10,0,20);
	HAL_GPIO_WritePin(U1_DE_GPIO_Port,U1_DE_Pin,GPIO_PIN_RESET);
	//HAL_Delay(1);
    if (HAL_UART_Receive(&huart1, &uartBuf ,1 , 0xFF)==HAL_OK){
    	HAL_GPIO_WritePin(U1_DE_GPIO_Port,U1_DE_Pin,GPIO_PIN_SET);
     	HAL_Delay(1);
    	HAL_UART_Transmit( &huart1,&uartBuf,1,100 );

    };
*/
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and crea  tion of myMutex_SPI1 */
  osMutexStaticDef(myMutex_SPI1, &myMutexSPI1ControlBlock);
  myMutex_SPI1Handle = osMutexCreate(osMutex(myMutex_SPI1));

  /* definition and creation of myMutexI2C1 */
  osMutexStaticDef(myMutexI2C1, &myMutexI2C1ControlBlock);
  myMutexI2C1Handle = osMutexCreate(osMutex(myMutexI2C1));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myCountingSem_S01 */
  osSemaphoreStaticDef(myCountingSem_S01, &myCountingSem_S01ControlBlock);
  myCountingSem_S01Handle = osSemaphoreCreate(osSemaphore(myCountingSem_S01), 2);

  /* definition and creation of myCountingSem_S02 */
  osSemaphoreStaticDef(myCountingSem_S02, &myCountingSem_S05);
  myCountingSem_S02Handle = osSemaphoreCreate(osSemaphore(myCountingSem_S02), 2);

  /* definition and creation of myCountingSemBUT1 */
  osSemaphoreStaticDef(myCountingSemBUT1, &myCountingSemBUT1ControlBlock);
  myCountingSemBUT1Handle = osSemaphoreCreate(osSemaphore(myCountingSemBUT1), 2);

  /* definition and creation of myCountingSemBUT2 */
  osSemaphoreStaticDef(myCountingSemBUT2, &myCountingSemBUT2ControlBlock);
  myCountingSemBUT2Handle = osSemaphoreCreate(osSemaphore(myCountingSemBUT2), 2);

  /* definition and creation of myCountingSemTIM4 */
  osSemaphoreStaticDef(myCountingSemMB, &myCountingSemMBcontrolBlock);
  myCountingSemMBhandle = osSemaphoreCreate(osSemaphore(myCountingSemMB), 5);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreWait( myCountingSem_S01Handle, portMAX_DELAY);
  osSemaphoreWait( myCountingSem_S02Handle, portMAX_DELAY);
  osSemaphoreWait( myCountingSemMBhandle, portMAX_DELAY);
  osSemaphoreWait( myCountingSem_S01Handle, portMAX_DELAY);
  osSemaphoreWait( myCountingSem_S02Handle, portMAX_DELAY);
  osSemaphoreWait( myCountingSemMBhandle, portMAX_DELAY);
  osSemaphoreWait( myCountingSemMBhandle, portMAX_DELAY);
  osSemaphoreWait( myCountingSemMBhandle, portMAX_DELAY);
  osSemaphoreWait( myCountingSemMBhandle, portMAX_DELAY);
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimerBUT2 */
  osTimerStaticDef(myTimerBUT2, myTimeCallbackBUT2, &myTimerBUT2ControlBlock);
  myTimerBUT2Handle = osTimerCreate(osTimer(myTimerBUT2), osTimerOnce, NULL);

  /* definition and creation of myTimerBUT1 */
  osTimerStaticDef(myTimerBUT1, myTimerCalbakBUT1, &myTimerBUT1ControlBlock);
  myTimerBUT1Handle = osTimerCreate(osTimer(myTimerBUT1), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 256, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask_S1 */
  osThreadStaticDef(myTask_S1, StartTask_S1, osPriorityAboveNormal, 0, 500, myTask_S1Buffer, &myTask_S1ControlBlock);
  myTask_S1Handle = osThreadCreate(osThread(myTask_S1), NULL);

  /* definition and creation of myTask_S2 */
  osThreadStaticDef(myTask_S2, StartTask_S2, osPriorityAboveNormal, 0, 500, myTask_S2Buffer, &myTask_S2ControlBlock);
  myTask_S2Handle = osThreadCreate(osThread(myTask_S2), NULL);

  /* definition and creation of myTaskModbus */
  osThreadStaticDef(myTaskModbus, StartTaskModbus, osPriorityNormal, 0, 512, myTaskModbusBuffer, &myTaskModbusControlBlock);
  myTaskModbusHandle = osThreadCreate(osThread(myTaskModbus), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */ 
  /* USER CODE END RTOS_QUEUES */
 

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
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
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}


/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /**Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
 // hspi1.Init.NSS=SPI_NSS_SOFT;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1050;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 21;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250;//501;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0x9FFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 41000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 25521;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}
/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 42000;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 25;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
	if(service)  {
		  huart1.Init.BaudRate = EEPROM.uartSpeed()*10;
		  huart1.Init.WordLength = UART_WORDLENGTH_8B;
		  huart1.Init.StopBits = uartMBparam::getStopBit();
		  huart1.Init.Parity = uartMBparam::getParity();
	  }else
  /* USER CODE END USART1_Init 1 */

  {
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  }
huart1.Instance = USART1;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RELAY_1_Pin|RELAY_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SS_Pin|SS_2_Pin|SS_3_Pin|U1_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, A_Pin|B_Pin|C_Pin|D_Pin 
                          |E_Pin|F_Pin|G_Pin|DP_Pin
						  |HG_1_Pin|HG_2_Pin|HG_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WC_GPIO_Port, WC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENS2_FAIL_Pin SENS1_FAIL_Pin */
  GPIO_InitStruct.Pin = SENS2_FAIL_Pin|SENS1_FAIL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_1_Pin RELAY_2_Pin */
  GPIO_InitStruct.Pin = RELAY_1_Pin|RELAY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SS_Pin SS_2_Pin SS_3_Pin */
  GPIO_InitStruct.Pin = SS_Pin|SS_2_Pin|SS_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT2_Pin */
  GPIO_InitStruct.Pin = BUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT1_Pin */
  GPIO_InitStruct.Pin = BUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin C_Pin D_Pin 
                           E_Pin F_Pin G_Pin DP_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin|D_Pin 
                          |E_Pin|F_Pin|G_Pin|DP_Pin|HG_1_Pin|HG_2_Pin|HG_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : U1_DE_Pin */
  GPIO_InitStruct.Pin = U1_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(U1_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WC_Pin */
  GPIO_InitStruct.Pin = WC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void Screen::display(){
		switch(screen.getCurPos()){
			case 0:
				SC39_show(SignalChenal::getInstance(&hadc1)->getVelocity());
				break;
			case 1:
				SC39_show( SignalChenal::getInstance(&hadc2)->getVelocity(), true );
				break;
			case 2:
				SC39_show( SignalChenal::getInstance(&hadc1)->getAcceleration() );
				break;
			case 3:
				SC39_show( SignalChenal::getInstance(&hadc2)->getAcceleration(), true );
				break;
		}
	}

void testMaxVelocity(float * velocity,uint8_t * tOut, float max , GPIO_TypeDef * GPIO_port, uint16_t GPIO_pin)
{
  if ( *velocity <  max ) 
    *tOut = 10;
  else 
    if (!--tOut)
    {
      *tOut = 10;
      HAL_GPIO_WritePin(GPIO_port, GPIO_pin, GPIO_PIN_RESET);
    } 
}

void PWM(float32_t velocity,  uint16_t max)
{
	uint32_t ar =__HAL_TIM_GET_AUTORELOAD(&htim3);
	float32_t dc = velocity*(ar-ar/5)/(10*max)+ar*serviceMenu.items[1].getValue(false)/200;
  uint16_t dutyCycle = (uint16_t)(dc+ar/5+ar*serviceMenu.items[0].getValue(false)/200);

//	uint16_t dutyCycle= 0x00FF;
  __HAL_TIM_SET_COMPARE( &htim3, TIM_CHANNEL_1, dutyCycle );

}


void set_mb_FFT_regs(uint8_t CHn,  uint16_t * data)
{
    ++data;
    
    for( uint16_t i = CHn * LEN/4;  i < CHn * LEN/4 + LEN/4; ++i, data+=4 )
    {
     
      ModBus_SetRegister( i + 10, (uint16_t)( (int8_t)((*(data) + *(data + 1)) * 5 ) | ((uint8_t)(*(data + 2) + *(data + 3)) * 5  << 8) ));

    }
}

void setCoef( uint8_t CHn , uint16_t Coef )
{

//  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
  uint16_t pin;
  GPIO_TypeDef * port;
  
  switch(CHn)
  {
   case 0:

     pin = SS_Pin;
     port = SS_GPIO_Port;
     break;

   case 1:  
     pin = SS_3_Pin;
     port = SS_3_GPIO_Port;
     break;
  }
  uint16_t txData,rxData;
//for(;;)
{

// while(1){//byteOrder::reverse(data);

  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
 // for(;;){
	  HAL_Delay(1);
  txData= 0x1803;

  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  if ( HAL_OK != HAL_SPI_Transmit( &hspi1, (uint8_t *)&txData, 1 , 200) )
  {
     _Error_Handler(__FILE__, __LINE__);
  }

   HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

   txData = (1 << 10) | ( Coef&0xFFC  );
   Coef =~Coef;
   HAL_Delay(1);

   HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

	  if ( HAL_OK != HAL_SPI_Transmit( &hspi1, (uint8_t *)&txData, 1 , 200) )
	  {
		 _Error_Handler(__FILE__, __LINE__);
	  }
   HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
//   }
 //  for(;;){
	   txData=0x0800 ;

   HAL_Delay(1);
		  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

		  if ( HAL_OK != HAL_SPI_TransmitReceive( &hspi1, (uint8_t *)&txData,(uint8_t *)&rxData, 1 , 200) )
		  {
			 _Error_Handler(__FILE__, __LINE__);
		  }


		   //  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
		  	  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  // }
   txData=0x0C00;

  HAL_Delay(1);
		  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

		  if ( HAL_OK != HAL_SPI_Transmit( &hspi1, (uint8_t *)&txData, 1 , 200) )
		  {
			 _Error_Handler(__FILE__, __LINE__);
		  }
 HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
 HAL_Delay(1);
 txData= 0x1C00;
    //byteOrder::reverse( data );
   HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
   if ( HAL_OK != HAL_SPI_Transmit( &hspi1, (uint8_t *)&txData, 1 , 200) )
   {
      _Error_Handler(__FILE__, __LINE__);
   }

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(1);
    txData= 0x0;
    // byteOrder::reverse(data);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    if ( HAL_OK != HAL_SPI_Transmit( &hspi1, (uint8_t *)&txData, 1 , 200) )
    {
       _Error_Handler(__FILE__, __LINE__);
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
     if ( HAL_OK !=  HAL_SPI_Receive(&hspi1, (uint8_t *)&rxData, 1 , 200))    {
         _Error_Handler(__FILE__, __LINE__);
      };
     HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}
  



/*
  data = 3 << 10;
  if ( HAL_OK != HAL_SPI_Transmit( &hspi1, (uint8_t *)data, 1, 100 ) ) 
  {
     _Error_Handler(__FILE__, __LINE__);
  }

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   */
 // osMutexRelease( myMutex_SPI1Handle );
  
}


/* USART1 reinit function */
void My_USART1_UART_ReInit( void )
{
const uint32_t BaudRate[] = {115200, 4800, 9600, 19200, 38400, 57600, 115200};  
  if (HAL_UART_DeInit(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  union{
    uint16_t raw;
    struct{
      unsigned baudRate: 7;
      unsigned wordLength : 4;
      unsigned parity : 3;
      unsigned stopBits : 2;
    };
    
  } param = {ModBus_GetRegister( 0 )};
  
  huart1.Instance = USART1;
  
  huart1.Init.BaudRate = BaudRate[ param.baudRate ];
  
  if( param.wordLength == 8 || param.wordLength == 0 )
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
  else 
    huart1.Init.WordLength = UART_WORDLENGTH_9B;
  
  if( param.stopBits <= 1 )
    huart1.Init.StopBits = UART_STOPBITS_1;
  else
     huart1.Init.StopBits = UART_STOPBITS_2;
  
   switch( param.parity )
   {
    case 0: huart1.Init.Parity = UART_PARITY_NONE;
      break;
    case 1:  huart1.Init.Parity = UART_PARITY_ODD;
      break;
    case 2: huart1.Init.Parity = UART_PARITY_EVEN;
      break;
   }
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
//uint32_t	runCnt = 0;
static uint8_t  testVelocity (float32_t v,float32_t porog1, float32_t porog2){
uint8_t ret;
	static uint32_t delay = 0X5F;

					if(delay)delay--;
					else{ if(porog1 != 0 &&( v > porog1 )){
					 		  HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET);
					 	 ret=(1<<4);//led(4,false,false);
					 	 }else if( v < (porog1 - v/10)){
					 		 	 HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_RESET);
					 		 	}

					 	 if(porog2 != 0 &&( v > porog2 )){
						 	 HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);
					 		ret|=(1<<5);//led(4,false,false);
						 }else if( v < (porog2 - v/10)){
						 		 HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);

						 	 	}
					};

return ret;
};
/* USER CODE END Header_StartDefaultTask */


void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	extern uint32_t port_IO[3];
	if(HAL_DMA_Start( &hdma_tim8_up, (uint32_t)&port_IO, (uint32_t)&GPIOD->BSRR, 3 ) != HAL_OK)
	{
   		    _Error_Handler(__FILE__, __LINE__);
    };
	  __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_UPDATE);

		 if( HAL_TIM_Base_Start(&htim8)!= HAL_OK)
		  {
		    _Error_Handler(__FILE__, __LINE__);
		  };
		  /* Enable the TIM Update DMA request */


		  //HAL_GPIO_WritePin(HG_1_GPIO_Port,HG_1_Pin,GPIO_PIN_SET);
		  //HAL_GPIO_WritePin(A_GPIO_Port,A_Pin,GPIO_PIN_SET);
   //menuInit();

		uint16_t tout = 0xF;
		bool onlyOne = true;
	/* Infinite loop */
  for(;;)
  {

	  if(service >= 5){

		  serviceMenu.display();
		  PWM(serviceMenu.getCurentIndex()?(serviceMenu.curCH?EEPROM.range1():EEPROM.range2()):0,(serviceMenu.curCH?EEPROM.range1():EEPROM.range2())/10);
	  } else if(service==0||tout==0){
		  if(onlyOne){
				led(6,false);
				led(7,true);
				led(2,true);
				led(3,false);
				onlyOne = false;
				service=0;
		  }
		  uint8_t msk;
		  msk=testVelocity( SignalChenal::getInstance(ADC1)->getVelocity(), (float32_t)EEPROM.porog11.get()/10, (float32_t)EEPROM.porog12.get()/10);
		  msk|=testVelocity( SignalChenal::getInstance(ADC2)->getVelocity(), (float32_t)EEPROM.porog21.get()/10, (float32_t)EEPROM.porog22.get()/10);
		  if(!menu.isEdit)
			  scale(SignalChenal::getInstance( ( (menu.getNch()==1)?ADC1:ADC2) )->getVelocity()*1000/( (menu.getNch()==1)?EEPROM.range1():EEPROM.range2() ),msk,  [](uint16_t ms){osDelay(ms);});

		  if(SignalChenal::getInstance(ADC1)->getVelocity() > SignalChenal::getInstance(ADC2)->getVelocity()){
			PWM(SignalChenal::getInstance(ADC1)->getVelocity(),EEPROM.range1()/10);
		  }else PWM(SignalChenal::getInstance(ADC2)->getVelocity(),EEPROM.range2()/10);
		  menu.display();
	  }else
	  --tout;

	  osDelay(300);
  }
  
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask_S1 */
/**
* @brief Function implementing the myTask_S1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_S1 */

void StartTask_S1(void const * argument)
{
  /* USER CODE BEGIN StartTask_S1 */
	 HAL_DMA_RegisterCallback(&hdma_adc1,HAL_DMA_XFER_M1CPLT_CB_ID,SignalChenal::HAL_ADC_M1ConvCpltCallback);
	if ( HAL_OK != HAL_ADC_Start_DMA( &hadc1, reinterpret_cast<uint32_t *>(SignalChenal::getInstance(&hadc1)->buffer1), LEN ))
	// if ( HAL_OK != HAL_ADC_Start(&hadc1))
	 {
	     _Error_Handler(__FILE__, __LINE__);
	  }

	uint32_t b1 = reinterpret_cast<uint32_t>(&SignalChenal::getInstance(&hadc1)->buffer1);
	uint32_t b2 = reinterpret_cast<uint32_t>(&SignalChenal::getInstance(&hadc1)->buffer2);
//	DMA_MultiBufferSetConfig(hdma_adc1, reinterpret_cast<uint32_t>(&(hadc1.Instance->DR)),b1, LEN);



	if ( HAL_OK != 	   HAL_DMAEx_MultiBufferStart_IT( &hdma_adc1
												,reinterpret_cast<uint32_t>(&(hadc1.Instance->DR))
												,b1
												,b2
												,LEN )) {
		     _Error_Handler(__FILE__, __LINE__);
		  }
	hdma_adc1.Instance->CR  &= ~DMA_IT_HT;
	if ( HAL_OK != HAL_TIM_Base_Start(&htim2)){
		     _Error_Handler(__FILE__, __LINE__);
		}

  //if (ARM_MATH_SUCCESS != rfft_init()) while(1);
  /* Infinite loop */
  for(;;)
  {
    if ( osSemaphoreWait( myCountingSem_S01Handle, portMAX_DELAY) == osOK )
    {

    	SignalChenal::getInstance(&hadc1)->calc();
    }



    // minMaxAvr((q15_t*)data1);
    // rfft( data1,  fftLenReal/200 * mainMenu.items[0][iF_lo].param,  fftLenReal/200 * mainMenu.items[0][iF_hi].param,&CH[0].velocity);

//=ьеярм.б.дея\("\2\1"\)\r
    //set_mb_FFT_regs(0,(float32_t *)data1);
    
   // if(mainMenu.CHn == 0) 
   //   PWM(&CH[0].velocity, mainMenu.items[0][iVmin].param/10.f, mainMenu.items[0][iVmax].param/10.f);
    
    //uint8_t tOut = 10;
    
   // testMaxVelocity(&CH[0].velocity, &tOut, mainMenu.items[0][iVrly].param/10.f, RELAY_1_GPIO_Port, RELAY_1_Pin );
    
//    if ( osSemaphoreWait(myCountingSem_S01Handle, portMAX_DELAY) != osOK )
//    {
//      _Error_Handler(__FILE__, __LINE__);
//    }

//    minMaxAvr((q15_t*)data2);
//
//    rfft( pS, data2, fftLenReal* mainMenu.items[0][iF_lo].param / 200 , fftLenReal * mainMenu.items[0][iF_hi].param / 200, &CH[0].velocity );
//    
//    set_mb_FFT_regs(0,(float32_t *)data2);
    
 //   if( mainMenu.CHn == 0 )
 //     PWM(&CH[0].velocity, mainMenu.items[0][iVmin].param/10.f, mainMenu.items[0][iVmax].param/10.f);
      
    //testMaxVelocity( &CH[0].velocity, &tOut, mainMenu.items[0][iVrly].param/10.f, RELAY_1_GPIO_Port, RELAY_1_Pin );
    
    
    //osDelay(1);
  }
  /* USER CODE END StartTask_S1 */
}

/* USER CODE BEGIN Header_StartTask_S2 */
/**
* @brief Function implementing the myTask_S2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_S2 */
void StartTask_S2(void const * argument)
{
  /* USER CODE BEGIN StartTask_S2 */
	 HAL_DMA_RegisterCallback(&hdma_adc2,HAL_DMA_XFER_M1CPLT_CB_ID,SignalChenal::HAL_ADC_M1ConvCpltCallback);
	if ( HAL_OK != HAL_ADC_Start_DMA( &hadc2, reinterpret_cast<uint32_t *>(SignalChenal::getInstance(&hadc2)->buffer1), LEN ))
	// if ( HAL_OK != HAL_ADC_Start(&hadc1))
	 {
	     _Error_Handler(__FILE__, __LINE__);
	  }

	uint32_t b1 = reinterpret_cast<uint32_t>(&SignalChenal::getInstance(&hadc2)->buffer1);
	uint32_t b2 = reinterpret_cast<uint32_t>(&SignalChenal::getInstance(&hadc2)->buffer2);
//	DMA_MultiBufferSetConfig(hdma_adc1, reinterpret_cast<uint32_t>(&(hadc1.Instance->DR)),b1, LEN);



	if ( HAL_OK != HAL_DMAEx_MultiBufferStart_IT( &hdma_adc2
												,reinterpret_cast<uint32_t>(&(hadc2.Instance->DR))
												,b1
												,b2
												,LEN )) {
		     _Error_Handler(__FILE__, __LINE__);
		  }

	if ( HAL_OK != HAL_TIM_Base_Start(&htim2)){
		     _Error_Handler(__FILE__, __LINE__);
	}

  /* Infinite loop */
  for(;;)
  {
	    if ( osSemaphoreWait( myCountingSem_S02Handle, portMAX_DELAY) == osOK )
	    {

	    	SignalChenal::getInstance(&hadc2)->calc();
	    }

  }
  /* USER CODE END StartTask_S2 */
}

/* USER CODE BEGIN Header_StartTaskModbus */
void sendData(uint8_t* data, uint8_t len ){
	if(len==0){
		while(	HAL_UART_Receive_IT(&huart1,mb_buf_in,1) != HAL_OK) {
		  osDelay(7);//Error_Handler();
		}
	}else{
	  HAL_GPIO_WritePin(U1_DE_GPIO_Port,U1_DE_Pin,GPIO_PIN_SET);
	  if( HAL_UART_Transmit_DMA( &huart1, data, len )!= HAL_OK ){
		  Error_Handler();
	  }
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart1)

	HAL_GPIO_WritePin(U1_DE_GPIO_Port,U1_DE_Pin,GPIO_PIN_RESET);
	  if(	HAL_UART_Receive_IT(&huart1,mb_buf_in,1) != HAL_OK) {
		  Error_Handler();
	  }
}
 void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart){
	 HAL_UART_TxCpltCallback(huart);
 }
/**
* @brief Function implementing the myTaskModbus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskModbus */
void StartTaskModbus(void const * argument)
{
  /* USER CODE BEGIN StartTaskModbus */
	ModBus_Init();
	ModBus_SetAddress(service?1:uartMBparam::getAddr());
	HAL_GPIO_WritePin(U1_DE_GPIO_Port,U1_DE_Pin,GPIO_PIN_RESET);
	if(	HAL_UART_Receive_IT(&huart1,mb_buf_in,1) != HAL_OK) {
		  Error_Handler();
	}
	//HAL_UART_Receive_DMA(&huart1,mb_buf_in,256);
	 ///* Infinite loop */
	  for(;;)
	  {
			 if( osSemaphoreWait(myCountingSemMBhandle, osWaitForever )!= osOK ){
				 Error_Handler();
			 }
			 if(uxSemaphoreGetCount(myCountingSemMBhandle) > 0  )
				 continue;
			  HAL_UART_DMAStop( &huart1 );

			  uint32_t cnt =256 - __HAL_DMA_GET_COUNTER(huart1.hdmarx)+1;

			ModBusParse(cnt);

  }
  /* USER CODE END StartTaskModbus */
}


/* myTimeCallbackBUT2 function */
void myTimeCallbackBUT2(void const * argument)
{


  /* USER CODE BEGIN myTimeCallbackBUT2 */
	//if(osOK != osTimerStop(myTimerBUT2Handle)){
//		 Error_Handler();
//	}
	if (service?serviceMenu.doubleBtn(but1pressed,but2pressed):menu.doubleBtn(but1pressed,but2pressed)) {
		if(osOK != osTimerStart(myTimerBUT2Handle, 200)){
				 Error_Handler();
			};
		return;
	}



		if (but2pressed == 20){
			if(service == 0 && menu.isEdit){
				menu.navigate();
			}else{
				serviceMenu.longPlus();
			}
			//but1pressed=0;
			//return;
		}

		but2pressed++;

		if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( BUT2_GPIO_Port, BUT2_Pin ) ){

		if(osOK != osTimerStart(myTimerBUT2Handle, 100)){
				 Error_Handler();
			}

		}
		else{

			but2pressed = 0;
		}
	/*switch(screen.getCurPos()){
		case 0:
		case 2: 	screen.moveCurPos(1);
	break;
		case 1:
		case 3: 	screen.moveCurPos(-1);
	break;
	}


    // osTimerStart(myTimerBUT1Handle, 10 );
  /* USER CODE END myTimeCallbackBUT2 */
}

/* myTimerCalbakBUT1 function */
void myTimerCalbakBUT1(void const * argument)
{

	/* USER CODE BEGIN myTimerCalbakBUT1 */
 extern uint8_t service;


	if (service?serviceMenu.doubleBtn(but1pressed,but2pressed):menu.doubleBtn(but1pressed,but2pressed)){
		if(osOK != osTimerStart(myTimerBUT1Handle, 200)){
				 Error_Handler();
			}
		return;

	}
	if(but1pressed==1)
		{
			if(service==0)menu.navigate(true);
		}
	if (but1pressed == 20){
	//	if(osOK != osTimerStop(myTimerBUT1Handle)){
	//		 Error_Handler();
//		}
		if(service==0)
			menu.enterExitSetting();
		else if(service>=5)
			serviceMenu.longMinus();
			//but1pressed=0;
		//return;
	}
	but1pressed++;
	if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( BUT1_GPIO_Port, BUT1_Pin ) ){

	if(osOK != osTimerStart(myTimerBUT1Handle, 100)){
			 Error_Handler();
		}

	}
	else
		but1pressed = 0;
/*	switch(screen.getCurPos()){
		case 0:
		case 1: 	screen.moveCurPos(2);
	break;
		case 2:
		case 3: 	screen.moveCurPos(-2);
	break;
	}
  /* USER CODE END myTimerCalbakBUT1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */


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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

