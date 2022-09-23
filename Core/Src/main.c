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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "stdio.h"
#include "math.h"
#include "arm_math.h"
#include "stm32_dsp.h"
#include "File_Handling_RTOS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
SemaphoreHandle_t SimMutex;
SemaphoreHandle_t Bin_sem;

TaskHandle_t Scope1_handler;
TaskHandle_t Scope2_handler;
TaskHandle_t Data_handler;
TaskHandle_t FFT_handler;
TaskHandle_t SD_handler;

QueueHandle_t SD_queue;
QueueHandle_t FFT_queue;
QueueHandle_t FFT_input_queue;

TimerHandle_t Timer_handler;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.141592
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI3_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//button_state//
int stop_button=0;
int FFT_button=0;
int SD_flag=0;

int SD_ADC_falg=0;
int SD_FFT_falg=0;
//time scale//
int time_scale_tune=128;//for tuning
int time_scale=128;//time scale(when IT frequency=10kHz)
int freq_scale_fft=1;//FFT scale
//magnitude scale//
uint8_t mag_scale_tune=1;//for tuning
uint8_t mag_scale=1;//mag scale
int amplify=1;
//scope bias//
uint8_t scope_bias_tune=0;//for tuning
uint8_t scope_bias=0;//tuning bias

uint8_t rx_buf[5];

//Uart_transmit and Critical section setup//
uint8_t cmd_end[3]={0xff,0xff,0xff};
uint8_t mutex_flag=0;
char buf[20];//for wave transfer
char buf_2[25];//for txt transfer
char buf_3[20];//for FFT transfer
char buf_4[25];//for txt transfer
void Scope1_task(void *argument);
void Scope2_task(void *argument);
void Data_task(void *argument);
void FFT_task(void *argument);
void SD_task(void *argument);

void Critical_section_Uart(int len,char *str)
{
	xSemaphoreTake(SimMutex,portMAX_DELAY);
	HAL_UART_Transmit_DMA(&huart4, (uint8_t *)str,len);
	HAL_Delay(1);
	xSemaphoreGive(SimMutex);
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)
	{
		if(rx_buf[1]==0x31)
		{
			if(rx_buf[2]==0x02)
			{
				if(rx_buf[3]==0x01)stop_button=1;
				else stop_button=0;
			}
			else if(rx_buf[2]==0x05)
			{
				time_scale_tune=time_scale*2;
				if(time_scale_tune<128)time_scale=time_scale_tune;
				else time_scale=128;
			}
			else if(rx_buf[2]==0x06)
			{
				time_scale_tune=time_scale/2;
				if(time_scale_tune>1)time_scale=time_scale_tune;
				else time_scale=1;
			}
			else if(rx_buf[2]==0x0A)
			{
				if(rx_buf[3]==0x01)FFT_button=1;
				else FFT_button=0;
			}
			else if(rx_buf[2]==0x0B)
			{
				if(rx_buf[3]==0x01)amplify=10;
				else amplify=1;
			}
			else if(rx_buf[2]==0x08)
			{
				SD_flag=1;//let sd_falg=1,when write is done,reset flag=0;
				SD_ADC_falg=1;//Store ADC data to SD card
			}
			else if(rx_buf[2]==0x0C)
			{
				SD_flag=1;//let sd_falg=1,when write is done,reset flag=0;
				SD_FFT_falg=1;//Store FFT data to SD card
			}
			else
			{

			}

		}

		else
		{
			if(rx_buf[2]==0x02)
			{
				if(rx_buf[3]==0x01)stop_button=1;
				else stop_button=0;
			}
			else if(rx_buf[2]==0x05)
			{
				time_scale_tune=time_scale*2;
				if(time_scale_tune<128)time_scale=time_scale_tune;
				else time_scale=128;
			}
			else if(rx_buf[2]==0x06)
			{
				time_scale_tune=time_scale/2;
				if(time_scale_tune>1)time_scale=time_scale_tune;
				else time_scale=1;
			}
			else if(rx_buf[2]==0x07)
			{
				mag_scale_tune=mag_scale*2;
				if(mag_scale_tune<32)mag_scale=mag_scale_tune;
				else mag_scale=32;
			}
			else if(rx_buf[2]==0x08)
			{
				mag_scale_tune=mag_scale/2;
				if(mag_scale_tune>1)mag_scale=mag_scale_tune;
				else mag_scale=1;
			}
			else if(rx_buf[2]==0x09)
			{
				scope_bias_tune=scope_bias_tune+5;
				if(scope_bias_tune<255)scope_bias=scope_bias_tune;
				else scope_bias=255;
			}
			else if(rx_buf[2]==0x0A)
			{
				scope_bias_tune=scope_bias_tune-5;
				if(scope_bias_tune>0)scope_bias=scope_bias_tune;
				else scope_bias=0;
			}
			else
			{

			}
		}
	}
}


//-----------------------------------------------------//
//------------------adc_measurement--------------------//
//-----------------------------------------------------//
uint32_t adc_val;
uint8_t adc_data_buf[1000];
uint16_t adc_dma[1];
int sample_counter=0;//decide sample rate
int store_counter=0;//decide store array index
int uart_trans_counter=0;//decide uart transfer index;
//frequency calculation//
int cros_time=0;//cros_zero time
uint8_t frequency=0;//wave freq
uint8_t frequency_mod=0;//wave freq
//magnitude calculation//
uint8_t mag_max=0;//wave magnitude
int mag_max_int=0;
int mag_max_mod=0;
int flush_counter=0;//use for flush magnitude data



//---------------------------------------------------//
//-------------sin waveform generate-----------------//
//---------------------------------------------------//
uint32_t sin_val[100];//for saving sin waveform sample data

void get_sinval ()
{
	for(int i=0;i<100;i++)
	{
		sin_val[i]=(sin(i*2*PI*1/100)*(4096/2)+2048);
	}
}


//-------------------------------------------------//
//------------------FFT setup----------------------//
//-------------------------------------------------//
#define FFT_length 64
int fft_counter=0;
int fft_transfer_counter=31;
int fft_hold_counter=0;
int fft_complete_flag=0;
int value_fft=0;
int fft_sample_counter=0;
uint32_t input[FFT_length], output[FFT_length],Mag[FFT_length];/* 输入，输出 */
float32_t Phase[FFT_length];/* 相位，幅值*/


/*
*********************************************************************************************************
* 函 数 名: PowerMag
* 功能说明: 求模值
* 形 参：_usFFTPoints FFT点数
* 返 回 值: 无
*********************************************************************************************************
*/
void PowerMag(uint16_t _usFFTPoints)
{
	int16_t lX,lY;
	uint16_t i;
	float32_t mag;
	/* 计算幅值 */
	for (i=0; i < _usFFTPoints; i++)
	{
	lX= (output[i]<<16)>>16; /* 实部*/
	lY= (output[i]>> 16); /* 虚部 */
	arm_sqrt_f32((float32_t)(lX*lX+ lY*lY), &mag); /* 求模 */
	Mag[i]= mag*2; /* 求模后乘以2才是实际模值，直流分量不需要乘2 */
	}

	/* 由于上面多乘了2，所以这里直流分量要除以2 */
	Mag[0] = Mag[0]>>1;
}

/*
*********************************************************************************************************
* 函 数 名: Power_Phase_Radians
* 功能说明: 求相位
* 形 参：_usFFTPoints FFT点数， uiCmpValue 阀值
* 返 回 值: 无
*********************************************************************************************************
*/
void Power_Phase_Radians(uint16_t _usFFTPoints, uint32_t _uiCmpValue)
{
	int16_t lX, lY;
	uint16_t i;
	float32_t phase;
	float32_t mag;
	for (i=0; i <_usFFTPoints; i++)
	{
		lX= (output[i]<<16)>>16; /* 实部 */
		lY= (output[i] >> 16); /* 虚部 */
		phase = atan2(lY, lX); /* atan2求解的结果范围是(-pi, pi], 弧度制 */
		arm_sqrt_f32((float32_t)(lX*lX+ lY*lY), &mag); /* 求模 */
		if(_uiCmpValue > mag)
		{
			Phase[i] = 0;
		}
		else
		{
			Phase[i] = phase* 180.0f/PI; /* 将求解的结果由弧度转换为角度 */
		}
	}
}
/*
*********************************************************************************************************
* 函 数 名: DSP_FFTPhase
* 功能说明: 64点FFT的相位求解
* 形 参：无
* 返 回 值: 无
*********************************************************************************************************
*/
void DSP_FFTPhase(void)
{

	/* 计算64点FFT
	 output：输出结果，高16位是虚部，低16位是实部。
	 input ：输入数据，高16位是虚部，低16位是实部。
	 第三个参数必须是64。
	*/
	cr4_fft_64_stm32(output, input, 64);
	/* 求幅值 */
	PowerMag(FFT_length);

	Power_Phase_Radians(FFT_length, 100);
}


//----------------SD_card setup---------------------//
char buf_sd[50];//for SD card sprintf use
int buf_val[64];//for test SD card function
int sd_buf[64];//store adc data,for save to SD card
int sd_store_counter=0;
int write_counter_adc=0;
int write_counter_fft=0;

void buffer_value_set()
{
	for(int i=0;i<64;i++)
	{
		buf_val[i]=i;
	}
}


//------------Software timer callback-------------//

void timer_callback (xTimerHandle xtimer)
{
	uint32_t Tick_delay=pdMS_TO_TICKS(1);
	int FFT_input_val;
	if(xtimer==Timer_handler)
	{
		sample_counter++;
		//sample counter decide sample rate//
		if(sample_counter>=1*time_scale)
		{

			sample_counter=0;
			if(store_counter>=1000)store_counter=0;
			HAL_ADC_Start(&hadc1);
			adc_data_buf[store_counter]=adc_val/16;
			store_counter++;



			//frequency calculation//
			if(adc_val>=2048)cros_time++;
			else if(adc_val<2048&&cros_time!=0)
			{
				frequency=1/(0.0001*time_scale*cros_time*2);
				cros_time=0;
			}

			//magnitude calculation//
			if((adc_val/16)>mag_max)
			{
				mag_max=adc_val/16;//if adc_val >previous value,change max value
				mag_max_int=mag_max*3.3*amplify/255;
				mag_max_mod=(mag_max*33/10*amplify%255)/26;
			}
			else
			{
				flush_counter++;
				if(flush_counter>=(1000/time_scale))//flush data in period
				{
					mag_max=0;//reset_mag
					frequency=0;
					flush_counter=0;
				}
			}
		}


		xQueueSend(SD_queue,&adc_val,Tick_delay);

		//FFT calculation data transfer//
		FFT_input_val=adc_val/4;
		if(fft_counter<=FFT_length)
		{
			if(xQueueSend(FFT_input_queue,&FFT_input_val,Tick_delay)!=pdTRUE)
				xSemaphoreGive(Bin_sem);//Data preparation complete Release Binary semaphore

			fft_counter++;
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
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */


  //SD card setup//
  buffer_value_set();


  Mount_SD("/");
  Format_SD();
  Create_File("ADC.TXT");
  Create_File("FFT.TXT");
  Unmount_SD("/");




  //DAC setup//
  HAL_TIM_Base_Start(&htim2);
  get_sinval();
  HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, sin_val, 100, DAC_ALIGN_12B_R);
  //ADC_DMA_setup//
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_dma,1);

  //TIM3 interrupt setup//
  HAL_TIM_Base_Start_IT(&htim3);

  //Start Receive IT//
  HAL_UART_Receive_DMA(&huart4, rx_buf, 5);


  //create semaphore//
  Bin_sem=xSemaphoreCreateBinary();
  //create mutex//
  SimMutex = xSemaphoreCreateMutex();
  if(SimMutex!=NULL)mutex_flag=1;

  //create software timer//
  Timer_handler=xTimerCreate("FFT_timer",pdMS_TO_TICKS(16), pdTRUE,(void *) 1,(TimerCallbackFunction_t)timer_callback);

  //Queue create//
  SD_queue=xQueueCreate(64,sizeof(int));
  FFT_queue=xQueueCreate(32,sizeof(int));
  FFT_input_queue=xQueueCreate(64,sizeof(uint32_t));

  //create task//
  xTaskCreate(Scope1_task,"HPT",128,NULL,4,&Scope1_handler);
  xTaskCreate(Scope2_task,"MPT",128,NULL,4,&Scope2_handler);
  xTaskCreate(Data_task,"LPT",128,NULL,3,&Data_handler);
  xTaskCreate(FFT_task,"VLPT",128,NULL,2,&FFT_handler);
  xTaskCreate(SD_task,"SDT",256,NULL,1,&SD_handler);
  vTaskStartScheduler();

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
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Scope1_task(void *argument)
{
	xTimerStart(Timer_handler,0);
	while(1)
	{
		if(uart_trans_counter>=1000)uart_trans_counter=0;

		int value=adc_data_buf[uart_trans_counter]/mag_scale+scope_bias;
		if(value>=255)value=255;

		int len=sprintf(buf,"add 1,0,%d%s",value,cmd_end);
		if(stop_button==0)
		{
		Critical_section_Uart(len,buf);//if stop button=0,send data to display
		uart_trans_counter++;
		}


		vTaskDelay(1*time_scale);
	}
}

void Scope2_task(void *argument)
{
	uint32_t Tick_delay=pdMS_TO_TICKS(1000);
	while(1)
	{
		if(FFT_button==1&&stop_button==0&&fft_complete_flag==0)
		{
			if(fft_transfer_counter<0)
			{
				fft_transfer_counter=31;
				fft_complete_flag=1;//transfer complete set flag to stop transfer data
				vTaskDelay(Tick_delay);
			}

			if(fft_hold_counter!=1)
			{
				value_fft=0;
			}
			else value_fft=Mag[fft_transfer_counter];

			int len=sprintf(buf_3,"add 1,1,%d%s",value_fft,cmd_end);
			Critical_section_Uart(len,buf_3);//if FFT_button=1,send data to display

			if(fft_hold_counter>=10)
			{
				fft_transfer_counter--;
				fft_hold_counter=0;
			}
				fft_hold_counter++;
		}
		vTaskDelay(10);
	}
}

void Data_task(void *argument)
{
	while(1)
	{
		HAL_UART_Receive_DMA(&huart4,rx_buf,5);

		if(stop_button==0)
		{
			int len=sprintf(buf_2,"t0.txt=\"freq=%d Hz\"%s",frequency,cmd_end);
			Critical_section_Uart(len,buf_2);
			len=sprintf(buf_4,"t1.txt=\"Mag=%d.%d V\"%s",mag_max_int,mag_max_mod,cmd_end);
			Critical_section_Uart(len,buf_4);
		}
		vTaskDelay(500);
	}
}

void FFT_task(void *argument)
{
	int value;
	int i;
	uint32_t Tick_delay=pdMS_TO_TICKS(1);
	while(1)
	{
		//Do FFT calculation,only when receive Binary semaphore
		if(xSemaphoreTake(Bin_sem,portMAX_DELAY)!=pdFALSE)
		{
			//Receive FFT calculation input data from Queue//
			for(i=0;i<FFT_length;i++)
			{
				xQueueReceive(FFT_input_queue, &value, Tick_delay);
				input[i]=value;
			}

			//implement calculation//
			DSP_FFTPhase();
			fft_counter=0;//Reset fft counter let timercallback can receive data
			fft_complete_flag=0;//when fft update,reset the fft flag
		}

		vTaskDelay(20);

	}
}

void SD_task(void *argument)
{
	int value;
	int value_FFT;
	uint32_t Tick_delay=pdMS_TO_TICKS(1);
	while(1)
	{
		if(SD_flag==1&&SD_ADC_falg==1)
		{
			xQueueReceive(SD_queue, &value, Tick_delay);
			if(write_counter_adc!=0)
			sprintf(buf_sd,"%d.%d\n",(write_counter_adc+1),value);
			else
			sprintf(buf_sd,"ADC_value\n%d.%d\n",(write_counter_adc+1),value);

			//update file//
			Mount_SD("/");
			Update_File("ADC.TXT",buf_sd);
			Unmount_SD("/");

			write_counter_adc++;
		}
		else
		{
			if(SD_flag==1&&SD_FFT_falg==1)
			{
				value_FFT=Mag[write_counter_fft];
				if(write_counter_fft!=0)
				sprintf(buf_sd,"%d.%d\n",(write_counter_fft+1),value_FFT);
				else
				sprintf(buf_sd,"Mag_value\n%d.%d\n",(write_counter_fft+1),value_FFT);

				//update file//
				Mount_SD("/");
				Update_File("FFT.TXT",buf_sd);
				Unmount_SD("/");

				write_counter_fft++;
			}
		}

		//Use counter to record how many data have been stored//
		//Check Data Store complete//

		if(write_counter_adc>=64&&SD_ADC_falg==1)
		{
			write_counter_adc=0;
			SD_ADC_falg=0;//ADC data store complete
		}

		if(write_counter_fft>=32&&SD_FFT_falg==1)
		{
			write_counter_fft=0;
			SD_FFT_falg=0;//FFT data store complete
		}

		if(SD_ADC_falg==0&&SD_FFT_falg==0)
		{
			SD_flag=0;//if data write complete reset flag
		}


		vTaskDelay(3*time_scale);
	}
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adc_val=adc_dma[0];
}

/* USER CODE END 4 */



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3)
  {

  }
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
