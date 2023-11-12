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
//https://www.youtube.com/watch?v=iU9Jb06Fpp4&lc=Ugxi7EpiAMnnVwpu3YF4AaABAg.9wjBtW2wV1w9wmlrlGRIeD
#include <string.h>
#include <stdio.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 1024          //E' la lunghezza della FFT. Forse va uguale al num di bit dell'ADC?? Cioè 2^12 = 4096
#define SAMPLING_RATE 6000

#define PRINTF 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
arm_rfft_fast_instance_f32 fft_audio_instance;
uint16_t adc_in_value[ADC_BUF_LEN];
float adc_in_value_volt[ADC_BUF_LEN];     //"adc_in_value" convertito in volt (tensione)


float32_t input_fft[ADC_BUF_LEN];
float32_t output_fft[ADC_BUF_LEN];
float32_t output_fft_mag[ADC_BUF_LEN / 2];
float32_t frequencies_axis_half_size[ADC_BUF_LEN / 2];
float32_t frequencies_axis[ADC_BUF_LEN / 2];

uint8_t fftIndex = 0;
int i = 0;
volatile uint16_t HalfCpltCallback;    //half_i2s
volatile uint16_t CpltCallback;        //full_i2s

float peakVal = 0.0f;
uint16_t peakHz = 0;
float curVal = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  arm_rfft_fast_init_f32(&fft_audio_instance, ADC_BUF_LEN); //Inizializzo la FFT
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


#if 0
	    int frequency = 1000;
		//generate sinusoidal signal (this is what you receive from outside in the ADC pin, for example)
		HAL_Delay(100);
		for (int i = 0; i < FFT_LENGTH; i++) {
			sine_val[i] = arm_sin_f32(2*PI*frequency*i/SAMPLING_RATE);
			//"sine_val" contains only one frequency (500Hz), so "output_fft" must contain all 0 except a spike at 500Hz
		}
#endif


		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_in_value, ADC_BUF_LEN);
		//aggiungi in basso delle funzioni callback perchè il main.c non sa quando il buffer è pieno (o pieno a metà)

//---------------------------------------------------------------------------------------------
		if (CpltCallback == 1) {      //full buffer riempito


		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);


		for (i = 0; i < ADC_BUF_LEN; i++) {
		adc_in_value_volt[i] = ((float)(adc_in_value[i])*3/4096);
		//printf("adc_in_value[%d] = %d\n", i, adc_in_value[i]);
		//printf("adc_in_value_volt[%d]: %f\n", i, adc_in_value_volt[i]);
		}


		//"output_fft" contiene: frequency values and phase informations.
		//E' formato da complex numbers:
		//- in posizione pari ci sono i numeri reali
		//- in posizione dispari c'è la parte immaginary del numero complesso.
		//Perciò, "output_fft" = [a0, b0, a1, b1, a2, b2, .. , a(N/2)-1, b(N/2)-] = a0+ib0, a1+ib1, a2+ib2 ..
		//..in cui a0,a1,a2 etc. sono la parte reale mentre b0,b1,b2 etc. sono la parte immaginaria del numero complesso.
		//Quindi se "adc_in_value_volt" input a sta funzione ha "ADC_BUF_LEN" elementi,
		//..allora "output_fft" in output avrà sempre dimensione "ADC_BUF_LEN", ma conterrà "ADC_BUF_LEN / 2" numeri complessi al suo interno .. dato che come ho detto è così: a0+ib0, a1+ib1, a2+ib2 ..
		arm_rfft_fast_f32(&fft_audio_instance, adc_in_value_volt, output_fft, 0);

		//ORA HO CALCOLATO LA FFT DI "adc_in_value_volt", RICORDA CHE CONTIENE SOLO "ADC_BUF_LEN" ELEMENTI!


		//estraggo magnitude perchè ci interessa quella .. non ci interessa la fase!
		//Estrai da "output_fft" la magnitude che è, per forza, metà fft (cioè FFT_LENGTH/2).
		//Quindi, sapendo che "output_fft" = [a0, b0, a1, b1, a2, b2, ..] = a0+ib0, a1+ib1, a2+ib2 ..
		//ottieni "output_fft_mag" = [c0, c1, c2 .. ] con c0 = sqrt(a0^2+b0^2), c1 = sqrt(a1^2+b1^2), c2 = sqrt(a2^2+b2^2)

		/*
		//1° Metodo per estrarre magnitude:
		arm_cmplx_mag_f32(output_fft, (float32_t*)output_fft_mag, ADC_BUF_LEN/2);
		output_fft_mag[0] = 0; //bias
		for (i = 0; i < ADC_BUF_LEN / 2; i++) {
			frequencies_axis[i] = ((i * SAMPLING_RATE) / ADC_BUF_LEN);
			printf("i = %d   frequency %f: %f \n", i, frequencies_axis[i], output_fft_mag[i]);
			//printf("frequency %f \n", frequencies_axis[i]);
			//HAL_UART_Transmit(&huart1, (uint16_t*)(frequencies_axis_str[i]), FFT_LENGTH / 2, HAL_MAX_DELAY);
			//HAL_Delay(5);
		}*/



		//2° Metodo per estrarre magnitude:
		uint16_t freqIndex = 0;
		for (uint16_t index = 0; index < ADC_BUF_LEN; index += 2) {

			curVal = sqrtf((output_fft[index] * output_fft[index]) + (output_fft[index+1] * output_fft[index+1]));                                                                                                  //k

			if (curVal > peakVal) {
				peakVal = curVal;
				peakHz = (uint16_t) (freqIndex * SAMPLING_RATE / ((float)ADC_BUF_LEN));
			}
			freqIndex++;
		}

	printf("peakHz = %f \n", peakVal);





CpltCallback = 0;

HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}	//fine CpltCallback == 1
//---------------------------------------------------------------------------------------------


#if 0
		//"output_fft" contiene: frequency values and phase informations.
		//E' formato da complex numbers:
		//- in posizione pari ci sono i numeri reali
		//- in posizione dispari c'è la parte immaginary del numero complesso.
		//Perciò, "output_fft" = [a0, b0, a1, b1, a2, b2, .. , a(N/2)-1, b(N/2)-] = a0+ib0, a1+ib1, a2+ib2 ..
		//..in cui a0,a1,a2 etc. sono la parte reale mentre b0,b1,b2 etc. sono la parte immaginaria del numero complesso.
		//Quindi se "adc_in_value_volt" input a sta funzione ha "ADC_BUF_LEN" elementi,
		//..allora "output_fft" in output avrà sempre dimensione "ADC_BUF_LEN", ma conterrà "ADC_BUF_LEN / 2" numeri complessi al suo interno .. dato che come ho detto è così: a0+ib0, a1+ib1, a2+ib2 ..
		arm_rfft_fast_f32(&fft_audio_instance, adc_in_value_volt, output_fft, 0);
		/*for (i = 0; i < ADC_BUF_LEN; i++) {
		printf("output_fft[%d]: %f\n", i, output_fft[i]);
		}*/


		//estraggo magnitude perchè ci interessa quella .. non ci interessa la fase!
		//Estrai da "output_fft" la magnitude che è, per forza, metà fft (cioè FFT_LENGTH/2).
		//Quindi, sapendo che "output_fft" = [a0, b0, a1, b1, a2, b2, ..] = a0+ib0, a1+ib1, a2+ib2 ..
		//ottieni "output_fft_mag" = [c0, c1, c2 .. ] con c0 = sqrt(a0^2+b0^2), c1 = sqrt(a1^2+b1^2), c2 = sqrt(a2^2+b2^2)

				//1° Metodo per estrarre magnitude:
		//arm_cmplx_mag_f32(output_fft, (float32_t*)output_fft_mag, ADC_BUF_LEN/2);
		//output_fft_mag[0] = 0; //bias
		for (i = 0; i < ADC_BUF_LEN / 2; i++) {
				frequencies_axis[i] = ((i * SAMPLING_RATE) / ADC_BUF_LEN);
				printf("i = %d   frequency %f: %f \n", i, frequencies_axis[i], output_fft_mag[i]);
				//printf("frequency %f \n", frequencies_axis[i]);
				//HAL_UART_Transmit(&huart1, (uint16_t*)(frequencies_axis_str[i]), FFT_LENGTH / 2, HAL_MAX_DELAY);
				//HAL_Delay(5);
			}

				//2° Metodo per estrarre magnitude:
		uint16_t freqIndex = 0;
		for (uint16_t index = 0; index < ADC_BUF_LEN; index += 2) {

			curVal = sqrtf((output_fft[index] * output_fft[index]) + (output_fft[index+1] * output_fft[index+1]));                                                                                                  //k

			if (curVal > peakVal) {
				peakVal = curVal;
				peakHz = (uint16_t) (freqIndex * SAMPLING_RATE / ((float)ADC_BUF_LEN));
			}
			freqIndex++;
		}
#endif

		/*for (i = 0; i < ADC_BUF_LEN; i++) {
			frequencies_axis[i] = ((i * SAMPLING_RATE) / ADC_BUF_LEN);
			printf("i = %d   frequency %f: %f \n", i, frequencies_axis[i], output_fft_mag[i]);
			//printf("frequency %f \n", frequencies_axis[i]);
			//HAL_UART_Transmit(&huart1, (uint16_t*)(frequencies_axis_str[i]), FFT_LENGTH / 2, HAL_MAX_DELAY);
			//HAL_Delay(5);
		}*/















#if 0
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); //inizio, start timing

	  //Get ADC values:
	  HAL_ADC_Start_DMA(&hadc1, adc_in_value, sizeof(adc_in_value));
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  //Poll for regular conversion complete .. il processore si blocca finchè non viene fatta conversione adc                         k
	  adc_in_value = HAL_ADC_GetValue(&hadc1);
	  tensione = adc_in_value*3/4095.0;

	  	  for (int i = 0; i < SAMPLES; i++) {
	  		  vReal[i] = tensione; // Converti il valore letto in tensione (0-3V) e convertilo in double
	  		  vImag[i] = 0;            // Imposta la parte immaginaria a zero (nel tuo caso)

	  		  HAL_Delay(1000 / SAMPLING_FREQUENCY);  //delay = 1s / SAMPLING_FREQUENCY
	  	  }
#endif


	  // Applica la finestra di Hamming ai dati
	  //FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

	  // Esegui la FFT
	  //FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);

	  // Trova la frequenza massima
	  //double peak = FFT.MajorPeak();


	  //sprintf(adc_in_str, "hu\r\n", adc_in_value); //converto "adc_in" in stringa per stamparlo
	  //HAL_UART_Transmit(&huart2, (uint8_t*)adc_in_str, strlen(adc_in_str), HAL_MAX_DELAY);



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LEDBlue_Pin|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDBlue_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin = LEDBlue_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



//Callback quando la prima metà del buffer è riempita/piena
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {

	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	HalfCpltCallback = 1;

}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	CpltCallback = 1;

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
