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
#include "vl53l0x_api.h"
#include "stm32f4xx.h"
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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//VL53L0X_RangingMeasurementData_t RangingData;
//VL53L0X_Dev_t  devCenter; // left module
//VL53L0X_Dev_t  devLeft; // center module
//VL53L0X_Dev_t  devRight; // right module


VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_RangingMeasurementData_t RangingData2;

VL53L0X_Dev_t  vl53l0x_p; // prawy
VL53L0X_Dev_t  vl53l0x_c; // center module
VL53L0X_Dev_t  vl53l0x_l; // lewy

VL53L0X_DEV    Dev = &vl53l0x_p;

int flaga_startu=0;
int flaga_startu2=0;
int status;

volatile static uint16_t sensor_left_dist;
volatile static uint16_t sensor_center_dist;
volatile static uint16_t sensor_right_dist;

uint8_t Message[64];
uint8_t MessageLen;

uint16_t ticksy=0;

volatile static uint16_t value[2];// pomiar adc tablica 2 elementowa
volatile static uint16_t value_calib[2];// progi dla czujnikow krawedziowych

void fun_namierz(void){

	HAL_TIM_Base_Start_IT(&htim11);


}



// Override the weak call back function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

      if (htim->Instance == TIM11)
      {


    	  if (flaga_startu)
		  {

    	  		MessageLen = sprintf((char*)Message, "tiksy przed, i jazda: %i\n\r",ticksy);
    	  	  	HAL_UART_Transmit(&huart1, Message, MessageLen, 100);


    	  	  	if(!(value[1]<=value_calib[0]))
    	  	  	{
    	  	  		//jazda w prawo
					HAL_GPIO_WritePin(motor_2B_GPIO_Port, motor_2B_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(motor_1B_GPIO_Port, motor_1B_Pin, GPIO_PIN_RESET);

    	  	  	}else
    	  	  	{
    	  			HAL_GPIO_WritePin(motor_2B_GPIO_Port, motor_2B_Pin, GPIO_PIN_SET);
    	  			HAL_GPIO_WritePin(motor_1B_GPIO_Port, motor_1B_Pin, GPIO_PIN_SET);

    	  	  	}


    	  		if(ticksy>=(sensor_right_dist/10)) // ms if(ticksy>=55)
    	  		{
    	  			//STOP
    	  			MessageLen = sprintf((char*)Message, "zatrzymanie %i\n\r",ticksy);
    	  	  		HAL_UART_Transmit(&huart1, Message, MessageLen, 100);


    	  			HAL_GPIO_WritePin(motor_2B_GPIO_Port, motor_2B_Pin, GPIO_PIN_SET);
    	  			HAL_GPIO_WritePin(motor_1B_GPIO_Port, motor_1B_Pin, GPIO_PIN_SET);

    	  			ticksy=0; // wyzeruj ticksy
    	  			flaga_startu=0;
    	  		}

    	  		ticksy=ticksy+1;
    	  }
////////////////////////////////////////////////////////////////////////////////////////////////////
    	  if (flaga_startu2)
		  {

    		  if(!(value[0]<=value_calib[0]))
    			  {
    			  //JAZDA w lewo (patrzac od kierowcy
    			  HAL_GPIO_WritePin(motor_1A_GPIO_Port, motor_1A_Pin, GPIO_PIN_SET);
    			  HAL_GPIO_WritePin(motor_2A_GPIO_Port, motor_2A_Pin, GPIO_PIN_RESET);
    			  }else{
      				HAL_GPIO_WritePin(motor_1A_GPIO_Port, motor_1A_Pin, GPIO_PIN_SET);
      				HAL_GPIO_WritePin(motor_2A_GPIO_Port, motor_2A_Pin, GPIO_PIN_SET);
    			  }




    	  		if(ticksy>=(sensor_left_dist/20)) //if(ticksy>=55)
    	  		{
    	  			//STOP
    	  			MessageLen = sprintf((char*)Message, "zatrzymanie %i\n\r",ticksy);
    	  	  		HAL_UART_Transmit(&huart1, Message, MessageLen, 100);


    				HAL_GPIO_WritePin(motor_1A_GPIO_Port, motor_1A_Pin, GPIO_PIN_SET);
    				HAL_GPIO_WritePin(motor_2A_GPIO_Port, motor_2A_Pin, GPIO_PIN_SET);

    	  			ticksy=0; // wyzeruj ticksy
    	  			flaga_startu2=0;
    	  		}

    	  		ticksy=ticksy+1;
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
//------------------------------------------------------------

//------------------------------------------------------------------------------
	// VL53L0X initialisation stuff
	//
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;




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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */



  //--------------------------------ustawienia ADC------------------------
  //HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)value, 2); // pomiar adc z nieznanym mi czasem :)
  // ------------------------------koniec ustawien adc-------------------------



//-----------------------------------------------pierwszy czujnik prawy zmiana adresu----------------------------------------------------------------------


   HAL_GPIO_WritePin(x_prawy_GPIO_Port,x_prawy_Pin, GPIO_PIN_SET); // Enable XSHUT
   HAL_GPIO_WritePin(x_srodek_GPIO_Port,x_srodek_Pin , GPIO_PIN_RESET); // Enable XSHUT
   HAL_GPIO_WritePin(x_lewy_GPIO_Port, x_lewy_Pin, GPIO_PIN_RESET); // Enable XSHUT

   HAL_Delay(100);

// blokA
  MessageLen = sprintf((char*)Message, "inicjalizacja VL53L0X PRAWY\n\r");
    HAL_UART_Transmit(&huart1, Message, MessageLen, 100);

    Dev->I2cHandle = &hi2c1;
    Dev->I2cDevAddr = 0x52;

    VL53L0X_SetDeviceAddress(Dev, 0x30);
    HAL_Delay(100);

    Dev->I2cDevAddr = 0x30;
    // VL53L0X init for Single Measurement
    //

    VL53L0X_WaitDeviceBooted( Dev );
    VL53L0X_DataInit( Dev );
    VL53L0X_StaticInit( Dev );
    VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    // Enable/Disable Sigma and Signal check
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
    VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
    VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    //


  //-------------------------------------------------------drugi CZUJNIK center zmiana adresu-------------------------------------------------------------------------


    Dev = &vl53l0x_c;

    HAL_GPIO_WritePin(x_prawy_GPIO_Port,x_prawy_Pin, GPIO_PIN_SET); // Enable XSHUT
    HAL_GPIO_WritePin(x_srodek_GPIO_Port,x_srodek_Pin , GPIO_PIN_SET); // Enable XSHUT
    HAL_GPIO_WritePin(x_lewy_GPIO_Port, x_lewy_Pin, GPIO_PIN_RESET); // Enable XSHUT
    //HAL_Delay(1000);
 // blokA
   MessageLen = sprintf((char*)Message, "inicjalizacja VL53L0X center\n\r");
     HAL_UART_Transmit(&huart1, Message, MessageLen, 100);

     Dev->I2cHandle = &hi2c1;
     Dev->I2cDevAddr = 0x52;

     VL53L0X_SetDeviceAddress(Dev, 0x32);
     HAL_Delay(100);

     Dev->I2cDevAddr = 0x32;
     // VL53L0X init for Single Measurement
     //
     HAL_Delay(1000);

     VL53L0X_WaitDeviceBooted( Dev );
     VL53L0X_DataInit( Dev );
     VL53L0X_StaticInit( Dev );
     VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
     VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
     VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
     //VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

     // Enable/Disable Sigma and Signal check
     VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
     VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
     VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
     VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
     VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
     VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
     VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
     //

     //-------------------------------------------------------trzeci CZUJNIK lewy bezzmiany-------------------------------------------------------------------------


       Dev = &vl53l0x_l;

       HAL_GPIO_WritePin(x_prawy_GPIO_Port,x_prawy_Pin, GPIO_PIN_SET); // Enable XSHUT
       HAL_GPIO_WritePin(x_srodek_GPIO_Port,x_srodek_Pin , GPIO_PIN_SET); // Enable XSHUT
       HAL_GPIO_WritePin(x_lewy_GPIO_Port, x_lewy_Pin, GPIO_PIN_SET); // Enable XSHUT

       HAL_Delay(100);
    // blokA
      MessageLen = sprintf((char*)Message, "msalamon.pl VL53L0X drugi\n\r");
        HAL_UART_Transmit(&huart1, Message, MessageLen, 100);

        Dev->I2cHandle = &hi2c1;
        Dev->I2cDevAddr = 0x52;

       // VL53L0X_SetDeviceAddress(Dev, 0x54);

        // VL53L0X init for Single Measurement
        //
        HAL_Delay(100);

        VL53L0X_WaitDeviceBooted( Dev );
        VL53L0X_DataInit( Dev );
        VL53L0X_StaticInit( Dev );
        VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
        VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
        VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
        //VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

        // Enable/Disable Sigma and Signal check
        VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
        VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
        VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
        VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
        VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
        //





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    //INICJALIZACJA CZUJNIKOW-----------pomiar na nacznym i ustawienie progu na 1/3

  	  // pomiar czujników krancowych pierwszy raz
  	 	MessageLen = sprintf((char*)Message, "calib 1=%u, calib 2=%u\n",value[0], value[1]);
  		HAL_UART_Transmit(&huart1, Message, MessageLen, 100);

  		value_calib[0]=value[0]/2; // tu wprowadzic zmienna
  		value_calib[1]=value[1]/2; //// tu wprowadzic zmienna

  	 	MessageLen = sprintf((char*)Message, "calib prog 1=%u, calib prog 2=%u\n",value_calib[0], value_calib[1]);
  		HAL_UART_Transmit(&huart1, Message, MessageLen, 100);


  while (1)
  {
//pierwszy--------------------------------------
	 HAL_Delay(20);
	 Dev = &vl53l0x_p;
	 VL53L0X_StartMeasurement(Dev);
	  VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
	  	  if(RangingData.RangeStatus == 0)
	  	  {
	  		sensor_right_dist=RangingData.RangeMilliMeter;
	  	//	  MessageLen = sprintf((char*)Message, "prawy: %i\n\r", RangingData.RangeMilliMeter);
	  	//	  HAL_UART_Transmit(&huart1, Message, MessageLen, 100);
	  	  }
	  	VL53L0X_StopMeasurement(Dev);

//----------------------------drugi--------------------------------------

	  HAL_Delay(20);
	  Dev = &vl53l0x_c;
	  VL53L0X_StartMeasurement(Dev);
	  HAL_Delay(2);
	  VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);

	  	  if(RangingData2.RangeStatus == 0)
		  	  {
	  		  sensor_center_dist=RangingData.RangeMilliMeter;
		  	//	MessageLen = sprintf((char*)Message, "srodek: %i\n\r", RangingData2.RangeMilliMeter);
		  	//	HAL_UART_Transmit(&huart1, Message, MessageLen, 100);
		  	  }
		  VL53L0X_StopMeasurement(Dev);


//----------------------------trzeci--------------------------------------

	HAL_Delay(20);
	Dev = &vl53l0x_l;
	VL53L0X_StartMeasurement(Dev);
	HAL_Delay(2);
		VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);

		 if(RangingData2.RangeStatus == 0)
		    {
			 sensor_left_dist=RangingData.RangeMilliMeter;


			 //if(RangingData2.RangeMilliMeter>=385)
			// 	 MessageLen = sprintf((char*)Message, "za daleko lewy: %i\n\r", RangingData2.RangeMilliMeter);
		  	//	HAL_UART_Transmit(&huart1, Message, MessageLen, 100);

		  	 }
	  VL53L0X_StopMeasurement(Dev);


	  // pomiar czujników KRANCOWYCH
	 //	 MessageLen = sprintf((char*)Message, "value1=%u, value2=%u\n",value[0], value[1]);
	//	HAL_UART_Transmit(&huart1, Message, MessageLen, 100);


/* wysylanie pomiarow uartem
		MessageLen = sprintf((char*)Message, "LEWY: %i\n\r",sensor_left_dist);
	  	HAL_UART_Transmit(&huart1, Message, MessageLen, 100);

		MessageLen = sprintf((char*)Message, "SRODEK: %i\n\r",sensor_center_dist);
	  	HAL_UART_Transmit(&huart1, Message, MessageLen, 100);

		MessageLen = sprintf((char*)Message, "PRAWY: %i\n\r",sensor_right_dist);
	  	HAL_UART_Transmit(&huart1, Message, MessageLen, 100);
	  	*/

//----------------------------------------------test timera i czujniika--------------------------------------






  		//HAL_TIM_Base_Start(&htim11);
	 HAL_TIM_Base_Start_IT(&htim11);

	  	//if(TIM11->CNT>=999){
		//	MessageLen = sprintf((char*)Message, "timer pelny licznik 11: %i\n\r",TIM11->CNT);
		  //	HAL_UART_Transmit(&huart1, Message, MessageLen, 100);
	  //	}


	  	if(sensor_right_dist<=500) // 20cm
	  	{

	  		//HAL_TIM_Base_Start_IT(&htim11);
	  		flaga_startu=1;
	  		//fun_namierz();


		}
	  	if(sensor_left_dist<=500)
	  	{
	  		flaga_startu2=1;
	  	}







//-----------------------------------------zalaczanie silniczkow-------------------------------------------
/* komentarz start
		if(value[0]<=value_calib[0])
		{

			// wychamowanie
			HAL_GPIO_WritePin(motor_1A_GPIO_Port, motor_1A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor_2A_GPIO_Port, motor_2A_Pin, GPIO_PIN_SET);

			//STOP
			HAL_GPIO_WritePin(motor_1A_GPIO_Port, motor_1A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor_2A_GPIO_Port, motor_2A_Pin, GPIO_PIN_SET);

		}else
		{
			// JAZDA
			HAL_GPIO_WritePin(motor_1A_GPIO_Port, motor_1A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor_2A_GPIO_Port, motor_2A_Pin, GPIO_PIN_RESET);
		}

		if(value[1]<=value_calib[1])
		{

			//wychamowanie
			HAL_GPIO_WritePin(motor_2B_GPIO_Port, motor_2B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor_1B_GPIO_Port, motor_1B_Pin, GPIO_PIN_SET);

			//STOP
			HAL_GPIO_WritePin(motor_1B_GPIO_Port, motor_1B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor_2B_GPIO_Port, motor_2B_Pin, GPIO_PIN_SET);

		}else
		{
			//JAZDA w lewo (patrzac od kierowcy
			HAL_GPIO_WritePin(motor_2B_GPIO_Port, motor_2B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor_1B_GPIO_Port, motor_1B_Pin, GPIO_PIN_RESET);


		}
komentarz end
*/





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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 159;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, x_lewy_Pin|x_srodek_Pin|x_prawy_Pin|motor_1B_Pin
                          |motor_2A_Pin|motor_1A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motor_2B_GPIO_Port, motor_2B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : x_lewy_Pin x_srodek_Pin x_prawy_Pin motor_1B_Pin
                           motor_2A_Pin motor_1A_Pin */
  GPIO_InitStruct.Pin = x_lewy_Pin|x_srodek_Pin|x_prawy_Pin|motor_1B_Pin
                          |motor_2A_Pin|motor_1A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : line_sens_L_Pin line_sens_P_Pin */
  GPIO_InitStruct.Pin = line_sens_L_Pin|line_sens_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : motor_2B_Pin */
  GPIO_InitStruct.Pin = motor_2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(motor_2B_GPIO_Port, &GPIO_InitStruct);

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
