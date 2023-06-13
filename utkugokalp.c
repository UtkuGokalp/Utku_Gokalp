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
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define INPUT_COUNT 2
#define COMBINATION_COUNT 4
#define ADC_RESOLUTION_MAX_VALUE 4096.0
#define COMMAND_MAX_WORD_COUNT 20
#define COMMAND_OUTPUT_BUFFER_SIZE 100
#define GetArraySize(a) (sizeof(a) / sizeof(*a))

typedef struct InOutData
{
	int inputs[INPUT_COUNT];
	int output;
} InOutData;

typedef void(*CommandFunc)(char* commandOutputBuffer, char* words[COMMAND_MAX_WORD_COUNT]);
typedef struct Command
{
	const char* command;
	CommandFunc func;
} Command;

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

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
double bias = 0.5;
double learningRate = 0.1;
bool trainingComplete = false; //Variable to use for the USB command "retrain"

InOutData combinations[COMBINATION_COUNT] =
{
	(InOutData){ .inputs = { 0, 0 }, .output = 0 }, //Green
	(InOutData){ .inputs = { 0, 1 }, .output = 0 }, //Orange
	(InOutData){ .inputs = { 1, 0 }, .output = 0 }, //Red
	(InOutData){ .inputs = { 1, 1 }, .output = 1 }, //Blue
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Gets the average of the last 100 ADC readings
double ReadADCValue(ADC_HandleTypeDef* adc)
{
	double adcReadingsBuffer[100] = { 0 };
	for (int i = 0; i < 100; i++)
	{
		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
		{
			adcReadingsBuffer[i] = (double)HAL_ADC_GetValue(adc);
			HAL_ADC_Start(adc);
		}
	}
	
	double sum = 0;
	for (int i = 0; i < 100; i++)
	{
		sum += adcReadingsBuffer[i];
	}
	return sum / 100;
}

//These variables are only for monitoring the weight values from STMCubeMonitor
double weight1_FOR_MONITOR = 0;
double weight2_FOR_MONITOR = 0;
void UpdatePWMDutyCycle(TIM_HandleTypeDef* tim, double percent)
{
	tim->Instance->CCR2 = (uint32_t)(percent * tim->Instance->ARR);
	
	if (tim->Instance == htim3.Instance)
	{
		weight1_FOR_MONITOR = (double)(uint32_t)(percent * tim->Instance->ARR);
	}
	if (tim->Instance == htim5.Instance)
	{
		weight2_FOR_MONITOR = (double)(uint32_t)(percent * tim->Instance->ARR);
	}
}

double GetWeight(int index)
{
	return ReadADCValue(index == 0 ? &hadc1 : &hadc2) / ADC_RESOLUTION_MAX_VALUE; 
}

void SetWeight(int index, double value)
{
	UpdatePWMDutyCycle(index == 0 ? &htim3 : &htim5, value);
}

int Decide(int* inputs)
{
	double sum = 0;
	for (int i = 0; i < INPUT_COUNT; i++)
	{
		sum += inputs[i] * GetWeight(i);
	}
	sum += bias;
	return sum >= 0 ? 1 : 0;
}

//Trains the perceptron once for every input-output combination
void TrainOnce(InOutData* combinations)
{
	for (int i = 0; i < COMBINATION_COUNT; i++)
	{
		int decision = Decide(combinations[i].inputs);
		double error = combinations[i].output - decision;
		bias += learningRate * error;
		SetWeight(0, GetWeight(0) + learningRate * error * combinations[i].inputs[0]);
		SetWeight(1, GetWeight(1) + learningRate * error * combinations[i].inputs[1]);
	}
}

//Returns the result of the decision
int UpdateLEDState(InOutData* combinations, int index, uint16_t pin)
{
	int result = Decide(combinations[index].inputs);
    HAL_GPIO_WritePin(GPIOD, pin, result == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return result;
}

void GetWordsFromCommand(char* command, char* words[COMMAND_MAX_WORD_COUNT])
{
	char* word = strtok(command, " ");
	words[0] = word;
	int index = 1;
	while (word != NULL && index < COMMAND_MAX_WORD_COUNT - 1)
	{
        words[index++] = word = strtok(NULL, " ");
	}
}

void Retrain(char* commandOutputBuffer, char* words[COMMAND_MAX_WORD_COUNT])
{
	if (trainingComplete)
	{
		trainingComplete = false;
		snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "Retraining.\n");
	}
	else
	{
		snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "Previous training not complete.\n");
	}
}

void WeightCommand(int index, char* commandOutputBuffer, char* words[COMMAND_MAX_WORD_COUNT])
{
	if (strcmp(words[1], "get") && strcmp(words[1], "set"))
	{
		snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "get or set is expected as the first argument.\n");
		return;
	}
	if (!strcmp(words[1], "set"))
	{
		SetWeight(index, atof(words[2]));
	}
	snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "weight%d is %f.\n", index, GetWeight(index));
}

void Weight0Command(char* commandOutputBuffer, char* words[COMMAND_MAX_WORD_COUNT])
{
	WeightCommand(0, commandOutputBuffer, words);
}

void Weight1Command(char* commandOutputBuffer, char* words[COMMAND_MAX_WORD_COUNT])
{
	WeightCommand(1, commandOutputBuffer, words);
}

void BiasCommand(char* commandOutputBuffer, char* words[COMMAND_MAX_WORD_COUNT])
{
	if (strcmp(words[1], "get") && strcmp(words[1], "set"))
	{
		snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "get or set is expected as the first argument.\n");
		return;
	}
	if (!strcmp(words[1], "set"))
	{
		bias = atof(words[2]);
	}
	snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "bias is %f.\n", bias);
}

void LearningRateCommand(char* commandOutputBuffer, char* words[COMMAND_MAX_WORD_COUNT])
{
	if (strcmp(words[1], "get") && strcmp(words[1], "set"))
	{
		snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "get or set is expected as the first argument.\n");
		return;
	}
	if (!strcmp(words[1], "set"))
	{
		learningRate = atof(words[2]);
	}
	snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "learningRate is %f.\n", learningRate);
}

void InOutCommand(char* commandOutputBuffer, char* words[COMMAND_MAX_WORD_COUNT])
{
	if (strcmp(words[1], "get") && strcmp(words[1], "set"))
	{
		snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "get or set is expected as the first argument.\n");
		return;
	}
	if (!strcmp(words[1], "set"))
	{
		combinations[0].output    = atoi(words[2]);
		combinations[1].output    = atoi(words[3]);
		combinations[2].output    = atoi(words[4]);
		combinations[3].output    = atoi(words[5]);
	}
	snprintf(commandOutputBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "%d %d %d\n%d %d %d\n%d %d %d\n%d %d %d\n",
		    		combinations[0].inputs[0], combinations[0].inputs[1], combinations[0].output,
		    		combinations[1].inputs[0], combinations[1].inputs[1], combinations[1].output,
					combinations[2].inputs[0], combinations[2].inputs[1], combinations[2].output,
					combinations[3].inputs[0], combinations[3].inputs[1], combinations[3].output);
}

//This function is called from the CDC_Receive_FS interrupt function.
void ProcessUSBCommand(char* buffer, uint32_t length)
{
	//retrain doesn't have any arguments. Other commands are used as follows
	//<command> get prints out the current value(s)
	//<command> set <value(s)> sets the value(s)
	Command commands[] =
	{
	    (Command) { .command = "retrain", .func = Retrain },
		(Command) { .command = "weight0", .func = Weight0Command },
		(Command) { .command = "weight1", .func = Weight1Command },
		(Command) { .command = "bias", .func = BiasCommand },
		(Command) { .command = "learningRate", .func = LearningRateCommand },
		(Command) { .command = "inout", .func = InOutCommand },
	};
	char transmitBuffer[COMMAND_OUTPUT_BUFFER_SIZE + 1] = { 0 };
	char* words[COMMAND_MAX_WORD_COUNT] = { 0 };
	//Buffer for copying the usb buffer contents. free'd at the end of the function
	char* tempBuffer = calloc(length + 1, sizeof(char)); 
	for (int i = 0; i < length; i++)
	{
		tempBuffer[i] = buffer[i];
	}
	memset(buffer, 0, length);
	GetWordsFromCommand(tempBuffer, words);
	int wordCount = 0;
	while (words[wordCount] != NULL)
	{
		wordCount++;
	}
	
	if (wordCount > 0)
	{
		int i = 0;
		for (; i < GetArraySize(commands); i++)
		{
			Command cmd = commands[i];
			if (!strcmp(words[0], cmd.command))
			{
				if (cmd.func == NULL)
				{
					snprintf(transmitBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "No function defined to be executed for the command.\n");
					break;
				}
				cmd.func(transmitBuffer, words);
				break;
			}
		}
		if (i == GetArraySize(commands))
		{
			snprintf(transmitBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "No matching command found. (given command: %s)\n", words[0]);
		}
	}
	else
	{
		snprintf(transmitBuffer, COMMAND_OUTPUT_BUFFER_SIZE, "No words found in given command.\n");
	}
	CDC_Transmit_FS((uint8_t*)transmitBuffer, strlen(transmitBuffer));
	free(tempBuffer);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*
	TIM3 (PA7) should be connected to ADC1 (PA0)
	TIM5 (PA1) should be connected to ADC2 (PA5)
	*/
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
  MX_ADC1_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Assign random values for the duty cycles
  UpdatePWMDutyCycle(&htim3, 0.54);
  UpdatePWMDutyCycle(&htim5, 0.68);
  while (1)
  {
	  if (!trainingComplete)
	  {
		  TrainOnce(combinations);
		  	  
		  /*
		  PIN12 -  Green  (comb0)
		  PIN13 - Orange  (comb1)
		  PIN14 -    Red  (comb2)
		  PIN15 -   Blue  (comb3)
		  */
		  int decision0 = UpdateLEDState(combinations, 0, comb0LED_Pin);
		  int decision1 = UpdateLEDState(combinations, 1, comb1LED_Pin);
		  int decision2 = UpdateLEDState(combinations, 2, comb2LED_Pin);
		  int decision3 = UpdateLEDState(combinations, 3, comb3LED_Pin);
		  	  
		  if (decision0 == combinations[0].output &&
		      decision1 == combinations[1].output &&
		      decision2 == combinations[2].output &&
		      decision3 == combinations[3].output)
		  {
		      //Signal training complete
			  HAL_Delay(500);
			  for (int i = 0; i < 5; i++)
			  {
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, i % 2);
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, i % 2);
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, i % 2);
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, i % 2);
				  HAL_Delay(500);
			  }
			  //Show the results again
		  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, decision0);
		  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, decision1);
		  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, decision2);
		  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, decision3);
		      trainingComplete = true;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, comb0LED_Pin|comb1LED_Pin|comb2LED_Pin|comb3LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : comb0LED_Pin comb1LED_Pin comb2LED_Pin comb3LED_Pin */
  GPIO_InitStruct.Pin = comb0LED_Pin|comb1LED_Pin|comb2LED_Pin|comb3LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
