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
#include "max7219.h"
#include "max7219_matrix.h" // control de la matriz de leds
#include "piezas.h" // librería dode están definidas las piezas en hexadecimal
#include "menu.h"
#include "stdlib.h"

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

SPI_HandleTypeDef hspi2;

/*-----------------< TABLERO >--------------------*/

uint8_t matriz[16][8] = {
	{0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0}
};


uint8_t number[8];
char num_binario[8][8];
int num_dec[8];

/*-----------------< PIEZAS >--------------------*/

int8_t pieceRow; // Fila de la pieza.
int8_t pieceColumn; // Columna de la pieza.
int8_t pieceRotation;
uint8_t piezaActual[4][4];

/*-----------------< BOTONERA >--------------------*/
int mover;
int boton_OK;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

/*-----------------< POTENCIOMETRO >--------------------*/
uint32_t ADC_val_ini;
uint32_t ADC_val_fin;

/*-----------------< PARTIDA >--------------------*/
int juego;



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);

void ponerPieza(uint8_t pieza[4][4], int8_t row, int8_t col); //1
void eliminarPieza(uint8_t pieza[4][4], int8_t row, int8_t col);
void piezaNueva(void);
void dibujarMatriz(void);
void bajarMatriz(int8_t row);
void dropPiece(uint8_t piece[4][4]);
void moverIzq(uint8_t piece[4][4]);
void moverDcha(uint8_t piece[4][4]);
void filaCompleta(void);
_Bool posibilidadBajar(uint8_t pieza[4][4], int8_t row, int8_t col);
_Bool posibilidadIzq(uint8_t pieza[4][4], int8_t row, int8_t col);
_Bool posibilidadDcha(uint8_t pieza[4][4], int8_t row, int8_t col);
_Bool filaComp(int8_t row);
void gameOver(void);
void gameInit(void);

void ponerPieza(uint8_t pieza[4][4], int8_t row, int8_t col) // row and col means left top corner
{
	for(int8_t i = row;i < row + 4;i++)
	{
		for(int8_t j = col; j < col + 4; j++)
		{
			if (j < 9 && i < 17 && j >= 0)
			{
				 if(pieza[i - row][j - col] == true) matriz[i][j] = 1;
			}
		}
	}
}

void eliminarPieza(uint8_t pieza[4][4], int8_t row, int8_t col)
{
	for(int8_t i = row;i < row + 4;i++)
		{
			for(int8_t j = col; j < col + 4; j++)
			{
				if (j < 9 && i < 17 && j >= 0)
				{
					 if(pieza[i - row][j - col] == true) matriz[i][j] = 0;
				}
			}
		}
}

void eliminarFila(int8_t row)
{
	for(int8_t columna = 0; columna < 8 ;columna++)
		{
		  matriz[row][columna] = 0;
		}
}

void piezaNueva()
{
	pieceRow = -2;
	pieceColumn = 2;
	int numero_aleatorio = rand() % 7;

	switch(numero_aleatorio){

	case 0:
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					piezaActual[i][j] = pieza_O[i][j];
				}
			}
			break;

	case 1:
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					piezaActual[i][j] = pieza_T[i][j];
				}
			}
			break;

	case 2:
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					piezaActual[i][j] = pieza_S[i][j];
				}
			}
			break;

	case 3:
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					piezaActual[i][j] = pieza_Z[i][j];
				}
			}
			break;

	case 4:
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					piezaActual[i][j] = pieza_I[i][j];
				}
			}
			break;

	case 5:
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					piezaActual[i][j] = pieza_L[i][j];
				}
			}
			break;

	case 6:
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					piezaActual[i][j] = pieza_J[i][j];
				}
			}
			break;

	}

	ponerPieza(piezaActual, pieceRow, pieceColumn);
}

// Está función sirve para traducir los valores de la matriz en hexadecimal para poder mandarlos a la mtriz de leds
void dibujarMatriz()
{
	for (uint8_t j = 0; j < 8; j++) {
			  uint8_t numero1 = 0;
	          for (uint8_t i = 0; i < 8; i++) {
	              numero1 += matriz[i][j] * pow(2, i);
	          }
	          MAX7219_MatrixSetPixel(0, j, numero1) ;
	  }

	for (int j = 0; j < 8; j++) {
		      uint8_t numero2 = 0;
	          for (uint8_t i = 0; i < 8; i++) {
	              numero2 += matriz[i + 8][j] * pow(2, i );
	          }
	          MAX7219_MatrixSetPixel(1, j , numero2) ;
	  }

	  MAX7219_MatrixUpdate();
}

void bajarMatriz(int8_t row)
{
	for (uint8_t fila = row; fila > 0; fila--) {
	     for (uint8_t columna = 0; columna < 8; columna++) {
	            matriz[fila][columna] = matriz[fila - 1][columna];
	        }
	    }

    for (uint8_t columna = 0; columna < 8; columna++) {
    	 matriz[0][columna] = 0;
	   }
}

// Función para hacer caer la pieza de Tetris.
void dropPiece(uint8_t piece[4][4]) {
	if(posibilidadBajar(piece, pieceRow, pieceColumn) == false){
		filaCompleta(); // esta función comprueba si hay alguna fila completa y la elimina
		if( pieceRow <= -2) gameOver();
		else piezaNueva();
	}

	else{
	 eliminarPieza(piece, pieceRow, pieceColumn);
	 pieceRow++;
	 ponerPieza(piece, pieceRow, pieceColumn);
	 dibujarMatriz();
	 HAL_Delay(1000);
	}
}

// Función para mover la pieza de Tetris a la IZQUIERDA
void moverIzq(uint8_t piece[4][4]) {
	 if (posibilidadIzq(piece, pieceRow, pieceColumn) == true){
	 eliminarPieza(piece, pieceRow, pieceColumn);
	 pieceColumn--;
	 ponerPieza(piece, pieceRow, pieceColumn);
	 dibujarMatriz();
	 HAL_Delay(1000);
	 }
}

// Función para mover la pieza de Tetris a la DERECHA
void moverDcha( uint8_t piece[4][4]) {
	 if (posibilidadDcha(piece, pieceRow, pieceColumn) == true){
	 eliminarPieza(piece, pieceRow, pieceColumn);
	 pieceColumn++;
	 ponerPieza(piece, pieceRow, pieceColumn);
	 dibujarMatriz();
	 HAL_Delay(1000);
	 }
}

void filaCompleta()
{
	for (int8_t fila = 0; fila < 16; fila++) {
		if(filaComp(fila) == true){
			eliminarFila(fila);
			bajarMatriz(fila);
		}
	}
}

_Bool posibilidadBajar(uint8_t pieza[4][4], int8_t row, int8_t col) //return if can change position or rotate shape
{
	for (int8_t fila = 0; fila < 4; fila++) {
	   for (int8_t columna = 0; columna < 4; columna++) {
	            if (pieza[fila][columna] == 1 && pieza[fila + 1][columna] == 0) {
	            	int8_t filaT = fila + row + 1;
	            	int8_t columnaT = columna + col;
	                if (row + 3 == 15 || matriz[filaT][columnaT] == 1) {
	                    return false;
	                }
	            }
	        }
	    }
	 return true;
}


_Bool posibilidadIzq(uint8_t pieza[4][4], int8_t row, int8_t col) //return if can change position or rotate shape
{
	for (int8_t fila = 0; fila < 4; fila++) {
	   for (int8_t columna = 0; columna < 4; columna++) {
	            if (pieza[fila][columna] == 1 && pieza[fila][columna - 1] == 0){
	            	int8_t filaT = fila + row;
	            	int8_t columnaT = columna + col - 1;
	                if (columnaT < 0 || matriz[filaT][columnaT] == 1) {
	                    return false;
	                }
	            }
	        }
	    }
	 return true;
}

_Bool posibilidadDcha(uint8_t pieza[4][4], int8_t row, int8_t col) //return if can change position or rotate shape
{
	for (int8_t fila = 0; fila < 4; fila++) {
	   for (int8_t columna = 0; columna < 4; columna++) {
	            if (pieza[fila][columna] == 1 && pieza[fila][columna + 1] == 0){
	            	int8_t filaT = fila + row;
	            	int8_t columnaT = columna + col + 1;
	                if (columnaT >= 8 || matriz[filaT][columnaT] == 1) {
	                    return false;
	                }
	            }
	        }
	    }
	 return true;
}

// comprobación de si la fila que le metes a la función está completa o no
_Bool filaComp(int8_t row)
{
   for ( int8_t columna = 0; columna < 8; columna++) {
			   if( matriz[row][columna] == 0) return false;
   }
	return true;
}

void gameOver()
{
	for (int8_t i = 0; i < 8; i++) {
		for (int8_t j = 0; j < 8; j++) {
			matriz[i][j] = sad_face1[i][j];
		}
	}

	for (int8_t i = 8; i < 16; i++) {
		for (int8_t j = 0; j < 8; j++) {
			matriz[i][j] = sad_face2[i-8][j];
		}
	}

	dibujarMatriz();

	juego = 2;
}

void gameInit()
{
	for (int8_t i = 0; i < 8; i++) {
		for (int8_t j = 0; j < 8; j++) {
			matriz[i][j] = happy_face1[i][j];
		}
	}

	for (int8_t i = 8; i < 16; i++) {
		for (int8_t j = 0; j < 8; j++) {
			matriz[i][j] = happy_face2[i-8][j];
		}
	}

	dibujarMatriz();

}

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  uint16_t pin_e = GPIO_Pin;
  currentMillis = HAL_GetTick();

  if (pin_e == GPIO_PIN_1 && (currentMillis - previousMillis > 50) )
	 {
	  	mover = 1;// dcha
	  	previousMillis = currentMillis;
	  	return;
	 }
  if (pin_e == GPIO_PIN_2 && (currentMillis - previousMillis > 50) )
	 {
		 mover = 2; // izq
		 previousMillis = currentMillis;
		 return;
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
  MX_ADC1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  MAX7219_MatrixInit(&hspi2, SPI2_CS_GPIO_Port, SPI2_CS_Pin); //inicialización de los pines de la matriz
  MAX7219_MatrixUpdate();
  /* USER CODE END 2 */

  // COSAS DE INICIALIZACIÓN::
   dibujarMatriz();
   pieceRow = -2;
   pieceColumn = 2;
   boton_OK = 0;
   juego = 0;
   mover = 0;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			piezaActual[i][j] = pieza_O[i][j];
		}
	}

	srand(HAL_GetTick()); // para el numero random

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (juego == 0){
		  gameInit();
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1){
			  juego = 1;
			  for (int8_t i = 0; i < 16; i++) {
			  		eliminarFila(i);
			  }
		  }
	  }

	  while(juego==1){

		  mover = 0;

		  while(mover==0){
			  dropPiece(piezaActual);


			  if( mover== 1){
				  moverDcha(piezaActual);

			  }
			  else if (mover == 2){
				  moverIzq(piezaActual);

			  }

	  // Lectura del potenciometro
	   //HAL_Delay(10);
	 /* HAL_ADC_Start(&hadc1);
	   if (HAL_ADC_PollForConversion(&hadc1, 100)==HAL_OK)
	   {
		   ADC_val_fin = HAL_ADC_GetValue(&hadc1);
	   }
	   HAL_ADC_Stop(&hadc1);

	   if(ADC_val_fin > ADC_val_ini + 50){
		   moverDcha(piezaActual);
		   ADC_val_ini = ADC_val_fin;
	   }
	   else if (ADC_val_fin < ADC_val_ini - 50){
		   moverIzq(piezaActual);
		   ADC_val_ini = ADC_val_fin;
	   }

	 }*/

		  }
	  }
  /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
