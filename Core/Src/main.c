/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	PWM_ARR = 0,
	PWM_CCR,
}command_change_pwm;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAPTURE_FREQ 1000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t duration_time = 0;
uint16_t falling_time = 0;
bool count_ic = false;
bool capture_start = true;
bool capture_flag = false;
uint8_t uart_send_buffer[32];

uint8_t uart_rx_data = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void uart_send(uint8_t uart_num,char *fmt,...);
void cal_pulse();
void pwm_change(TIM_HandleTypeDef *htim,uint16_t arr_cal_val,uint16_t ccr_cal_val,command_change_pwm command);

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if(huart->Instance==huart1.Instance){
		 HAL_UART_Receive_DMA(huart, &uart_rx_data, 1);
	 }
	 UNUSED(huart);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim9){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			 if((htim->Instance->CCER & 0x02) == 0x02){//falling
					if (!count_ic){
						htim->Instance->CNT = 0;
						__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
					}
					else{
							duration_time = htim->Instance->CCR1;
							capture_flag = true;
							count_ic = false;
					}
			 }
			 else if((htim->Instance->CCER & 0x02) == 0x00){//rising
					count_ic=true;
					falling_time = htim->Instance->CCR1;
					htim->Instance->CNT = 0;
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
			 }
		}
	}
  UNUSED(htim);
}

void cal_pulse(){
		uint32_t duty_cycle;
    uint32_t hz;
    duty_cycle = (duration_time*1000)/(duration_time+falling_time);
    hz = CAPTURE_FREQ/(duration_time+falling_time);
    uart_send(1,"LOW:%d HIGH:%d ",falling_time,duration_time);
    uart_send(1,"%luhz ",hz);
    uart_send(1,"%lu.%lu%%\n",duty_cycle/10,duty_cycle-(duty_cycle/10)*10);
}

void uart_send(uint8_t uart_num,char *fmt,...)
{
    va_list arg;
    va_start(arg,fmt);
    vsnprintf((char *)uart_send_buffer,32,fmt,arg);

    if(uart_num==1){
        HAL_UART_Transmit(&huart1,uart_send_buffer,32,10);
    }

    va_end(arg);
    memset(uart_send_buffer,0,32);
}

void pwm_change(TIM_HandleTypeDef *htim,uint16_t arr_cal_val,uint16_t ccr_cal_val,command_change_pwm command){
	if(command==PWM_ARR){
		uart_send(1,"ARR_MODE\n");
		while(1){
			if(uart_rx_data == '+'){
				htim->Instance->ARR = (htim->Instance->ARR>65535-arr_cal_val) ?
						htim->Instance->ARR : htim->Instance->ARR + arr_cal_val;
				uart_send(1,"ARR : %d\n",htim->Instance->ARR);
				uart_rx_data = 0;
			}
			else if(uart_rx_data == '-'){
				htim->Instance->ARR = (htim->Instance->ARR > arr_cal_val)&&(htim->Instance->ARR > arr_cal_val + htim->Instance->CCR1) ?
						htim->Instance->ARR - arr_cal_val: htim->Instance->ARR;
				uart_send(1,"ARR : %d\n",htim->Instance->ARR);
				uart_rx_data = 0;
			}
			else if(uart_rx_data == 'q'){
				uart_send(1,"break\n");
				break;
			}
		}
	}

	else if(command==PWM_CCR){
		uart_send(1,"CCR_MODE\n");
		while(1){
			if(uart_rx_data == '+'){
				htim->Instance->CCR1 = (htim->Instance->CCR1 + ccr_cal_val >= htim->Instance->ARR) ?
						htim->Instance->CCR1 : htim->Instance->CCR1 + ccr_cal_val;
				uart_send(1,"CCR1 : %d\n",htim->Instance->CCR1);
				uart_rx_data = 0;
			}
			else if(uart_rx_data == '-'){
				htim->Instance->CCR1 = (htim->Instance->CCR1 > ccr_cal_val) ?
						htim->Instance->CCR1 - ccr_cal_val : htim->Instance->CCR1;
				uart_send(1,"CCR1 : %d\n",htim->Instance->CCR1);
				uart_rx_data = 0;
			}
			else if(uart_rx_data == 'q'){
				uart_send(1,"break\n");
				break;
			}
		}
	}
}
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
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA(&huart1, &uart_rx_data, 1);
  HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  uint32_t pretime=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if(uart_rx_data=='a'||uart_rx_data=='A'){
  		pwm_change(&htim3, 1000, 1000, PWM_ARR);
  		uart_rx_data = 0;
  	}
  	else if(uart_rx_data=='c'||uart_rx_data=='C'){
  		pwm_change(&htim3, 1000, 1000, PWM_CCR);
  		uart_rx_data = 0;
  	}
  	if(HAL_GetTick()-pretime>500){
  		cal_pulse();
  		pretime=HAL_GetTick();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
