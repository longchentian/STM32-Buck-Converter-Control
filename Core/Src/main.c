/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "hrtim.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "protocol.h"
#include "aw_pid.h"
#include "sp_pid.h"
#include "eh_pid.h"
#include "buck_pid_contral.h"
#include <string.h>
#include "fuzzy_pid.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define PID_ASSISTANT_EN 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define CH_COUNT 2
//#define PID_ASSISTANT_EN 1


static float data[CH_COUNT];

static unsigned char tail[12]={0,0,0,0,0,0,0,0,0x00, 0x00, 0x80, 0x7f};

static float data[CH_COUNT];


struct Frame {
    float fdata[CH_COUNT];
    unsigned char tail[4];
};

struct Frame real_data ={{0}, {0x00, 0x00, 0x80, 0x7f}};

#define FUZZYPID 1
#define DOF 1


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t temp;
float man;
float val;
uint16_t duty_val =1000;


float VoltageRef1 = 3.3;
float VoltageRef2 = 7.5;
float Ts = 0.001;
int target_val;
int real_val;
uint16_t Ref_scaler = 10000;

float control_uk;

uint16_t Isr_count=0;


  

struct PID **pid_vector;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	struct PID **subpid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, 1);

	pid_vector=subpid_vector;
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_HRTIM_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	/*协议初始化*/
  //protocol_init();


   ///*PID参数初始化*
#if defined(PID1)  
   /*抗饱和PID初始化*/
	pid.Kp = 10.4597502;
    pid.Ki = 5.0053997;
    pid.Kd = 8.59500027;
    pid.kw = 0;
    pid.Kc = 0;
    pid.Ts = Ts;
    pid.N = 20;
	pid.OutMax = 3750;
    pid.OutMin =  250;
	
    pid.SatErr = 0.005;
    pid.Ref=VoltageRef1;

#elif defined(PID2)  
	/*增量式PID初始化*/
    pid.Kp = 0.459750444;
    pid.Ki = 0.00539999164;
    pid.Kd = 0.595000153;
	
	pid.Kp = 5.4597502;
    pid.Ki = 5.0053997;
    pid.Kd = 8.59500027;
	
    pid.Ts = Ts;
//    pid.OutMax = 11;
//    pid.OutMin =  1;
	
	pid.OutMax = 3750;
    pid.OutMin =  250;
    pid.Ref = VoltageRef2;
		
    pid.init(&pid);
	

#elif defined(PID3)	
	/*专家PID初始化*/
    pid.Kp = 0.0459750444;
    pid.Ki = 0.000539999164;
    pid.Kd = 0.00595000153;
    pid.Ts = Ts;
	
	pid.Kp = 10.4597502;
    pid.Ki = 5.0053997;
    pid.Kd = 8.59500027;
	
	
	
//    pid.OutMax = 11;
//    pid.OutMin =  1;
//    pid.Ref = VoltageRef2;
	
	pid.OutMax = 3750;
    pid.OutMin =  250;
    pid.Ref = VoltageRef2;
	
	

#endif	

	#if defined(PID_ASSISTANT_EN)
	  float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
	  set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值
	#endif
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		HAL_Delay(2000);
//		__HAL_HRTIM_SETCOMPARE(&hhrtim,HRTIM_TIMERINDEX_TIMER_A,HRTIM_COMPAREUNIT_1,duty_val);
//		temp=__HAL_HRTIM_GETCOMPARE(&hhrtim,HRTIM_TIMERINDEX_TIMER_A,HRTIM_OUTPUT_TA1);
////		printf("compare=%d\n",temp);
//		temp=get_ad();//正常采集
//		man=(float) temp;
//		man=KalmanFilter(man);//卡尔曼滤波
//		HAL_Delay(1000);
//		man = Kalman2();
//		//val = man/65535*3.3*5;
//		
//		val = man*3.3f/13107;
//		
//		data[1]=val;
//		real_data.fdata[1]=val;
////		printf("卡尔曼滤波电压=%f\n",val);
//		temp=filter2();
//		
//		//val = temp/65335*3.3*5;
//		val = temp*3.3f/13107;
////		printf("中值滤波电压=%f\n",val);
//		
//		data[0]=val;
//		real_data.fdata[0]=val;
//		
//		
//		memcpy(tail, (uint8_t *)&data, sizeof(data));
//		
//		HAL_UART_Transmit(&huart1,(uint8_t *)&real_data, sizeof(real_data), 0xFFFFF);
//		//HAL_UART_Transmit(&huart1,(uint8_t *)&tail, sizeof(tail), 0xFFFFF);
//		
//		duty_val+=500;
//		if(duty_val>3500)
//			duty_val=1000;
		//target=duty_val;
			  /* 设置目标位置 */
		




		
//		receiving_process();

		
		
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
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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

/* USER CODE BEGIN 4 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	uint8_t data[1];
//    data[0] = (*huart).Instance->RDR;
//    protocol_data_recv(data, 1);
//}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(TIM6 == htim->Instance)
	{
		
		Isr_count++;
	
//		temp=get_ad();//正常采集
//		man=(float) temp;
//		man=KalmanFilter(man);//卡尔曼滤波
//		man=man*3.3f/13107;
		
		temp=filter2();//中值滤波
		man = temp*3.32f/13107;
//		man = (32768.0f-(float)temp)/65535.0f*3.30f*17.0f;
		
		
		real_val = (int)(man*1000);	
		
		
		#ifdef FUZZYPID 
			control_uk = fuzzy_pid_motor_pwd_output(man, pid.Ref, true, pid_vector[0]);
			if(control_uk<250)
				control_uk=250;
			else if (control_uk>3750)
				control_uk=3750;
		
		#else
			pid.Fdb = man;	
			pid.calc(&pid);
			control_uk = pid.Out;
		#endif
		
	
//		duty_val = (int) (control_uk/12*4000);
		duty_val = (int)control_uk;
		
		__HAL_HRTIM_SETCOMPARE(&hhrtim,HRTIM_TIMERINDEX_TIMER_A,HRTIM_COMPAREUNIT_1,duty_val);	
		

		#if defined(PID_ASSISTANT_EN)
			set_computer_value(SEND_FACT_CMD, CURVES_CH1, &real_val, 1);     // 给通道 1 发送实际值
			target_val =  (int)( pid.Ref*1000);
			set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_val, 1);     // 给通道 1 发送目标值
			
		#else
			real_data.fdata[0]=pid.Ref;
			real_data.fdata[1]=man;
			HAL_UART_Transmit(&huart1,(uint8_t *)&real_data, sizeof(real_data), 0xFFFFF);
			
		#endif  
		
		/*测试用*/

//		
//		if(Isr_count>10000)
//		{
//			Isr_count=0;
//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//		}

		if(Isr_count>=Ref_scaler)
	   {
		   Isr_count=0;
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

			if(pid.Ref==VoltageRef2)
			{
				
				pid.Ref=VoltageRef1;   // 电压给定 轮换
				#if defined(PID2)
					pid.init(&pid);
				#endif
				
				
			}
			else
			{
				pid.Ref=VoltageRef2;
				#if defined(PID2)
					pid.init(&pid);
				#endif
			}

	   }

		
	}
		
	
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
