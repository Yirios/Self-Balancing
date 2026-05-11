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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
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

/* USER CODE BEGIN PV */
u8 Pick_up_stop=0;                          //检查是否被拿起标志位
int Middle_angle=0;                         //机械中值默认为0
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
u16 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2,Target_Velocity=30; //蓝牙遥控相关的变量
float RC_Velocity,RC_Turn_Velocity;			    //遥控控制的速度
u8 Flag_Stop=1,Flag_Show=0;                 //电机停止标志位和显示标志位  默认停止 显示打开
u8 PS2_ON_Flag = 0;		                      //默认所有方式不控制
u8 Mode = 0;								                //模式选择，默认是普通的控制模式
float Move_X,Move_Z;                        //控制小车避障、跟随时前进的变量，转弯的变量
u16 determine;                              //雷达跟随模式的一个标志位
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Motor_Left,Motor_Right;                 //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //温度变量
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
u32 Distance;                               //雷达测距
u8 delay_50,PID_Send; 						//延时和调参相关变量
volatile u8 delay_flag;
u8 Flag_follow=0,Flag_avoid=0,Flag_straight=0;							//雷达跟随、雷达避障标志位
u8 Lidar_Detect = Lidar_Detect_ON;			//电磁巡线模式雷达检测障碍物，默认开启
float Acceleration_Z;                       //Z轴加速度计  
u8 CCD_Zhongzhi,CCD_Yuzhi;                 //线性CCD相关
float Balance_Kp=27000,Balance_Kd=110,Velocity_Kp=400,Velocity_Ki=2,Turn_Kp=4200,Turn_Kd=100;//PID参数（放大100倍）
u16 Angle_ADC = 0;
int Avoid_Flag=0;//遥控时雷达避障开启标志位
int Lidar_Online_Flag=0,Lidar_Online_Cnt=0;
volatile int init_cnt=0,APP_ON=0;
volatile int Steady_Flag,time_cnt=0;//稳态标定



volatile bool answer_flag = true;



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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试
	delay_init();                   //延迟函数初始化
	BEEP_GPIO_Config();							//蜂鸣器初始化
	OLED_Init();										//OLED初始化
	MPU6050_initialize();           //MPU6050初始化
	DMP_Init();                     //初始化DMP
  /* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		if(Flag_Show==0)          		//使用MiniBalance APP和OLED显示屏
			{
				static u16 frame_cnt = 0;
				frame_cnt++;

				if (answer_flag) {
					APP_Show();        // 发送状态格式数据
					answer_flag = false;
				}
				// APP_Show();								//发送数据给APP

				PS2_Read();							//手柄数据读取（每圈必须读）

				// OLED 跳帧：每 10 圈刷一次，主循环从 ~30ms 降到 ~5ms
				if (frame_cnt % 10 == 0) {
					oled_show();          		//显示屏打开
				}
			}
		else                      		//使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
		{
			DataScope();          			//开启MiniBalance上位机
		}
		if(Lidar_Deal_Flag){			//成功接收雷达一圈的点云
			Lidar_data_Deal();			//雷达数据处理
			Lidar_Deal_Flag=0;			
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

