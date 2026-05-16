/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "sys.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;
extern short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
extern float Velocity_Left,Velocity_Right;	//车轮速度(mm/s)
#define CONTROL_DELAY		1000
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	if(Flag_Show==0)
	{	
	  while((USART3->SR&0X40)==0);//Flag_Show=0  使用串口3   
	  USART3->DR = (u8) ch;      
	}
	else
	{	
		while((USART1->SR&0X40)==0);//Flag_Show!=0  使用串口1   
		USART1->DR = (u8) ch;      
	}	
	return ch;
}

#endif
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart1_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 230400;
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
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE); //使能IDLE中断
	HAL_UART_Receive_DMA(&huart4,receive_buff,255);

  /* USER CODE END UART4_Init 2 */

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
	HAL_UART_Receive_IT(&huart5,&temp_data,sizeof(temp_data));
  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

uint8_t usart1_rx_byte;  // single-byte RX buffer for ext-ctrl

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
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
/* USART2 init function (ST-Link VCP, binary telemetry) */

void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf)); //打开串口3接收中断
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA2_Channel3;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */
    /* USART1 DMA TX Init (binary telemetry via USB) */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 DMA TX Init (binary telemetry, non-blocking) */
    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart3_tx);
    __HAL_LINKDMA(&huart3, hdmatx, hdma_usart3_tx);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
    /* USART2 peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

///**************************************************
//Function:Serial port 1 receivees interrupted
//Input:None
//Output:None
//函数：串口一中断
//入口参数：无
//放回值：无
//**************************************************/
//void USART1_IRQHandler(void)
//{
//	static u8 Count=0;
//	u8 Usart_Receive;
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)//cheak if data is receives 判断是否接收到数据
//	{
//		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
//		Usart_Receive = USART_ReceiveData(USART1);//Read Data 读数据
//		Mode=ROS_Mode;//ros控制时，将小车模式设为ROS模式
//////		if(Time_count < CONTROL_DELAY)
//////		{
//////			//Data is not processed until 10 seconds after startup
//////			//开机十秒前不允许接收数据
//////		}
//		Receive_Data.buffer[Count] = Usart_Receive;//Fill the array with serial data  串口数据填入数组
//		
//		//Ensure that the first data is the array is FRAME_HEADER
//		//确保第一个数据时帧头
//		if(Usart_Receive == FRAME_HEADER ||Count>0)
//			Count++;
//		else Count=0;
//		if(Count == 11) //Verify the length of the packet  验证数据包的长度
//		{
//			Count = 0;
//			if(Receive_Data.buffer[10] == FRAME_TAIL)
//			{
//				if(Receive_Data.buffer[9] == Check_Sum(9,0))
//				{
//					//Calculate the 3-axis target velocity from the serial data,which is divided into 8-bit high and 8-bit low units mm/s
//					Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);//平衡小车没有运用到运动学正解逆解
//					Move_Z=-XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8])/1000*160/2/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision/Perimeter;//Move_Z是直接作用在转向环
//					//Move_Z=ROS给的弧度/1000*轮距/2/周长/编码器读取频率*频数*减速比*精度；
//					Ros_Rate=0;  //只要中断进来，就把它置0，防止小车控制的卡顿
//				}
//			}
//		}
//	}
//}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //帧头
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //帧尾
	
	//According to different vehicle types, different kinematics algorithms were selected to carry out the forward kinematics solution, 
	//and the three-axis velocity was obtained from each wheel velocity
	//根据运动学算法进行运动学正解，从各车轮速度求出三轴速度
	
	Send_Data.Sensor_Str.X_speed = ((Velocity_Left+Velocity_Right)/2); //平衡小车的速度是mm/s
	Send_Data.Sensor_Str.Y_speed = 0;
	Send_Data.Sensor_Str.Z_speed = ((Velocity_Right-Velocity_Left)/160)*1000;//160为轮距
	
	
	//The acceleration of the triaxial acceleration //加速度计三轴加速度
	Send_Data.Sensor_Str.Accelerometer.X_data= -Accel_Y; //The accelerometer Y-axis is converted to the ros coordinate X axis //加速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Accelerometer.Y_data=Accel_X; //The accelerometer X-axis is converted to the ros coordinate y axis //加速度计X轴转换到ROS坐标Y轴
	Send_Data.Sensor_Str.Accelerometer.Z_data= Accel_Z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //加速度计Z轴转换到ROS坐标Z轴
	
	//The Angle velocity of the triaxial velocity //角速度计三轴角速度
	Send_Data.Sensor_Str.Gyroscope.X_data= -Gyro_Y*4; //The Y-axis is converted to the ros coordinate X axis //角速度计Y轴转换到ROS坐标X轴
	Send_Data.Sensor_Str.Gyroscope.Y_data= Gyro_X*4; //The X-axis is converted to the ros coordinate y axis //角速度计X轴转换到ROS坐标Y轴
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //如果电机控制位使能状态，那么正常发送Z轴角速度
		Send_Data.Sensor_Str.Gyroscope.Z_data=Gyro_Z*4;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;        
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//电池电压(这里将整数放大十倍传输，相应的在接收端在接收到数据后会缩小一千倍得到正常显示的电压，如检测Voltage为1140，放大十倍后再缩小一千倍为11.40)
	Send_Data.Sensor_Str.Power_Voltage = (float)Voltage*10; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //帧头
  Send_Data.buffer[1]=Flag_Stop; //Car software loss marker //小车软件失能标志位
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//小车三轴速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed>>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed>>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU加速度计三轴加速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU角速度计三轴角速度,各轴都拆分为两个8位数据再发送
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//电池电压,拆分为两个8位数据发送
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage>>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //数据校验位计算，模式1是发送数据校验
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //帧尾
}


/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	short transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return transition; 
								
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
函数功能：串口1发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
函数功能：计算要发送/接收的数据校验结果
入口参数：Count_Number：校验的前几位数；Mode：0-对接收数据进行校验，1-对发送数据进行校验
返回  值：校验结果
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//对要发送的数据进行校验
	if(mode==1)
	for(k=0;k<Count_Number;k++)
	{
	  check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//对接收到的数据进行校验
	if(mode==0)
	for(k=0;k<Count_Number;k++)
	{
		check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}
/* USER CODE END 1 */
