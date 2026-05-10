/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/
#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265							//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  500.0 	//编码器精度 500线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm
#define Wheel_spacing 160         //轮距，单位mm

#define DIFFERENCE 100

#define INT PBin(9)   //PB9连接到MPU6050的中断引脚

//小车各模式定义
#define Normal_Mode							0
#define Lidar_Avoid_Mode					1
#define Lidar_Follow_Mode					2
#define Lidar_Straight_Mode       			3
#define ELE_Line_Patrol_Mode				4
#define CCD_Line_Patrol_Mode				5
#define ROS_Mode							6

/*******************雷达避障宏定义**************************/
#define Avoid_ON 	1//雷达避障开启
#define Avoid_OFF 	0//雷达避障关闭
//避障模式的参数
#define  avoid_Distance 450//避障距离300mm（触发避障的距离）
#define Avoid_Min_Distance 250	//最小避障距离（触发后退的距离）
#define avoid_Angle1 50 //避障的角度，在310~360、0~50°的范围
#define avoid_Angle2 310	

//避障状态定义
#define	Avoid_Stop  0	//不避障
#define	Avoid_Left  1	//左避
#define	Avoid_Right 2	//右避
#define	Avoid_Back  3	//后退
/***********************************************************/
/*********遥控速度**********/
#define movement_speed 0.05 //前进后退的目标角度的步距，即运动速度
#define turn_speed 0.02	////转向的目标角度的步距，即转向速度
/***************************/
//雷达走直线的参数
#define Initial_speed 0.2 //小车的初始速度大概为200mm每秒
#define Limit_time 500    //限制时间，5ms中断*数值=时间 ，这里就是3s
#define refer_angle1  71  //参照物的角度1
#define refer_angle2  74  //参照物的角度2

//雷达跟随参数
#define Follow_distance 1500  //雷达跟随模式最远距离


int Balance(float angle,float gyro);
int Velocity(int encoder_left,int encoder_right);
int Turn(float gyro);
void Set_Pwm(int motor_left,int motor_right);
void Limit_Pwm(void);
int PWM_Limit(int IN,int max,int min);
u8 Turn_Off(float angle, int voltage);
void Middle_angle_Check(void);
void Get_Angle(u8 way);
int myabs(int a);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right,int Angle_ADC_Bias);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
void Choose(int encoder_left,int encoder_right);
void Read_distance(void);
void Select_Zhongzhi(void);
void Normal(void);
int Lidar_Avoid(void);
extern short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
extern float Target_x_speed, Target_angle_x, Target_gyro_z;
static int Incremental_L(float CurrentVal,float TargetVal);
static int Incremental_R(float CurrentVal,float TargetVal);
static float angle_count(float Angle_ADC, float mid);
extern float Target_theta_L, Target_theta_R, Target_theta_L_dot, Target_theta_R_dot, Target_theta_1;
extern float theta_1,theta_2, last_theta_2, theta_dot_2, last_theta_dot_2;	
extern int Moto_Ki, Moto_Kp;//PI控制器系数
extern float L_Bias,R_Bias;
extern int Bias_interval;
#endif

