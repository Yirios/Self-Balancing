///***********************************************
//鍏�鍙革細杞�瓒ｇ�戞妧(涓滆帪)鏈夐檺鍏�鍙�
//鍝佺墝锛歐HEELTEC
//瀹樼綉锛歸heeltec.net
//娣樺疂搴楅摵锛歴hop114407458.taobao.com 
//閫熷崠閫�: https://minibalance.aliexpress.com/store/4455017
//鐗堟湰锛歏1.0
//淇�鏀规椂闂达細2022-09-05

//Brand: WHEELTEC
//Website: wheeltec.net
//Taobao shop: shop114407458.taobao.com 
//Aliexpress: https://minibalance.aliexpress.com/store/4455017
//Version: V1.0
//Update锛�2022-09-05

//All rights reserved
//***********************************************/



//#ifndef __ELE_CCD_H
//#define	__ELE_CCD_H

//#include "sys.h"

///*******************************鐢电�佸贰绾緼DC**************************/
////PA4锛孭A5锛孭C5--IN4锛孖N5锛孖N15

//#define    ELE_ADC_APBxClock_FUN             	 RCC_APB2PeriphClockCmd
//#define    ELE_ADC                          	 ADC1
//#define    ELE_ADC_CLK                      	 RCC_APB2Periph_ADC1
//#define    ELE_ADC_GPIO_APBxClock_FUN       	 RCC_APB2PeriphClockCmd

////宸﹁矾鐢电�佸贰绾跨��鍙�A4
//#define    ELE_ADC_L_GPIO_CLK                	 RCC_APB2Periph_GPIOA  
//#define    ELE_ADC_L_PORT                     	 GPIOA
//#define    ELE_ADC_L_PIN                       	 GPIO_Pin_4

////涓�闂寸數纾佸贰绾跨��鍙�A5
//#define    ELE_ADC_M_GPIO_CLK                	 RCC_APB2Periph_GPIOA  
//#define    ELE_ADC_M_PORT                     	 GPIOA
//#define    ELE_ADC_M_PIN                       	 GPIO_Pin_5

////鍙宠矾鐢电�佸贰绾跨��鍙�C5
//#define    ELE_ADC_R_GPIO_CLK                	 RCC_APB2Periph_GPIOC  
//#define    ELE_ADC_R_PORT                     	 GPIOC
//#define    ELE_ADC_R_PIN                       	 GPIO_Pin_5


///********************************CCD宸＄嚎***************************/
////PA4--TSL_SI锛汸A5--TSL_CLK;PC5--ADC

////TSL_SI
//#define    TSL_SI_GPIO_CLK                	 	RCC_APB2Periph_GPIOA  
//#define    TSL_SI_PORT                     	 	GPIOA
//#define    TSL_SI_PIN                       	GPIO_Pin_4


////TSL_CLK
//#define    TSL_CLK_GPIO_CLK                	 	RCC_APB2Periph_GPIOA  
//#define    TSL_CLK_PORT                     	GPIOA
//#define    TSL_CLK_PIN                       	GPIO_Pin_5

////CCD_ADC
//#define    CCD_ADC_GPIO_CLK                	 	RCC_APB2Periph_GPIOC  
//#define    CCD_ADC_PORT                     	GPIOC
//#define    CCD_ADC_PIN                       	GPIO_Pin_5

//#define    CCD_ADC_APBxClock_FUN             	 RCC_APB2PeriphClockCmd
//#define    CCD_ADC                          	 ADC1
//#define    CCD_ADC_CLK                      	 RCC_APB2Periph_ADC1



//#define TSL_SI_HIGH								PAout(4) = 1
//#define TSL_SI_LOW								PAout(4) = 0
//#define TSL_CLK_HIGH							PAout(5) = 1
//#define TSL_CLK_LOW								PAout(5) = 0



//#define Barrier_Detected						1
//#define No_Barrier								0

//#define tracking_speed 0.3      //缁欏皬杞︿竴涓�澶ф��0.3m/s鐨勯�熷害
//#define Detect_distance 700//妫�娴嬭窛绂讳负700mm

//extern int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
//extern u16  CCD_ADV[128];
//extern u8 CCD_Median,CCD_Threshold;             


//void ELE_ADC_Init(void);
//void ELE_Mode(void);
//void CCD_Init(void);
//void Dly_us(void);
//void RD_TSL(void);
//void  Find_CCD_Median(void);
//void CCD_Mode(void);
//int CCD_turn(u8 CCD,float gyro);//CCD妯″紡杞�鍚戞帶鍒�
//int ELE_turn(int encoder_left,int encoder_right,float gyro);//ELE妯″紡杞�鍚戞帶鍒�

//u8 Detect_Barrier(void);


//#endif
