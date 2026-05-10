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


//#include "ELE_CCD.h"

//int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
//u16  CCD_ADV[128]={0};
//u8 CCD_Median,CCD_Threshold;                 //绾挎��CCD鐩稿叧

////鐢电�佸贰绾緼DC绔�鍙ｅ垵濮嬪寲
///**************************************************************************
//Function: ELE_ADC_GPIO_Config
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛氬垵濮嬪寲鐢电�佸贰绾縂PIO
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	
//static void ELE_ADC_GPIO_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	// 鎵撳紑 ADC IO绔�鍙ｆ椂閽�
//	RCC_APB2PeriphClockCmd ( ELE_ADC_L_GPIO_CLK|ELE_ADC_M_GPIO_CLK|ELE_ADC_R_GPIO_CLK, ENABLE );
//	
//	// 閰嶇疆 ADC IO 寮曡剼妯″紡
//	// 蹇呴』涓烘ā鎷熻緭鍏�
//	//宸﹁矾绔�鍙ｉ厤缃�锛孭A4
//	GPIO_InitStructure.GPIO_Pin = ELE_ADC_L_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(ELE_ADC_L_PORT, &GPIO_InitStructure);	

//	//涓�闂寸��鍙ｉ厤缃�锛孭A5
//	GPIO_InitStructure.GPIO_Pin = ELE_ADC_M_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(ELE_ADC_M_PORT, &GPIO_InitStructure);				


//	//鍙宠矾绔�鍙ｉ厤缃�锛孭C5
//	GPIO_InitStructure.GPIO_Pin = ELE_ADC_R_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(ELE_ADC_R_PORT, &GPIO_InitStructure);				
//	
//}

///**************************************************************************
//Function: ELE_ADC_Mode_Config
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛氬垵濮嬪寲鐢电�佸贰绾緼DC
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	

//static void ELE_ADC_Mode_Config(void)
//{
//	ADC_InitTypeDef ADC_InitStructure;	

//	// 鎵撳紑ADC鏃堕挓
//	ELE_ADC_APBxClock_FUN ( ELE_ADC_CLK, ENABLE );
//	
//	//澶嶄綅ADC1,灏嗗�栬�� ADC1 鐨勫叏閮ㄥ瘎瀛樺櫒閲嶈�句负缂虹渷鍊�
//	ADC_DeInit(ELE_ADC); 

//	// ADC 妯″紡閰嶇疆
//	// 鍙�浣跨敤涓�涓狝DC锛屽睘浜庣嫭绔嬫ā寮�
//	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
//	
//	// 绂佹�㈡壂鎻忔ā寮忥紝澶氶�氶亾鎵嶈�侊紝鍗曢�氶亾涓嶉渶瑕�
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 

//	// 鍗曟�¤浆鎹�
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

//	// 涓嶇敤澶栭儴瑙﹀彂杞�鎹�锛岃蒋浠跺紑鍚�鍗冲彲
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

//	// 杞�鎹㈢粨鏋滃彸瀵归綈
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	
//	// 杞�鎹㈤�氶亾1涓�
//	ADC_InitStructure.ADC_NbrOfChannel = 1;	
//		
//	// 鍒濆�嬪寲ADC
//	ADC_Init(ELE_ADC, &ADC_InitStructure);
//	
//	// 閰嶇疆ADC鏃堕挓涓篜CLK2鐨�6鍒嗛�戯紝鍗�12MHz
//	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
//	
//	// 閰嶇疆 ADC 閫氶亾杞�鎹㈤『搴忓拰閲囨牱鏃堕棿
//	ADC_RegularChannelConfig(ELE_ADC, ELE_ADC_L_CHANNEL, 1, 
//	                         ADC_SampleTime_239Cycles5);
//							 
//	ADC_RegularChannelConfig(ELE_ADC, ELE_ADC_M_CHANNEL, 1, 
//	                         ADC_SampleTime_239Cycles5);

//	ADC_RegularChannelConfig(ELE_ADC, ELE_ADC_R_CHANNEL, 1, 
//	                         ADC_SampleTime_239Cycles5);
//		
//	// 涓嶄腑鏂�
//	ADC_ITConfig(ELE_ADC, ADC_IT_EOC, DISABLE);
//	
//	// 寮�鍚疉DC 锛屽苟寮�濮嬭浆鎹�
//	ADC_Cmd(ELE_ADC, ENABLE);
//	
//	// 鍒濆�嬪寲ADC 鏍″噯瀵勫瓨鍣�  
//	ADC_ResetCalibration(ELE_ADC);
//	// 绛夊緟鏍″噯瀵勫瓨鍣ㄥ垵濮嬪寲瀹屾垚
//	while(ADC_GetResetCalibrationStatus(ELE_ADC));
//	
//	// ADC寮�濮嬫牎鍑�
//	ADC_StartCalibration(ELE_ADC);
//	// 绛夊緟鏍″噯瀹屾垚
//	while(ADC_GetCalibrationStatus(ELE_ADC));
//	
//}

///**************************************************************************
//Function: ELE_ADC_Init
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛氬垵濮嬪寲鐢电�佸贰绾緼DC
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	
////鐢电�佸贰绾垮垵濮嬪寲
//void ELE_ADC_Init(void)
//{
//	ELE_ADC_GPIO_Config();
//	ELE_ADC_Mode_Config();

//}


///**************************************************************************
//Function: CCD_ADC_Mode_Config
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛氬垵濮嬪寲CCD宸＄嚎ADC
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	
//static void CCD_ADC_Mode_Config(void)
//{
//	ADC_InitTypeDef ADC_InitStructure;	

//	// 鎵撳紑ADC鏃堕挓
//	CCD_ADC_APBxClock_FUN ( CCD_ADC_CLK, ENABLE );
//	
//	//澶嶄綅ADC1,灏嗗�栬�� ADC1 鐨勫叏閮ㄥ瘎瀛樺櫒閲嶈�句负缂虹渷鍊�
//	ADC_DeInit(CCD_ADC); 

//	// ADC 妯″紡閰嶇疆
//	// 鍙�浣跨敤涓�涓狝DC锛屽睘浜庣嫭绔嬫ā寮�
//	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
//	
//	// 绂佹�㈡壂鎻忔ā寮忥紝澶氶�氶亾鎵嶈�侊紝鍗曢�氶亾涓嶉渶瑕�
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 

//	// 鍗曟�¤浆鎹�
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

//	// 涓嶇敤澶栭儴瑙﹀彂杞�鎹�锛岃蒋浠跺紑鍚�鍗冲彲
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

//	// 杞�鎹㈢粨鏋滃彸瀵归綈
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	
//	// 杞�鎹㈤�氶亾1涓�
//	ADC_InitStructure.ADC_NbrOfChannel = 1;	
//		
//	// 鍒濆�嬪寲ADC
//	ADC_Init(CCD_ADC, &ADC_InitStructure);
//	
//	// 閰嶇疆ADC鏃堕挓涓篜CLK2鐨�6鍒嗛�戯紝鍗�12MHz
//	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 

//	// 閰嶇疆 ADC 閫氶亾杞�鎹㈤『搴忓拰閲囨牱鏃堕棿
//	ADC_RegularChannelConfig(CCD_ADC, CCD_ADC_CHANNEL, 1, 
//	                         ADC_SampleTime_239Cycles5);
//		
//	// 涓嶄腑鏂�
//	ADC_ITConfig(CCD_ADC, ADC_IT_EOC, DISABLE);
//	
//	// 寮�鍚疉DC 锛屽苟寮�濮嬭浆鎹�
//	ADC_Cmd(CCD_ADC, ENABLE);
//	
//	// 鍒濆�嬪寲ADC 鏍″噯瀵勫瓨鍣�  
//	ADC_ResetCalibration(CCD_ADC);
//	// 绛夊緟鏍″噯瀵勫瓨鍣ㄥ垵濮嬪寲瀹屾垚
//	while(ADC_GetResetCalibrationStatus(CCD_ADC));
//	
//	// ADC寮�濮嬫牎鍑�
//	ADC_StartCalibration(CCD_ADC);
//	// 绛夊緟鏍″噯瀹屾垚

//	while(ADC_GetCalibrationStatus(CCD_ADC));
//}


///**************************************************************************
//Function: CCD_GPIO_Config
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛氬垵濮嬪寲CCD宸＄嚎GPIO
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	
//static void CCD_GPIO_Config(void)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	// 鎵撳紑CCD绔�鍙ｆ椂閽�
//	RCC_APB2PeriphClockCmd ( TSL_CLK_GPIO_CLK|TSL_SI_GPIO_CLK|CCD_ADC_GPIO_CLK, ENABLE );
//	
//	// CLK,SI閰嶇疆涓鸿緭鍑�	
//	GPIO_InitStructure.GPIO_Pin = TSL_SI_PIN;				//PA4
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(TSL_SI_PORT, &GPIO_InitStructure);	


//	GPIO_InitStructure.GPIO_Pin = TSL_CLK_PIN;				//PA5
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_Init(TSL_CLK_PORT, &GPIO_InitStructure);				


//	//閰嶇疆ADC杈撳叆妯″紡
//	GPIO_InitStructure.GPIO_Pin = CCD_ADC_PIN;				//PC5
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(CCD_ADC_PORT, &GPIO_InitStructure);				
//	
//}


///**************************************************************************
//Function: CCD_Init
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛氬垵濮嬪寲CCD宸＄嚎
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	
//void CCD_Init(void)
//{
//	CCD_GPIO_Config();
//	CCD_ADC_Mode_Config();

//}

///**************************************************************************
//Function: ELE_Mode
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛氱數纾佸贰绾挎ā寮忚繍琛�
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	
//void ELE_Mode(void)
//{
//	if(Mode == ELE_Line_Patrol_Mode && Flag_Left!=1 &&Flag_Right!=1)
//	{
//		int Sum = 0;
//		Sensor_Left = Get_Adc(ELE_ADC_L_CHANNEL);
//		Sensor_Middle = Get_Adc(ELE_ADC_M_CHANNEL);
//		Sensor_Right = Get_Adc(ELE_ADC_R_CHANNEL);
//		Sum = Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;			
//		Sensor = Sum/(Sensor_Left+Sensor_Middle+Sensor_Right);
//		if(Detect_Barrier() == No_Barrier)		//妫�娴嬪埌鏃犻殰纰嶇墿
//				Target_x_speed=tracking_speed;       //缁欏皬杞︿竴涓�澶ф��300mm/s鐨勯�熷害
//		else									//鏈夐殰纰嶇墿
//		{
//			if(!Flag_Stop)
//				Buzzer_Alarm(100);				//褰撶數鏈轰娇鑳界殑鏃跺�欙紝鏈夐殰纰嶇墿鍒欒渹楦ｅ櫒鎶ヨ��
//			else 
//				Buzzer_Alarm(0);
//			Target_x_speed = 0;
//		}	
//	}
//	Target_gyro_z=ELE_turn(Encoder_Left,Encoder_Right,Gyro_Turn);
//	
//}

///**************************************************************************
//鍑芥暟鍔熻兘锛欵LE妯″紡杞�鍚戞帶鍒�
//鍏ュ彛鍙傛暟锛氬乏杞�缂栫爜鍣ㄣ�佸彸杞�缂栫爜鍣ㄣ�乑杞撮檧铻轰华
//杩斿洖  鍊硷細杞�鍚戞帶鍒禤WM
//浣�    鑰咃細骞宠　灏忚溅涔嬪��
//**************************************************************************/
//int ELE_turn(int encoder_left,int encoder_right,float gyro)//杞�鍚戞帶鍒�
//{
//	float Turn;     
//	float Bias,kp=60,Kd=0.2;	  
//	Bias=Sensor-100;
//	Turn=-Bias*kp/100-gyro*Kd/100;
//	if(Detect_Barrier() == Barrier_Detected)		//妫�娴嬪埌鏈夐殰纰嶇墿
//	{
//		if(!Flag_Stop)
//			Turn = 0;
//		}	
//	  return Turn;
//}

///**************************************************************************
//Function: Detect_Barrier
//Input   : none
//Output  : 1or0(Barrier_Detected or No_Barrier)
//鍑芥暟鍔熻兘锛氱數纾佸贰绾挎ā寮忛浄杈炬��娴嬮殰纰嶇墿
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細1鎴�0(妫�娴嬪埌闅滅�嶇墿鎴栨棤闅滅�嶇墿)
//**************************************************************************/	 	
////妫�娴嬮殰纰嶇墿
//u8 Detect_Barrier(void)
//{
//	u8 i;
//	u8 point_count = 0;
//	if(Lidar_Detect == Lidar_Detect_ON)
//	{
//		for(i=0;i<225;i++)	//妫�娴嬫槸鍚︽湁闅滅�嶇墿
//		{
//			if(Dataprocess[i].angle>340 || Dataprocess[i].angle <20) //鍦ㄥ皬杞︾殑姝ｅ墠鏂�40掳鑼冨洿
//			{
//				if(0<Dataprocess[i].distance&&Dataprocess[i].distance<Detect_distance)//700mm鍐呮槸鍚︽湁闅滅�嶇墿
//				  point_count++,Distance=Dataprocess[i].distance;
//			}
//		}
//		if(point_count > 3)//鏈夐殰纰嶇墿
//			return Barrier_Detected;
//		else
//			return No_Barrier;
//	}
//	else
//		return No_Barrier;
//}
///**************************************************************************
//鍑芥暟鍔熻兘锛氳蒋浠跺欢鏃�
//鍏ュ彛鍙傛暟锛氭棤
//杩斿洖  鍊硷細鏃�
//浣�    鑰咃細骞宠　灏忚溅涔嬪��
//**************************************************************************/
//void Dly_us(void)
//{
//   int ii;    
//   for(ii=0;ii<10;ii++);      
//}

///**************************************************************************
//Function: Read_TSL
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛氳�诲彇CCD妯″潡鐨勬暟鎹�
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	
////璇诲彇CCD妯″潡鐨勬暟鎹�
//void RD_TSL(void) 
//{
//	u8 i=0,tslp=0;
//	
//	TSL_CLK_HIGH;
//	TSL_SI_LOW;
//	Dly_us();
//	Dly_us();

//	TSL_CLK_LOW;
//	TSL_SI_LOW;
//	Dly_us();
//	Dly_us();

//	TSL_CLK_LOW;
//	TSL_SI_HIGH;
//	Dly_us();
//	Dly_us();

//	TSL_CLK_HIGH;
//	TSL_SI_HIGH;
//	Dly_us();
//	Dly_us();

//	TSL_CLK_HIGH;
//	TSL_SI_LOW;
//	Dly_us();
//	Dly_us();
//	
//	for(i=0;i<128;i++)
//	{ 
//		TSL_CLK_LOW; 
//		Dly_us();  //璋冭妭鏇濆厜鏃堕棿
//		Dly_us();
//		CCD_ADV[tslp]=(Get_Adc(CCD_ADC_CHANNEL))>>4;		
//		++tslp;
//		TSL_CLK_HIGH;
//		Dly_us();  
//	}
//}


///**************************************************************************
//鍑芥暟鍔熻兘锛氱嚎鎬�CCD鍙栦腑鍊�
//鍏ュ彛鍙傛暟锛氭棤
//杩斿洖  鍊硷細鏃�
//**************************************************************************/
//void  Find_CCD_Median(void)
//{ 
//	static u8 i,j,Left,Right,Last_CCD_Zhongzhi;
//	static u16 value1_max,value1_min;

//	value1_max=CCD_ADV[0];  //鍔ㄦ�侀槇鍊肩畻娉曪紝璇诲彇鏈�澶у拰鏈�灏忓��
//	for(i=15;i<123;i++)   //涓よ竟鍚勫幓鎺�15涓�鐐�
//	{
//		if(value1_max<=CCD_ADV[i])
//		value1_max=CCD_ADV[i];
//	}
//	value1_min=CCD_ADV[0];  //鏈�灏忓��
//	for(i=15;i<123;i++) 
//	{
//		if(value1_min>=CCD_ADV[i])
//		value1_min=CCD_ADV[i];
//	}
//	CCD_Yuzhi=(value1_max+value1_min)/2;	  //璁＄畻鍑烘湰娆′腑绾挎彁鍙栫殑闃堝��
//	for(i = 15;i<118; i++)   //瀵绘壘宸﹁竟璺冲彉娌�
//	{
//		if(CCD_ADV[i]>CCD_Yuzhi&&CCD_ADV[i+1]>CCD_Yuzhi&&CCD_ADV[i+2]>CCD_Yuzhi&&CCD_ADV[i+3]<CCD_Yuzhi&&CCD_ADV[i+4]<CCD_Yuzhi&&CCD_ADV[i+5]<CCD_Yuzhi)
//		{	
//			Left=i;
//			break;	
//		}
//	}
//	for(j = 118;j>15; j--)//瀵绘壘鍙宠竟璺冲彉娌�
//	{
//		if(CCD_ADV[j]<CCD_Yuzhi&&CCD_ADV[j+1]<CCD_Yuzhi&&CCD_ADV[j+2]<CCD_Yuzhi&&CCD_ADV[j+3]>CCD_Yuzhi&&CCD_ADV[j+4]>CCD_Yuzhi&&CCD_ADV[j+5]>CCD_Yuzhi)
//		{	
//			Right=j;
//			break;	
//		}
//	}
//	CCD_Zhongzhi=(Right+Left)/2;//璁＄畻涓�绾夸綅缃�
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //璁＄畻涓�绾跨殑鍋忓樊锛屽�傛灉澶�澶�
//		CCD_Zhongzhi=Last_CCD_Zhongzhi;    //鍒欏彇涓婁竴娆＄殑鍊�
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //淇濆瓨涓婁竴娆＄殑鍋忓樊
//}	
///**************************************************************************
//Function: CCD_Mode
//Input   : none
//Output  : none
//鍑芥暟鍔熻兘锛欳CD宸＄嚎妯″紡杩愯��
//鍏ュ彛鍙傛暟: 鏃�
//杩斿洖  鍊硷細鏃�
//**************************************************************************/	 	
//void CCD_Mode(void)
//{
//	static u8 Count_CCD = 0;								//璋冭妭CCD鎺у埗棰戠巼
//	if(Mode == CCD_Line_Patrol_Mode && Flag_Left !=1 &&Flag_Right !=1)
//	{
//		if(++Count_CCD == 4)									//璋冭妭鎺у埗棰戠巼锛�4*5 = 20ms鎺у埗涓�娆�
//		{
//			RD_TSL(); 
//      Find_CCD_Median();	//鎵句腑鍊�					
//      Count_CCD = 0;			
//		}
//		else if(Count_CCD>4)  Count_CCD = 0;
//	 if(Detect_Barrier() == No_Barrier)		  //妫�娴嬪埌鏃犻殰纰嶇墿
//	    Target_x_speed = tracking_speed;		//CCD宸＄嚎閫熷害
//		Target_gyro_z=CCD_turn(CCD_Zhongzhi,Gyro_Turn);
//	}
//}	


///**************************************************************************
//鍑芥暟鍔熻兘锛欳CD妯″紡杞�鍚戞帶鍒�  宸＄嚎
//鍏ュ彛鍙傛暟锛欳CD鎻愬彇鐨勪腑绾� Z杞撮檧铻轰华
//杩斿洖  鍊硷細杞�鍚戞帶鍒禤WM
//浣�    鑰咃細骞宠　灏忚溅涔嬪��
//**************************************************************************/
//int CCD_turn(u8 CCD,float gyro)//杞�鍚戞帶鍒�
//{
//	  float Turn;     
//    float Bias,kp=30,Kd=0.12;	  
//	  Bias=CCD-64;
//	  Turn=Bias*kp/100+gyro*Kd/100;
//	  return Turn;
//}

