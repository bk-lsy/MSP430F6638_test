/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
* 文件名-FileName:			 System.c
* 附属文件-Dependencies:  	 System.h; System_HeadFile.h; 	
* 文件描述-File Description:	 ( 源程序-Source File) 
	■ "系统" 常用功能函数集
	01)    系统单片机内部资源的初使化程序
		A. 晶振设置初使化		      B. IO口初使化定义  
		C. AD口初使化				      D. 定时器初使化	  
		E. 中断初使化			  	      F. 上电后，部份变量初使化
		G. 看门狗初使化(周期)	      H. 各种复位初使化
		I.  其他功能模块初使化
	02)   系统变量初使化(包括状态机初使化)，分为"复位"和"上电"两种情况
		A. 系统主状态初始值赋值   B. 系统次状态初始值赋值
		C.				 D.
	03)  系统自检
		A. 自检蜂鸣器		 B. 自检显示部份:Led，数码管，液晶
		C.
	04)  通用"系统时钟"处理  
		分别在中断中产生定时为1ms,10ms, 20ms, 100ms, 200ms, 250ms, 500ms ,1s, 2s
	05) 系统状态机
	06) 

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* 注意事项-Attention : 	
	▲01) 处理器IO口使用宏定义，统一在Hardware_Profile.h 定义        
	▲02)     ▲03)    ▲04)    ▲05)    ▲06)     
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  
* 修改记录-Change History:   
	作者   时间        版本  内容描述
	Author 	 Date		   Rev      Comment
	-------------------------------------------------------------------------------
	BlueS	2012-12-12	  1.0	   
			xxxx-xx-xx	  x.x	   
			xxxx-xx-xx	  x.x				
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
* 公司-Company: 			CS-EMLAB  Co. , Ltd.
* 软件许可协议-Software License Agreement:
	Copyright (C) 2012-2020 	CS-EMLAB  Co. , Ltd.	All rights reserved.	

*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/



#include "System_HeadFile.h"   //引入: 系统全部头文件集(包含所有使用到的头文件)

#include "System.h"	 //"系统" 常用功能函数集-头文件



////////////////////////////////////////////////////////////////////////////
//==**全局变量定义**Global variables**========================//
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//==**系统时间标志位定义**time bit flag define**===============//
union FLAGBIT16 TimeFlag1Bits;  //时间函数标志位定义1
union FLAGBIT16 TimeFlag2Bits;  //时间函数标志位定义2


////////////////////////////////////////////////////////////////////////////
//==**系统时间计数器变量**============================//
unsigned long G_TimingCnt = 0; //人为定时计数器(以1ms为时基,减计数器)

unsigned long G_Timing_InWork_Cnt = 0; //(工作中的定时)人为定时计数器(以1ms为时基,减计数器)



//unsigned char	G_1msCnt = 0;    //系统定时  1ms计数器
unsigned char	G_10msCnt = 0;   //系统定时10ms计数器
unsigned char	G_100msCnt = 0;  //系统定时100ms计数器
unsigned int	G_250msCnt = 0;  //系统定时250ms计数器
unsigned int	G_500msCnt = 0;  //系统定时500ms计数器
unsigned int   G_1sCnt = 0;     //系统定时1s计数器

/////////////////////////////////////////////////////////////////////////////////
//在中断中，不同的第G_1ms_Case 个 1 ms执行中断中的部分任务
//good 这样做可以使在中断中程序执行的总时间大大减小
//放到这个Switch中的子程序为执行时间比较长的子程序，
//分任务进行，减小中断总时间
unsigned char  G_1ms_Case = 0;  


////////////////////////////////////////////////////////////////////////////
//==**系统状态机变量**=================================//
unsigned int G_MainState = 0;      //Main系统状态变量 
unsigned int G_Last_MainState = 0; //保存上一次系统状态变量 

//unsigned char G_UsersType = 0;     //用户类型:高级管理员、普通管理员

union FLAGBIT SystemFlagBits;	//系统标志位定义


////////////////////////////////////////////////////////////////////////////
//==**工作运行时,用到的状态变量**====================//
unsigned int G_WorkState = 0;	      //系统工作状态变量 
unsigned int G_Last_WorkState = 0;  //保存上一次工作状态变量 




////////////////////////////////////////////////////////////////////////////
//==**局部变量定义**Local variables**===========================//
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
//==**系统单片初使化程序**===============================//
////////////////////////////////////////////////////////////////////////////


/****************************************************************************
*函数名-Function:	void System_Initial(void)
*描述- Description:	系统初使化 :单片机内部资源+外部基本资源的初使化
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 把不使用的内部资源和外部资源用"//"屏蔽掉，以免误动作   
	▲02) 初使化顺序要求:
	⑴内部资源:	先关所有中断→晶振→AD口设置→IO口→Timer0
		→UART串口通信 →其他功能模块→复位寄存器→看门狗
	⑵内部资源: ①LCD 液晶模块 ②  ③ ④⑤⑥	
	⑶中断初使化:使能要使用的中断
	▲03)    ▲04)  
*****************************************************************************/
void System_Initial(void)
{	 
//==**先关闭所有中断，再进行初使化 //good**=======//

//==**判断是否为看门狗溢出复位**================//
P4DIR &= ~BIT0;  //care 上电后，一定要设为输入,不然，有可能会电源短路
	//P4.0为锂电池中断引脚
//P4REN |= BIT0;          
//P4OUT &= ~BIT0;              
	
		
////////////////////////////////////////////////////////////////////////////
//==**单片机内部资源初使化**=========================//
//==把不用的模块用屏蔽掉，不要删除==/////////////////////
	Osccon_Initial(); //晶振初始化函数:定义相关晶振参数

	mClose_LED_RED; //配置引脚为"输出",并"熄灭"LED_RED
	mClose_LED_GREEN; //配置引脚为"输出",并"熄灭"LED_GREEN 
	mClose_LED_YELLOW; //配置引脚为"输出",并"熄灭"LED_YELLOW


//Timer1_Initial(); //定时器1初始化设置, 用于系统每1ms自动中断一次

//Init_Timer0_B();  //定时器B 初始化设置(用于系统每1ms自动中断一次)

	Init_Timer0_A(); //定时器A0初始化设置(用于系统每1ms自动中断一次)

	//blues// Init_Timer2_A(); //定时器A2初始化设置(用于系统每1ms自动中断一次)

	ADC12_A_Initial(); //ADC12_A  内部资源初始化
	Close_ADC12_A(); // 关闭ADC12_A模块


//HR202_Humidity_Initial();  //初始化设置:  "HR202_Humidity 电阻型湿度传感器"


	//care 使用哪个串口，还要到中断总程序中，打开串口接收时间结束判断
//Uart1_Initial();   //Uart1 串口初始化

//Uart2_Initial();   //Uart2 串口初始化

	Init_RTC_B();  //初始化: RTC_B 实时时钟

////////////////////////////////////////////////////////////////////////////
//==**外部资源初使化**=================================//
//==把不用的模块用屏蔽掉，不要删除=================//


		//==**模块**初使化**==================//
		//==**模块**初使化**==================//
		//==**模块**初使化**==================//


		//==**LCM液晶模块**初使化**================//
		     //care 液晶的初使化要放初使化的后面,比较好	
	Init_LCD_TFT_ILI9325(); //"彩屏LCD_TFT_ILI9325" 初使化: 所有相关资源的初始化


	Close_LED_Segment();//关闭LED_Segment ("动态数码管LED_Segment" )
		//亮度变量-"动态数码管LED_Segment" 
	G_BrightnessKind = M_Brightness_1_LedSeg; //亮度为1(最暗)--"动态数码管LED_Segment" 


	BuzzTime_Key = M_Buzz200ms;    //按键有效时，蜂鸣器响的时间
////////////////////////////////////////////////////////////////////////////
//==**单片机中断功能初使化**=========================//


		//中断初使化放到初使化的最后//good  
	
}      



/****************************************************************************
*函数名-Function:	void Osccon_Initial(void)
*描述- Description:	晶振初始化函数:配置相关晶振参数
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void Osccon_Initial(void)
{

	//IO初始化，在放在"晶振"配置的前面
//P1DIR |= BIT0;		 // ACLK set out to pins
//P1SEL |= BIT0;	   		

//==care=下面之两句一定要有，不知道为什么?//////////////////
//P3DIR |= BIT4;							  // SMCLK set out to pins
//P3SEL |= BIT4;							  
//P3SEL &= (~BIT4);//SMCLK不从P3.4输出

	
/************************************/

	SetVCore(PMMCOREV_3);			 // Set Vcore to accomodate for max. allowed system speed

	UCSCTL3 |= SELREF_2;					  // Set DCO FLL reference = REFO

	__bis_SR_register(SCG0);				  // Disable the FLL control loop
	UCSCTL0 = 0x0000;						  // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_5;					  // Select DCO range 24MHz operation
	UCSCTL2 = FLLD_1 + 374; 				  // Set DCO Multiplier for 12MHz
									  // (N + 1) * FLLRef = Fdco
									  // (374 + 1) * 32768 = 12MHz
									  // Set FLL Div = fDCOCLK/2
	__bic_SR_register(SCG0);				  // Enable the FLL control loop
	

/******************
//==care=倍频到20M, 程序一定要放在这里,不然会影响到RTC//////
	 SetVCore(PMMCOREV_3);			 // Set Vcore to accomodate for max. allowed system speed
	 UCSCTL3 |= SELREF_2;				 // Set DCO FLL reference = REFO
	// UCSCTL4 |= SELA_2;				 // Set ACLK = REFO
	 Init_FLL_Settle(20000, 630);	// MCLK=DCO = 20MHz // Set system clock to max (20MHz)
*******/


									 
////////////////////////////////////////////////////////////////////////////								 
	 while(BAKCTL & LOCKIO) 				   // Unlock XT1 pins for operation
		BAKCTL &= ~(LOCKIO);   
	 

	 P7SEL |= BIT2+BIT3;					   // Port select XT2

	 UCSCTL6 &= ~XT2OFF;					   // Enable XT2 
//	 UCSCTL3 |= SELREF_2;					   // Set DCO FLL reference = REFO
											   // Since LFXT1 is not used,
											   // sourcing FLL with LFXT1 can cause
											   // XT1OFFG flag to set
	//  UCSCTL4 |= SELA_2; 					   // ACLK=REFO=32.768KHz,SMCLK=DCO,MCLK=DCO


	 UCSCTL6 &= ~(XT1OFF);					   // XT1 On
	 UCSCTL6 |= XCAP_3; 					   // Internal load cap 


UCSCTL4 &= ~SELS_5;    
UCSCTL4 |= SELS_4;	 //SMCLK选择DCOCLKDIV (倍频后的频率)


	//UCSCTL4 |= SELS_5 + SELM_5; 			  // SMCLK=MCLK=XT2
//UCSCTL4 |= SELS_5;	   // SMCLK=XT2 = 4MHz
//UCSCTL4 |= SELA_0; // ACLK = LFTX1 (by default)=32.768KHz



	// Loop until XT1,XT2 & DCO stabilizes - in this case loop until XT2 settles
	do
	{
	  UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
											  // Clear XT2,XT1,DCO fault flags
	  SFRIFG1 &= ~OFIFG;					  // Clear fault flags
	}while (SFRIFG1&OFIFG); 				  // Test oscillator fault flag

	UCSCTL6 &= ~XT2DRIVE0;					  // Decrease XT2 Drive according to  expected frequency
							
}






////////////////////////////////////////////////////////////////////////////
//==**系统变量初使化(包括状态机初使化)**============//
////////////////////////////////////////////////////////////////////////////


/****************************************************************************
*函数名-Function:	void System_Variable_Initial(void)
*描述- Description:	系统变量初使化(包括状态机初使化)，分为"复位"和"上电"两种情况
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void System_Variable_Initial(void)
{
////////////////////////////////////////////////////////////////////////////
//==**上电时，初使化的变量**=========================//
	//--**EEPROM中读出: 背光屏保开关**------------------//	  
//	G_EnBackLight = AT24CXX_ReadOneByte(M_BackLight_EepRomAdr); 

	//--**从EEPROM中读出: 屏保时间设定的参数值**---------//
//	AT24CXX_ReadSomeByte(M_BackLightTime_EepRomAdr, (unsigned char *)&G_MatrixKeyBuf[0], 4);		
//	G_BackLightNum = (1000*G_MatrixKeyBuf[0])+(100*G_MatrixKeyBuf[1])+(10*G_MatrixKeyBuf[2]) + G_MatrixKeyBuf[3];		
//	G_BackLightCnt = G_BackLightNum; //为屏保附初值



	//--**从EEPROM中读出: **-------------------------// 
	//--**从EEPROM中读出: **-------------------------// 	
	//--**从EEPROM中读出: **-------------------------// 



		//--**系统状态值初使化**-------------------// 
	Initial_MainState(MAIN_Self_Check); //上电后，装载//Main█系统自检状态
	  

////////////////////////////////////////////////////////////////////////////
//==**复位时，初使化的变量**=========================//


}


/****************************************************************************
*函数名-Function:	void System_LoopInitial(void)
*描述- Description:	系统程序中"循环"初使化,
						//good 可防止在程序循环中，初使化内容初改掉
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲04)  
*****************************************************************************
void System_LoopInitial(void)
{	  

  //==*循环初使化*-**===================================//


}      
**************************************************************/


////////////////////////////////////////////////////////////////////////////
//==**系统上电自检**===================================//
////////////////////////////////////////////////////////////////////////////

/****************************************************************************
*函数名-Function:	void System_SelfCheck(void)
*描述- Description:	系统上电自检:
						01)蜂鸣器 02)Led 灯03)LCD 液晶04)
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲  
*****************************************************************************/
void System_SelfCheck(void)
{


//==**检查"蜂鸣器"**蜂鸣器响1秒**=====================//
/*
	BuzzTimeNum = M_Buzz1s; //蜂鸣器叫1s，用于上电自检时	
	F_StartBuzz = 1;  //上电自检时，打开蜂鸣器1秒

	while(F_StartBuzz)
	{
		CLRWDT();
	}
*/


//==**检查显示部分有没有坏**--**=======================//

}



////////////////////////////////////////////////////////////////////////////
//==**通用"系统时钟"处理**=============================//
////////////////////////////////////////////////////////////////////////////

/****************************************************************************
*函数名-Function:	void System_Working_Led(void)
*描述- Description:	单片机正常工作指示灯: 250ms闪烁一次
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01)  系统"正常": 指示灯每250ms 闪烁一次
	            系统"异常": ①不是250ms 闪一下②闪烁忽快忽慢  ③ ④⑤
	▲02)    ▲03)    ▲04)  
*****************************************************************************/
void System_Working_Led(void)  
{
	if(F_250ms_Working_Led)
	{
		F_250ms_Working_Led = 0;
		
			//MCU正常工作，灯250ms闪烁
		mTurn_Working_Led;	//设置引脚为"输出",并"翻转"引脚输出状态		   
	}
}


/****************************************************************************
*函数名-Function:	void System_GetLoopClockTick(void)
*描述- Description:	每次"死等"延时10ms后，系统"循环"执行一次程序
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void System_GetLoopClockTick(void)
{
    while (F_10ms_TaskClock == 0)
    {
//mClr_WDTCNT;  // 清看门狗
    }
	
    F_10ms_TaskClock = 0;
}



/****************************************************************************
*函数名-Function:	void System_Clock(void)
*描述- Description:	通用"系统时钟"处理: 产生定时为1ms,10ms, 20ms, 100ms,等
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：
	▲01) 在"定时器中断"程序中，调用
	▲02)    ▲03)    ▲04)  
*****************************************************************************/
void System_Clock(void)  
{

////////////////////////////////////////////////////////////////////////////
//==**系统定时1ms,  大于1ms的定时，都以此1ms为时基**===//	

	System_Timing_Task();  //以1ms时基的人为定时，定时时间可设置,处理相关事务 

///////////////////////////////////////////////////////////////////////////
	System_1ms_Task();   //处理系统定时"1ms" 下的相关事务
	
	System_10ms_Task();   //处理系统定时"10ms" 下的相关事务
	
	System_100ms_Task();  //处理系统定时"100ms" 下的相关事务

	System_250ms_Task();  //处理系统定时"250ms" 下的相关事务 
	
	System_500ms_Task();  //处理系统定时"500ms" 下的相关事务

	System_1s_Task();     //处理系统定时"1s" 下的相关事务	
	
			//工作中用到的定时器。// 以1ms时基的人为定时，定时时间可设置,处理相关事务
	Timing_In_Work();
}



/****************************************************************************
*函数名-Function:	void System_Timing_Task(void) 
*描述- Description:	以1ms时基的人为定时，定时时间可设置,处理相关事务
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 在1ms的基础上累加计数 
	▲02) 定时过程中，标志F_Timing一直为1，定时到后，自动清零     
	▲03)    ▲04)  
*****************************************************************************/
void System_Timing_Task(void)  
{
	//为1时，启动人为定时器 ,定时到后，自动清零(以1ms为时基,减计数器)	
	if((F_Timing)&&(!F_SuspendTiming))
	{
			//若计数器初值本来就为0，则马上启动定后，又马上关闭，相当于没开
		if(G_TimingCnt == 0)  
		{
			F_Timing = 0;		 //定时到后自动清此标志位 
		}
		else G_TimingCnt--;
	}


////////////////////////////////////////////////////////////////////////////
//--**项目"专用"时间标志**Project-specific time flag**-----------------//



	
}


/****************************************************************************
*函数名-Function:	void Open_Timing_Task(unsigned int TimeValue)  
*描述- Description:	打开系统定时，定时给定的时间值(中断内)
*输入参数-Input:	TimeValue: 定时的时间值(时基为1ms)
*输出参数-output:	None
*注意事项-Note：	
	▲01)  在中断中，对定时时间计数器，进行递减   
	▲02)  定时到后，自动清"定时标志位"	  
	▲03)    ▲04)  
*****************************************************************************/
void Open_Timing_Task(unsigned int TimeValue)  
{
	G_TimingCnt = TimeValue;  //定时(时基为1ms)//给自减计数器附初值
	F_Timing = 1;	 //启动人为定时器 //定时到后自动清此标志位	 
}




/****************************************************************************
*函数名-Function:	void System_1ms_Task(void) 
*描述- Description:	处理系统定时"1ms" 下的相关事务 
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 系统定时 1 ms标志位
	▲02) F_1ms_LCD: 用于在LCD等待Busy信号时，每1ms累加1，
			但超过一定计数值时，则液晶复位
	▲03) F_1ms_Uart: 用于串口等待应答信号每1ms累加1	
	▲04)  每 1ms 判断串口UART是否通信结束(是否接收完)
*****************************************************************************/
void System_1ms_Task(void)  
{
	F_1ms = 1;		 //系统定时 1 ms标志位 
	
	F_1ms_LCD = 1;	 //用于在LCD等待Busy信号时，每1ms累加1，但超过一定计数值时，则液晶复位
	F_1ms_Uart = 1;  //用于串口等待应答信号每1ms累加1	

	Check_Buzz_Speaker();   //在中断里，检查是否结束: 蜂鸣器 -- "Speaker 扬声器" (包括:常用和报警用)


/////////////////////////////////////////////////////////////////////////////////
//==**每 1ms 判断串口UART是否通信结束(是否接收完)**=======//
//	Uart1_Check_Rx_End(); // UART 1中,判断数据串是否接收完毕
//	Uart2_Check_Rx_End(); // UART 2中,判断数据串是否接收完毕


/////////////////////////////////////////////////////////////////////////////////
//在中断中，不同的第G_1ms_Case 个 1 ms执行中断中的部分任务
//good 这样做可以使在中断中程序执行的总时间大大减小
//放到这个Switch中的子程序为执行时间比较长的子程序，
//分任务进行，减小中断总时间

	G_1ms_Case ++;	   // G_1ms_Case最大值为5故，故下面每个Case,都是每隔5ms执行一次
	
	if(G_1ms_Case >= 6)  
	{
		G_1ms_Case = 1;
	}
	
	switch(G_1ms_Case)
	{
		case 1:   // 中断中(每5ms执行一次)，扫描一次主要按键 (包括:设置按键和工作按键)			   
	
				//此子程序在32M晶振下，执行时间为11.8us
				//矩阵按键//放在中断中按键扫描消抖，提高抗干扰性
				//--care--care--TM1638,一个API在中断中调用，一个API在主循环中调用。如果在执行主循环的API时，进入中断里的TM1638的API,就会产生干扰
			if(F_En_Scan_MatrixKey) //为1时，使能"矩阵按键扫描",  
			{
				if((G_WorkState != WORK_Speaker_Test)&&(!F_StartSpeaker))
				{
					Scan_MatrixKey(); //"矩阵键盘"扫描，5ms扫描一次按键(中断里)		
				}
			}

			break;
	
		case 2: 
				//IO口按键//放在中断中按键扫描消抖，提高抗干扰性
			Scan_Key();  //"IO口键盘"扫描，5ms扫描一次按键(中断里)
			break;
	
		case 3:  // 中断中(每5ms执行一次)，系统输出，提高抗干扰性	
		/********************************************
			if((!F_Close595In_Int) && (!F_HaveDone_595out))  // 为0时，开74HC595在中断中的循环输出
			{  
				   //此子程序在32M晶振下，执行时间为57.5us
				OutputSomeByte_595();	 //多个字节，595串行数据输出转并行输出
			}
			
			if(F_HaveDone_595out) // 为1，表示595刚输出一次( 即防止一次中断输出2次595，以减小中断时间)	
			{
				F_HaveDone_595out = 0;
			}
		********************************************/
			break;
	
		case 4:   // 中断中(每5ms执行一次)，系统输入，提高抗干扰性		
		/********************************************
			if(!F_Close165In_Int)	// 为0时，开74HC165在中断中的循环采样输入
			{	
				  //此子程序在32M晶振下，执行时间为50us
				InPutSomeByte_165();   //多个字节，165并行输入转串行数据输出
			}
			********************************************/
			break;
	
	
		case 5:   // 中断中(每5ms执行一次)，扫描165输入信号(包括:行程开关)	
		/********************************************
			  //此子程序在32M晶振下，执行时间为7.2us
			Scan_Input_165();		//放在中断中按键扫描消抖，提高抗干扰性	
			********************************************/
			break;
		
		default:	  
			break;
	}

////////////////////////////////////////////////////////////////////////////
//--**项目"专用"**Project-specific **-------------------------------------//
        

}



/****************************************************************************
*函数名-Function:	void System_10ms_Task(void)
*描述- Description:	处理系统定时"10ms" 下的相关事务 
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 在1ms的基础上累加计数 
	▲02) 每10ms判断下位机是否收到有效数据
	▲03) 检查是否启动蜂鸣器(以10ms为时基)
	▲04)  
*****************************************************************************/
void System_10ms_Task(void)  
{
	G_10msCnt++;			  //系统定时10ms
	
	if(G_10msCnt >= 10)	
	{
		G_10msCnt = 0; 
		F_10ms_TaskClock = 1;			 //系统定时10ms标志位 
		F_10MS = 1;
		

	//--*每10ms判断下位机是否收到有效数据*---//
		//Check_RxOk_Uart2();	

	//--*每10ms中断中判断串口2是否发送数据*---//
		//Check_EnTx_Uart2(); //中断中判断串口2是否发送数据


	//--*检查是否启动蜂鸣器(以10ms为时基)*----------//
		Check_OutputBuzz();    //在中断里，每10ms检测蜂鸣器功能(包括:常用和报警用		

////////////////////////////////////////////////////////////////////////////
//--**项目"专用"**Project-specific **-------------------------------------//

	}


}


/****************************************************************************
*函数名-Function:	void System_100ms_Task(void) 
*描述- Description:	处理系统定时"100ms" 下的相关事务
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 在1ms的基础上累加计数  
	▲02) 系统定时每100ms读一次"实时时钟"  
	▲03)    ▲04)  
*****************************************************************************/
void System_100ms_Task(void)  
{
	G_100msCnt++; 		   //系统定时100ms
	if(G_100msCnt >= 100)  
	{
		G_100msCnt = 0; 
		F_100ms = 1; 		  //系统定时100ms标志位   
		F_100ms_ReadTime = 1; //系统定时每100ms读一次"实时时钟"
		//F_100ms_Current = 1;     //系统定时每100ms 采样一次"监测电流值"

////////////////////////////////////////////////////////////////////////////
//--**项目"专用"**Project-specific **-------------------------------------//
	


	}
}



/****************************************************************************
*函数名-Function:	void System_250ms_Task(void) 
*描述- Description:	处理系统定时"250ms" 下的相关事务
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 在1ms的基础上累加计数 
	▲02) 产生占空比为50%,周期为0.5s的矩行波
	▲03) 工作指示灯一直闪烁(250ms闪烁一次) 
	▲04)  
*****************************************************************************/
void System_250ms_Task(void)  
{
	G_250msCnt++;		   //系统定时250ms
	if(G_250msCnt >= 250)  
	{
		G_250msCnt = 0; 
		F_250ms = 1;		  //系统定时250ms标志位 
		   //产生占空比为50%,周期为0.5s的矩行波。系统定时250ms取反状态位	
		F_250ms_Turn = !F_250ms_Turn;	//参数闪烁时间为250ms闪烁一次,在数码管或液晶菜单设置时
			   
		F_250ms_Working_Led = 1;	  //用于单片机正常工作时，工作指示灯一直闪烁(250ms闪烁一次)  

////////////////////////////////////////////////////////////////////////////
//--**项目"专用"**Project-specific **-------------------------------------//
		F_250ms_Blink_Line_LCD_TFT = 1;
		F_250ms_Blink_Key_LCD_TFT = 1;
		F_250ms_Sample_Humidity = 1; //为1时，250ms读取一次湿度值"相应频率值
	}
}


/****************************************************************************
*函数名-Function:	void System_500ms_Task(void) 
*描述- Description:	处理系统定时"500ms" 下的相关事务
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 在1ms的基础上累加计数 
	▲02)    ▲03)    ▲04)  
*****************************************************************************/
void System_500ms_Task(void)  
{
	G_500msCnt++; 		 //定时500ms
	if(G_500msCnt >= 500)  
	{
		G_500msCnt = 0; 
		F_500ms = 1; 					 //系统定时500ms标志位


////////////////////////////////////////////////////////////////////////////
//--**项目"专用"**Project-specific **-------------------------------------//

		F_500ms_LcdSeg = 1;  //"段式LCD_Segment"显示"间隔"时间
		
		F_500ms_LedSeg = 1;  //"动态数码管LED_Segment" 显示"间隔"时间

		F_500ms_Temperature = 1; //系统定时每500ms读一次"实时TMP温度值"
		F_500ms_TemperatureInTest = 1; //系统定时每500ms读一次"实时TMP温度值"

		F_500ms_Reflash_INA21x = 1; //为1时，在常用区域，刷新LCD_TFT 显示("INA21x 电流分流监控器" )
		F_500ms_INA21x_InTest = 1;  //为1时，在测试区域，刷新LCD_TFT 显示("INA21x 电流分流监控器" )

		
		F_500ms_Reflash_Potentiometer = 1; //为1时，刷新LCD_TFT显示"电压值"(电位计 )

		F_500ms_Reflash_NTC_TMP = 1; //为1时，在常用区域，刷新LCD_TFT的显示("NTC 电阻温度传感器"  )
		F_500ms_NTC_HR202_InTest = 1;  //为1时，在测试区域，刷新LCD_TFT显示NTC（温度）、HR202（湿度）

		F_500ms_Reflash_Humidity = 1;  //为1时，在常用区域，刷新LCD_TFT 显示("HR202_Humidity 电阻型湿度传感器" )

		F_500ms_Reflash_DC_Motor = 1; //为1时，刷新LCD_TFT显示"转速" ("DC_Motor 直流电机")

		F_500ms_Reflash_Step_Motor = 1; //为1时，刷新LCD_TFT显示"转速" ("Step_Motor 步进电机")

		F_500ms_IrDA = 1; //为1时，//IrDA   //系统定时每500ms 

	}
}



/****************************************************************************
*函数名-Function:	void System_1s_Task(void) 
*描述- Description:	处理系统定时" 1 s " 下的相关事务
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 在1ms的基础上累加计数    
	▲02) 长时间无按键动作，关液晶显示   
	▲03) 其他"长时间"的定时，都以这里的1秒为时基
	▲04)  
*****************************************************************************/
void System_1s_Task(void)  
{
	G_1sCnt++;			//系统定时1秒
	if(G_1sCnt >= 1000)  
	{
		G_1sCnt = 0; 
		F_1s = 1;		   //系统定时1秒标志位 



////////////////////////////////////////////////////////////////////////////
//--**项目"专用"**Project-specific **-------------------------------------//

		F_1s_Frequency_DC_Motor = 1;	//为1时，直流电机转速(1s内的脉冲次数)

		F_1s_BQ27410 = 1;  //系统每1s 读取"实时BQ27410采样的 电量值"
		F_1s_BQ27410InTest = 1;  //系统每1s 读取"实时BQ27410采样的 电量值"
		

		/************************************************************************
		//---**G_BackLightNum秒长时间无按键动作，关液晶显示，省电//good**----------//
		if(G_EnBackLight && (G_BackLightNum != 0))	////存液晶背光屏保控制，为1时为开； 为0时为关
		{
			G_BackLightCnt --;
			if(G_BackLightCnt == 0)
			{
				G_BackLightCnt = G_BackLightNum;
				F_CloseLcm = 1;
			}
		}	
		************************************************************************/
	}

}







////////////////////////////////////////////////////////////////////////////
//==**常用通用的函数**=================================//
////////////////////////////////////////////////////////////////////////////





/****************************************************************************
*函数名-Function:	void Timing_In_Work(void) 
*描述- Description:	工作中用到的定时器。
          以1ms时基的人为定时，定时时间可设置,处理相关事务
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	
	▲01) 在1ms的基础上累加计数 
	▲02) 定时过程中，标志F_Timing_In_Work一直为1，定时到后，自动清零     
	▲03)    ▲04)  
*****************************************************************************/
void Timing_In_Work(void)  
{
	//为1时，启动人为定时器 ,定时到后，自动清零(以1ms为时基,减计数器)	
	if(F_Timing_In_Work)
	{
			//若计数器初值本来就为0，则马上启动定后，又马上关闭，相当于没开
		if(G_Timing_InWork_Cnt == 0)  
		{
			F_Timing_In_Work = 0;		 //定时到后自动清此标志位 
		}
		else G_Timing_InWork_Cnt --;
	}
}


/****************************************************************************
*函数名-Function:	void Open_Timing_In_Work(unsigned long TimeValue)  
*描述- Description:	打开工作中的定时器，定时给定的时间值(中断内)
*输入参数-Input:	TimeValue: 定时的时间值(时基为1ms)
*输出参数-output:	None
*注意事项-Note：	
	▲01)  在中断中，对定时时间计数器，进行递减   
	▲02)  定时到后，自动清"定时标志位"	  
	▲03)    ▲04)  
*****************************************************************************/
void Open_Timing_In_Work(unsigned long TimeValue)  
{
	G_Timing_InWork_Cnt = TimeValue;  //定时(时基为1ms)//给自减计数器附初值
	F_Timing_In_Work = 1;	 //启动人为定时器 //定时到后自动清此标志位	 
}




////////////////////////////////////////////////////////////////////////////
//==**系统状态机的函数**==============================//
////////////////////////////////////////////////////////////////////////////




