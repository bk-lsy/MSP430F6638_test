/*******************************************************************************
 *
 *  MassStorage.c - Uses the USB MSC stack
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/***************************************************************************//**
 * @file       MassStorage.c
 * @addtogroup MassStorage
 * @{
 ******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "msp430.h"
#include "HAL_PMM.h"
#include "HAL_UCS.h"
#include "mmc.h"
#include "diskio.h" // Low level disk interface module from the FatFs package
#include "device.h"
#include "types.h"  // Basic Type declarations
#include "descriptors.h"
#include "usb.h"    // USB-specific functions
#include "UsbMscScsi.h"
#include "USB_MSC_API\UsbMsc.h"
#include "USB_MSC_API\UsbMscStateMachine.h"
#include "UsbMscUser.h"

#include "MassStorageCommon.h"
#include "MassStorage.h"

#include "Hardware_Profile.h"  //处理器"硬件配置文件" -Hardware specific definitions for Processor


void msc_Init(void);
void msc_Loop(void);
uint8_t bDetectCard = 0x00;


/****************************************************************************
*函数名-Function:	void Disable_USB_SD_Card_Reader(void)
*描述- Description:		禁用:  "USB_SD_Card_Reader 读卡器(USB-SD卡)"
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
void Disable_USB_SD_Card_Reader(void)  //禁用:  "USB_SD_Card_Reader 读卡器(USB-SD卡)"
{
	SFRIFG1 = 0;

	SFRIE1 &= ~OFIE;

	mSPI_All_Disable;  //关闭所有SPI模块的使能


    if (USB_connectionInfo() & kUSB_vbusPresent)
    {
      USB_disconnect();
      USB_disable();
    }
	


}


/****************************************************************************
*函数名-Function:	void Initial_USB_SD_Card_Reader(void)
*描述- Description:		初始化设置:  "USB_SD_Card_Reader 读卡器(USB-SD卡)"
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    	▲02)    	▲03)    ▲04)  
*****************************************************************************/
void Initial_USB_SD_Card_Reader(void)  //初始化设置:  "USB_SD_Card_Reader 读卡器(USB-SD卡)"
{
//	Initial_Osccon_USB(); //初始化晶振--"USB_SD_Card_Reader 读卡器(USB-SD卡)"


mSPI_All_Disable;  //关闭所有SPI模块的使能
/***

P8SEL |= BIT4; 
P8SEL |= BIT5; 
P8SEL |= BIT6; 
****/

//P8REN &= ~BIT4;
//P8REN &= ~BIT5;
//P8REN &= ~BIT6;




	SFRIFG1 = 0;
	SFRIE1 |= OFIE;


	SFRIE1 &= ~OFIE;
	disk_initialize(0); 				  // Initialize Disk Drive #0

	SFRIE1 |= OFIE;

////////////////////////////////////////////////////////////////////////////
//==**配置"B模块"多路复用器TS3A5017: 选择IN2 、IN1**==============//
	mConfig_TS3A5017_B_IN2(0);	//IN2 =0
	mConfig_TS3A5017_B_IN1(1);	//IN1 =1	 //选择s2



	ClockUSB();

	USB_init(); 						  // Initialize the USB module

	// Enable all USB events
	USB_setEnabledEvents(kUSB_allUsbEvents);

	// Clal Initialization Function
	msc_Init();

	// If USB is already connected when the program starts up, then there won't be a
	// USB_handleVbusOnEvent().
	// So we need to check for it, and manually connect if the host is already present.
	if (USB_connectionInfo() & kUSB_vbusPresent)
	{
		if (USB_enable() == kUSB_succeed)
		{
			USB_reset();
			USB_connect();
		}
	}

}



/****************************************************************************
*函数名-Function:	void Initial_Osccon_USB(void)
*描述- Description:		初始化晶振--"USB_SD_Card_Reader 读卡器(USB-SD卡)"
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void Initial_Osccon_USB(void) //初始化晶振--"USB_SD_Card_Reader 读卡器(USB-SD卡)"
{

	//IO初始化，在放在"晶振"配置的前面
//P1DIR |= BIT0;		 // ACLK set out to pins
//P1SEL |= BIT0;			

//==care=下面之两句一定要有，不知道为什么?//////////////////
//P3DIR |= BIT4;							  // SMCLK set out to pins
//P3SEL |= BIT4;							  




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
	


	/*******************


	//==care=倍频到20M, 程序一定要放在这里,不然会影响到RTC//////
	SetVCore(PMMCOREV_3);			 // Set Vcore to accomodate for max. allowed system speed
	UCSCTL3 |= SELREF_2;				 // Set DCO FLL reference = REFO
	// UCSCTL4 |= SELA_2;				 // Set ACLK = REFO
	Init_FLL_Settle(20000, 630);	// MCLK=DCO = 20MHz // Set system clock to max (20MHz)

************/

									 
////////////////////////////////////////////////////////////////////////////								 
	 while(BAKCTL & LOCKIO) 				   // Unlock XT1 pins for operation
		BAKCTL &= ~(LOCKIO);   
	 

	 P7SEL |= BIT2+BIT3;					   // Port select XT2

	 UCSCTL6 &= ~XT2OFF;					   // Enable XT2 
//	 UCSCTL3 |= SELREF_2;					   // Set DCO FLL reference = REFO
											   // Since LFXT1 is not used,
											   // sourcing FLL with LFXT1 can cause
											   // XT1OFFG flag to set
	//	UCSCTL4 |= SELA_2;					   // ACLK=REFO=32.768KHz,SMCLK=DCO,MCLK=DCO


	 UCSCTL6 &= ~(XT1OFF);					   // XT1 On
	 UCSCTL6 |= XCAP_3; 					   // Internal load cap 


	// Loop until XT1,XT2 & DCO stabilizes - in this case loop until XT2 settles
	do
	{
	  UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
											  // Clear XT2,XT1,DCO fault flags
	  SFRIFG1 &= ~OFIFG;					  // Clear fault flags
	}while (SFRIFG1&OFIFG); 				  // Test oscillator fault flag

	UCSCTL6 &= ~XT2DRIVE0;					  // Decrease XT2 Drive according to  expected frequency
							
	//UCSCTL4 |= SELS_5 + SELM_5;			  // SMCLK=MCLK=XT2
	UCSCTL4 |= SELS_5;	   // SMCLK=XT2 = 4MHz
	
	UCSCTL4 |= SELA_0; // ACLK = LFTX1 (by default)=32.768KHz

}







/****************************************************************************
*函数名-Function:	void Deal_USB_SD_Card_Reader(void)
*描述- Description:		处理"USB_SD_Card_Reader 读卡器(USB-SD卡)"
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲04)  
*****************************************************************************/
void Deal_USB_SD_Card_Reader(void)  //处理"USB_SD_Card_Reader 读卡器(USB-SD卡)"
{
    switch (USB_connectionState())
    {
        case ST_USB_DISCONNECTED:
//blues//         __bis_SR_register(LPM3_bits);    // Enter LPM3 until VBUS-on event
            _NOP();
            break;

        case ST_USB_CONNECTED_NO_ENUM:
            break;

        case ST_ENUM_ACTIVE:

            msc_Loop();
            break;

        case ST_ENUM_SUSPENDED:
//blues//        __bis_SR_register(LPM3_bits + GIE);    // Enter LPM3, until a resume or VBUS-off
                                                   // event
            break;

        case ST_ENUM_IN_PROGRESS:
            break;

        case ST_ERROR:
            break;
        default:;
    }

}




/****************************************************************************
*函数名-Function:	void Deal_USB_SD_Card_Reader(void)
*描述- Description:		处理"USB_SD_Card_Reader 读卡器(USB-SD卡)"
*输入参数-Input:	None
*输出参数-output:	None
*注意事项-Note：	▲01)    ▲02)    ▲03)    ▲04)  
**************************************************************
void Deal_USB_SD_Card_Reader(void)  //处理"USB_SD_Card_Reader 读卡器(USB-SD卡)"
{
    while (1)
    {
        switch (USB_connectionState())
        {
            case ST_USB_DISCONNECTED:
//blues//         __bis_SR_register(LPM3_bits);    // Enter LPM3 until VBUS-on event
                _NOP();
                break;

            case ST_USB_CONNECTED_NO_ENUM:
                break;

            case ST_ENUM_ACTIVE:

                msc_Loop();
                break;

            case ST_ENUM_SUSPENDED:
//blues//        __bis_SR_register(LPM3_bits + GIE);    // Enter LPM3, until a resume or VBUS-off
                                                       // event
                break;

            case ST_ENUM_IN_PROGRESS:
                break;

            case ST_ERROR:
                break;
            default:;
        }
    }
}
***************/





/***************************************************************************//**
 * @brief  Handles UNMI interrupts
 * @param  none
 * @return none
 ******************************************************************************/
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR(void)
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
            UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG); // Clear OSC flaut Flags fault flags
            SFRIFG1 &= ~OFIFG;                          // Clear OFIFG fault flag
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            // If bus error occurred - the cleaning of flag and re-initializing of USB is required.
            SYSBERRIV = 0;                              // clear bus error flag
            USB_disable();                              // Disable
    }
}
/***************************************************************************//**
 * @}
 ******************************************************************************/
