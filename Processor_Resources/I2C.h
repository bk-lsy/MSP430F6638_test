/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
* 文件名-FileName:			 I2C.h
* 附属文件-Dependencies:  	 None	
* 文件描述-File Description:	 ( 头文件-Header File )
	■"I2C" -驱动程序-头文件(处理器内部资源) 
	01)     02)     03)    04)    05)    06)	
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* 注意事项-Attention : 	
	▲01)     ▲02)     ▲03)    ▲04)    ▲05)    ▲06)     
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

#ifndef  __I2C_H 
#define  __I2C_H 


////////////////////////////////////////////////////////////////////////////
//==**"此模块专用"宏定义**Module-specific macro**==============//
////////////////////////////////////////////////////////////////////////////
#define   TXBUF         UCB1TXBUF
#define   RXBUF         UCB1RXBUF

#define   I2C_CTL0      UCB1CTL0
#define   I2C_CTL1      UCB1CTL1
#define   I2C_IE        UCB1IE
#define   I2C_IFG       UCB1IFG

#define   I2C_STT_IFG      UCSTTIFG
#define   I2C_STP_IFG      UCSTPIFG
#define   I2C_TX_IFG       UCTXIFG
#define   I2C_RX_IFG       UCRXIFG
#define   I2C_NACK_IFG     UCNACKIFG
#define   I2C_AL_IFG       UCALIFG

#define   START            UCTXSTT
#define   STOP             UCTXSTP
#define   NACK             UCTXNACK



////////////////////////////////////////////////////////////////////////////
//==**项目"专用"宏定义**Project-specific macro**================//
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
//==**全局变量定义**Global variables**========================//
////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////
//==**"函数"宏定义**Functions macro**=========================//
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//==**"此模块专用"函数声明**Exported Module-specific funcitions**===//
////////////////////////////////////////////////////////////////////////////

extern void  I2C_B_Initial(unsigned char slaveAddress);  //I2C_B 内部资源初始化

extern unsigned int Read_Word_IIC_B(unsigned char address); //通过IIC_B 读数据(一个字)

extern void Write_Word_IIC_B(u8 cmd,u8 data1,u8 data2); //通过IIC_B 向从机的某个寄存器写一个字


////////////////////////////////////////////////////////////////////////////
//==**"外部" API 函数声明**Exported  API funcitions**===============//
////////////////////////////////////////////////////////////////////////////





#endif   /* __I2C_H  */





