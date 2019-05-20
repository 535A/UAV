#include "usart.h"	
#include "com_def.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化（适合STM32F10x系列）		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2010/1/1
//版本：V1.7
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
//V1.6修改说明 20150109
//uart_init函数去掉了开启PE中断
//V1.7修改说明 20150322
//修改OS_CRITICAL_METHOD宏判断为：SYSTEM_SUPPORT_OS
////////////////////////////////////////////////////////////////////////////////// 	  
extern void usart2_send_char(u8 c);
/////////////////////////////////////////////////////////////////  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB  
#if defined ( __ICCARM__ )  //IAR下使用  
//重定义fputc函数   
int fputc(int ch, FILE *f)  
{        
    while((USART2->SR&0X40)==0);//循环发送,直到发送完毕     
    USART2->DR = (u8) ch;        
    return ch;  
}  
#else  //非IAR下使用  
#pragma import(__use_no_semihosting)               
//标准库需要的支持函数                   
struct __FILE   
{   
    int handle;   
};   
  
FILE __stdout;         
//定义_sys_exit()以避免使用半主机模式      
_sys_exit(int x)   
{   
    x = x;   
}   
//重定义fputc函数   
int fputc(int ch, FILE *f)  
{        
    while((USART1->SR&0X40)==0);//循环发送,直到发送完毕     
    USART1->DR = (u8) ch;        
    return ch;  
}  
#endif  



#if EN_USART2_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void USART2_IRQHandler(void)
{
	u8 res;	

	if(USART2->SR&(1<<5))	//接收到数据
	{	 
		res=USART2->DR; 
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}else //还没收到0X0D
			{	
				if(res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=res;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
                                        ANO_DT_Data_Receive_Prepare(res);
				}		 
			}
		}
        
	}

} 
#endif										 
//初始化IO 串口2
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率 
void uart_init(u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;
        u32 pclk2=72;
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
        mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  //使能串口时钟 
	GPIOA->CRL&=0XFFFF00FF;//IO状态设置
	GPIOA->CRL|=0X00008B00;//IO状态设置 
	RCC->APB1RSTR|=1<<17;   //复位串口1
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	//波特率设置
 	USART2->BRR=mantissa; // 波特率设置	 
	USART2->CR1|=0X200C;  //1位停止,无校验位.
//#if EN_USART1_RX		  //如果使能了接收
	//使能接收中断 
	USART2->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(3,4,USART2_IRQn,3);//组2，最低优先级 
//#endif
}




//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart2_niming_report(u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	
	for(i=0;i<len;i++)send_buf[i]=data[i];			//复制数据
	
	for(i=0;i<len;i++)usart2_send_char(send_buf[i]);	//发送数据到串口1 
}

/*
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart2_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart2_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
}  

*/
/*通过串口发送PID给匿名上位机



*/
/*
void usart_report_pid(short rol_p, short rol_i, short rol_d, short pit_p, short pit_i, short pit_d, short yaw_p, short yaw_i, short yaw_d )
{
  u8 tbuf[28];
  u8 i;
  for(i=0;i<28;i++) tbuf[i]=0; //clear 
  tbuf[0]=0XAD;
  tbuf[1] = (rol_p>>8)&0xFF;
  tbuf[2] = rol_p&0xFF;
  tbuf[3] = (rol_i>>8)&0xFF;
  tbuf[4] = rol_i&0xFF;
  tbuf[5] = (rol_d>>8)&0xFF;
  tbuf[6] = rol_d&0xFF;
  tbuf[7] = (pit_p>>8)&0xFF;
  tbuf[8] = pit_p&0xFF;
  tbuf[9] = (pit_i>>8)&0xFF;
  tbuf[10] = pit_i&0xFF;
  tbuf[11] = (pit_d>>8)&0xFF;
  tbuf[12] = pit_d&0xFF;
  tbuf[13] = (yaw_p>>8)&0xFF;
  tbuf[14] = yaw_p&0xFF;
  tbuf[15] = (yaw_i>>8)&0xFF;
  tbuf[16] = yaw_i&0xFF;
  tbuf[17] = (yaw_d>>8)&0xFF;
  tbuf[18] = yaw_d&0xFF;
  usart2_niming_report(0XAC,tbuf,28);
    
}
*/
