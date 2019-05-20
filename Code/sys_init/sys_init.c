/***************************** (C) COPYRIGHT NULL 小破孩 *************************

 * 文件名       ：
 * 描述         ：初始化
 *
 * 作者         ：小破孩
 * 说明         ：借鉴钱大神 官方库 原子库

**********************************************************************************/
#include "com_def.h"


//1ms控制一次 周期

void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//溢出中断
	{
            run();			    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}


void TIM2_init(u16 arr,u16 psc)
{
    RCC->APB1ENR|=1<<0;	//TIM3时钟使能    
    TIM2->ARR=arr;  	//设定计数器自动重装值//刚好1ms    
    TIM2->PSC=psc;  	//预分频器7200,得到10Khz的计数时钟		  
    TIM2->DIER|=1<<0;   //允许更新中断	  
    TIM2->CR1|=0x01;    //使能定时器3
    MY_NVIC_Init(1,3,TIM2_IRQn,2);//抢占1，子优先级3，组2	
}

u8 sz_sr=0;
void EXTI9_5_IRQHandler(void)
{
  //COUNT=~COUNT;
  NRF24L01_RxPacket(RX_buf);

  start=RX_buf[5]; 
         
   if(start==1)
   {
     update_data();   //nrf2401
   }
   
  //NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器  
   EXTI->PR=1<<7;  //清除中断标志位 
}


//uint16_t PrescalerValue = 0;

//arr 72000000/(arr+1)
//psc 分频数
void TIM3_PWM_init(u16 arr,u16 psc)
{
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC->APB2ENR|=1<<2;   	//使能PORTA时钟	 
  RCC->APB1ENR|=1<<1;   	//使能TIM3时钟	
 	  	
  GPIOA->CRL&=0X00FFFFFF;	//PA6 A7输出 
  GPIOA->CRL|=0XBB000000;	//复用功能输出 	  
   
  
  //PrescalerValue = (uint16_t) (SystemCoreClock /24000000) - 1;
  TIM_TimeBaseStructure.TIM_Period =arr;                                 //周期
  TIM_TimeBaseStructure.TIM_Prescaler =psc;                             //分频
  TIM_TimeBaseStructure.TIM_ClockDivision =0;                            //时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;             //向上计数模式
                 //计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                  //初始TIM3
  
  
  //************************** 通道1 *******************************
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM2;                //PWM2     
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;    //PWM功能使能
  TIM_OCInitStructure.TIM_Pulse =PWM1;                            //写比较值(占空比
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;        //置高
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);                        //根据T指定的参数初始化外设TIM3 OC1
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);               //使能TIM3在CCR1上的预装载寄存器
  

  //****************************** 通道2 ******************************
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM2;               //PWM2 
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM2;
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  
  TIM_Cmd(TIM3,ENABLE);                                         //使能计数

}

void TIM4_PWM_init(u16 arr,u16 psc)
{
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC->APB2ENR|=1<<3;   	//使能PORTA时钟	 
  RCC->APB1ENR|=1<<2;   	//使能TIM4时钟	
 	  	
  GPIOB->CRH&=0XFFFFFF00;	//PB8 B9输出 
  GPIOB->CRH|=0X000000BB;	//复用功能输出 	  
   
  //PrescalerValue = (uint16_t) (SystemCoreClock /24000000) - 1;
  TIM_TimeBaseStructure.TIM_Period =arr;                                 //周期
  TIM_TimeBaseStructure.TIM_Prescaler =psc;                             //分频
  TIM_TimeBaseStructure.TIM_ClockDivision =0;                            //时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;             //向上计数模式
                 //计数模式
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);                  //初始TIM1
  
  /*************************** 通道3 ********************************/
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM2;    //PWM2     
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;    //PWM功能使能
  TIM_OCInitStructure.TIM_Pulse =PWM3;                        //写比较值(占空比
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;        //置高
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);                        //根据T指定的参数初始化外设TIM3 OC2
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);               //使能TIM3在CCR2上的预装载寄存器

  /****************************** 通道4 ******************************/
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM2;    //PWM2 
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM4;
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);


  TIM_Cmd(TIM4,ENABLE);                                         //使能计数
}


void SPI2_init()
{
  
  RCC->APB2ENR|=1<<3;  	//PORTB时钟使能 	 
  RCC->APB1ENR|=1<<14;   	//SPI2时钟使能 
	//这里只针对SPI口初始化
  GPIOB->CRH&=0X000FFFFF; 
  GPIOB->CRH|=0XBBB00000;	//PB13/14/15复用 	    
  GPIOB->ODR|=0X7<<13;   	//PB13/14/15上拉
  SPI2->CR1|=0<<10;		//全双工模式	
  SPI2->CR1|=1<<9; 		//软件nss管理
  SPI2->CR1|=1<<8;  

  SPI2->CR1|=1<<2; 		//SPI主机
  SPI2->CR1|=0<<11;		//8bit数据格式	
  SPI2->CR1|=1<<1; 		//空闲模式下SCK为1 CPOL=1
  SPI2->CR1|=1<<0; 		//数据采样从第二个时间边沿开始,CPHA=1  
	                 //对SPI2属于APB1的外设.时钟频率最大为36M.
  SPI2->CR1|=3<<3; 		//Fsck=Fpclk1/256   ？  011是Fpclk1/16
  SPI2->CR1|=0<<7; 		//MSBfirst   
  SPI2->CR1|=1<<6; 		//SPI设备使能
  SPI2_ReadWriteByte(0xff);//启动传输	
  
}

/*  
void SPI1_init()  //从机  外接
{
  RCC->APB2ENR|=1<<2;  	//PORTA时钟使能 	 
  RCC->APB2ENR|=1<<12;   	//SPI1时钟使能 
          //这里只针对SPI口初始化
  GPIOA->CRL&=0X000FFFFF; 
  GPIOA->CRL|=0XBBB00000;	//PA5/6/7复用 	    
  GPIOA->ODR|=0X7<<5;   	//PA5/6/7上拉
  SPI1->CR1|=0<<10;		//全双工模式	
  SPI1->CR1|=1<<9; 		//软件nss管理
  SPI1->CR1|=1<<8;  

  SPI1->CR1|=1<<2; 		//SPI主机
  SPI1->CR1|=0<<11;		//8bit数据格式	
  SPI1->CR1|=1<<1; 		//空闲模式下SCK为1 CPOL=1
  SPI1->CR1|=1<<0; 		//数据采样从第二个时间边沿开始,CPHA=1  
	//对SPI2属于APB1的外设.时钟频率最大为36M.
  SPI1->CR1|=3<<3; 		//Fsck=Fpclk1/256   ？  011是Fpclk1/16
  SPI1->CR1|=0<<7; 		//MSBfirst   
  SPI1->CR1|=1<<6; 		//SPI设备使能
  SPI1_ReadWriteByte(0xff);//启动传输	
  
}
*/

void n2401_init()
{
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOG, ENABLE);	 //使能PB,G端口时钟
   
//接收的2401          UAV  B 5 6 7
  RCC->APB2ENR|=1<<3;  	//PORTB时钟使能 
  
  GPIOB->CRL&=0X000FFFFF;       //PB7上/下拉输入,PB6推挽输出 PB5推挽输出
  GPIOB->CRL|=0X83300000;		
  GPIOB->ODR|=7<<5;     	//PB5 6 7 上拉
  
  GPIOB->CRH&=0XFFF0FFFF; 
  GPIOB->CRH|=0X00030000;	//PB12 推挽 	    
  GPIOB->ODR|=1<<12;    	//PB12上拉   摄像头会用么？
  
  /*
  GPIOG->CRL&=0X00FFFFFF;       //PG6上/下拉输入,PG7推挽输出
  GPIOG->CRL|=0X38000000;	
  GPIOG->CRH&=0XFFFFFFF0;
  GPIOG->CRH|=0X00000003;	//PG8 推挽输出 	    
  GPIOG->ODR|=7<<6;     	//PG6 7 8 上拉	
  */
  
   SPI2_init();          //配置SPI2
   
  SPI2->CR1&=~(1<<6); 	//SPI设备失能
  SPI2->CR1&=~(1<<1); 	//空闲模式下SCK为0 CPOL=0
  SPI2->CR1&=~(1<<0); 	//数据采样从第1个时间边沿开始,CPHA=0  
  SPI2->CR1|=1<<6; 		//SPI设备使能
  
  NRF24L01_CE=0; 			//使能24L01
  NRF24L01_CSN=1;			//SPI片选取消
  delay_us(20);
  
  /*
     //发射的2401
  GPIOB->CRH&=0XFFF0FFFF;         //就是GPIOB_CRH  将PB12配置为推挽输出模式 最大速度2Mhz   
  GPIOB->CRH|=0X00030000;
  GPIOB->ODR|=1<<12;    	//PB12上拉 防止W25Qxx的干扰 
  
  GPIOG->CRL&=0X00FFFFFF;       //PG6上/下拉输入,PG7推挽输出
  GPIOG->CRL|=0X38000000;	
  GPIOG->CRH&=0XFFFFFFF0;
  GPIOG->CRH|=0X00000003;	//PG8 推挽输出 	    
  GPIOG->ODR|=7<<6;     	//PG6 7 8 上拉	 
  
  
  SPI2_init();                   //配置SPI2
  
  //针对NRF的特点修改SPI的设置
  SPI2->CR1&=~(1<<6); 	//SPI设备失能
  SPI2->CR1&=~(1<<1); 	//空闲模式下SCK为0 CPOL=0
  SPI2->CR1&=~(1<<0); 	//数据采样从第1个时间边沿开始,CPHA=0  
  SPI2->CR1|=1<<6; 		//SPI设备使能
 
  NRF24L01_M_CE=0; 			//使能24L01
  NRF24L01_M_CSN=1;			//SPI片选取消	
  */
  
}

void LED_init()    
{      
  RCC->APB2ENR|=1<<3;    //使能PORTB时钟	   	 
  RCC->APB2ENR|=1<<6;    //使能PORTE时钟	
                   
  GPIOB->CRL&=0XFF0FFFFF; 
  GPIOB->CRL|=0X00300000;//PB.5 推挽输出   	 
  GPIOB->ODR|=1<<5;      //PB.5 输出高
                                                                                            
  GPIOE->CRL&=0XFF0FFFFF;
  GPIOE->CRL|=0X00300000;//PE.5推挽输出
  GPIOE->ODR|=1<<5;      //PE.5输出高 
  
}


void EXIT_init()
{
  
   Ex_NVIC_Config(GPIO_B,7,FTIR); 	//B.7  外部中断 下降沿触发
   
   MY_NVIC_Init(1,2,EXTI9_5_IRQn,2);//抢占1，子优先级3，组2	
  
}


void sys_config()
{
  //Stm32_Clock_Init(9);
  SystemInit();
  delay_init();
  
  delay_ms(4000);  //delay 4s
  
 // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级    
  
  uart_init(230400);   //115200
  RCC->APB2ENR&=0xFFFFFBFF;    //关掉uart1时钟
  
  TIM3_PWM_init(65535,0);         //不分频  72000000/7200=10khz   由于APB2分频了  所以不是10k  5k   1k
  TIM4_PWM_init(65535,0);         //不分频  72000000/7200=10khz   由于APB2分频了  所以不是10k
  
  n2401_init();
  NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
  NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
  while(NRF24L01_Check());
  //NRF24L01_RX_Mode();//主机调试
  MPU_Init();
  Adc_Init();
  
  //HMC5883_init();   //电机磁场干扰
  
  RCC->APB2ENR|=1<<3;    //使能PORTB时钟	 
  GPIOB->CRL&=0XFFFF0F0F; 
  GPIOB->CRL|=0X00003030;//PB.1 3 推挽输出   	 
  GPIOB->ODR|=1<<1;      //PB.1 3 输出高
  GPIOB->ODR|=1<<3;

  SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）
  
  delay_ms(1000);
  
  PID_init();
  
  //EXIT_init();
  
  TIM2_init(19,7199);  //6ms
  
}


