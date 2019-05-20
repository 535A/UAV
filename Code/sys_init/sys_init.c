/***************************** (C) COPYRIGHT NULL С�ƺ� *************************

 * �ļ���       ��
 * ����         ����ʼ��
 *
 * ����         ��С�ƺ�
 * ˵��         �����Ǯ���� �ٷ��� ԭ�ӿ�

**********************************************************************************/
#include "com_def.h"


//1ms����һ�� ����

void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//����ж�
	{
            run();			    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
}


void TIM2_init(u16 arr,u16 psc)
{
    RCC->APB1ENR|=1<<0;	//TIM3ʱ��ʹ��    
    TIM2->ARR=arr;  	//�趨�������Զ���װֵ//�պ�1ms    
    TIM2->PSC=psc;  	//Ԥ��Ƶ��7200,�õ�10Khz�ļ���ʱ��		  
    TIM2->DIER|=1<<0;   //��������ж�	  
    TIM2->CR1|=0x01;    //ʹ�ܶ�ʱ��3
    MY_NVIC_Init(1,3,TIM2_IRQn,2);//��ռ1�������ȼ�3����2	
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
   
  //NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���  
   EXTI->PR=1<<7;  //����жϱ�־λ 
}


//uint16_t PrescalerValue = 0;

//arr 72000000/(arr+1)
//psc ��Ƶ��
void TIM3_PWM_init(u16 arr,u16 psc)
{
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC->APB2ENR|=1<<2;   	//ʹ��PORTAʱ��	 
  RCC->APB1ENR|=1<<1;   	//ʹ��TIM3ʱ��	
 	  	
  GPIOA->CRL&=0X00FFFFFF;	//PA6 A7��� 
  GPIOA->CRL|=0XBB000000;	//���ù������ 	  
   
  
  //PrescalerValue = (uint16_t) (SystemCoreClock /24000000) - 1;
  TIM_TimeBaseStructure.TIM_Period =arr;                                 //����
  TIM_TimeBaseStructure.TIM_Prescaler =psc;                             //��Ƶ
  TIM_TimeBaseStructure.TIM_ClockDivision =0;                            //ʱ�ӷָ�
  TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;             //���ϼ���ģʽ
                 //����ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                  //��ʼTIM3
  
  
  //************************** ͨ��1 *******************************
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM2;                //PWM2     
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;    //PWM����ʹ��
  TIM_OCInitStructure.TIM_Pulse =PWM1;                            //д�Ƚ�ֵ(ռ�ձ�
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;        //�ø�
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);                        //����Tָ���Ĳ�����ʼ������TIM3 OC1
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);               //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
  

  //****************************** ͨ��2 ******************************
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM2;               //PWM2 
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM2;
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  
  TIM_Cmd(TIM3,ENABLE);                                         //ʹ�ܼ���

}

void TIM4_PWM_init(u16 arr,u16 psc)
{
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC->APB2ENR|=1<<3;   	//ʹ��PORTAʱ��	 
  RCC->APB1ENR|=1<<2;   	//ʹ��TIM4ʱ��	
 	  	
  GPIOB->CRH&=0XFFFFFF00;	//PB8 B9��� 
  GPIOB->CRH|=0X000000BB;	//���ù������ 	  
   
  //PrescalerValue = (uint16_t) (SystemCoreClock /24000000) - 1;
  TIM_TimeBaseStructure.TIM_Period =arr;                                 //����
  TIM_TimeBaseStructure.TIM_Prescaler =psc;                             //��Ƶ
  TIM_TimeBaseStructure.TIM_ClockDivision =0;                            //ʱ�ӷָ�
  TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;             //���ϼ���ģʽ
                 //����ģʽ
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);                  //��ʼTIM1
  
  /*************************** ͨ��3 ********************************/
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM2;    //PWM2     
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;    //PWM����ʹ��
  TIM_OCInitStructure.TIM_Pulse =PWM3;                        //д�Ƚ�ֵ(ռ�ձ�
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;        //�ø�
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);                        //����Tָ���Ĳ�����ʼ������TIM3 OC2
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);               //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���

  /****************************** ͨ��4 ******************************/
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM2;    //PWM2 
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM4;
  TIM_OCInitStructure.TIM_OCPolarity =TIM_OCPolarity_High;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);


  TIM_Cmd(TIM4,ENABLE);                                         //ʹ�ܼ���
}


void SPI2_init()
{
  
  RCC->APB2ENR|=1<<3;  	//PORTBʱ��ʹ�� 	 
  RCC->APB1ENR|=1<<14;   	//SPI2ʱ��ʹ�� 
	//����ֻ���SPI�ڳ�ʼ��
  GPIOB->CRH&=0X000FFFFF; 
  GPIOB->CRH|=0XBBB00000;	//PB13/14/15���� 	    
  GPIOB->ODR|=0X7<<13;   	//PB13/14/15����
  SPI2->CR1|=0<<10;		//ȫ˫��ģʽ	
  SPI2->CR1|=1<<9; 		//���nss����
  SPI2->CR1|=1<<8;  

  SPI2->CR1|=1<<2; 		//SPI����
  SPI2->CR1|=0<<11;		//8bit���ݸ�ʽ	
  SPI2->CR1|=1<<1; 		//����ģʽ��SCKΪ1 CPOL=1
  SPI2->CR1|=1<<0; 		//���ݲ����ӵڶ���ʱ����ؿ�ʼ,CPHA=1  
	                 //��SPI2����APB1������.ʱ��Ƶ�����Ϊ36M.
  SPI2->CR1|=3<<3; 		//Fsck=Fpclk1/256   ��  011��Fpclk1/16
  SPI2->CR1|=0<<7; 		//MSBfirst   
  SPI2->CR1|=1<<6; 		//SPI�豸ʹ��
  SPI2_ReadWriteByte(0xff);//��������	
  
}

/*  
void SPI1_init()  //�ӻ�  ���
{
  RCC->APB2ENR|=1<<2;  	//PORTAʱ��ʹ�� 	 
  RCC->APB2ENR|=1<<12;   	//SPI1ʱ��ʹ�� 
          //����ֻ���SPI�ڳ�ʼ��
  GPIOA->CRL&=0X000FFFFF; 
  GPIOA->CRL|=0XBBB00000;	//PA5/6/7���� 	    
  GPIOA->ODR|=0X7<<5;   	//PA5/6/7����
  SPI1->CR1|=0<<10;		//ȫ˫��ģʽ	
  SPI1->CR1|=1<<9; 		//���nss����
  SPI1->CR1|=1<<8;  

  SPI1->CR1|=1<<2; 		//SPI����
  SPI1->CR1|=0<<11;		//8bit���ݸ�ʽ	
  SPI1->CR1|=1<<1; 		//����ģʽ��SCKΪ1 CPOL=1
  SPI1->CR1|=1<<0; 		//���ݲ����ӵڶ���ʱ����ؿ�ʼ,CPHA=1  
	//��SPI2����APB1������.ʱ��Ƶ�����Ϊ36M.
  SPI1->CR1|=3<<3; 		//Fsck=Fpclk1/256   ��  011��Fpclk1/16
  SPI1->CR1|=0<<7; 		//MSBfirst   
  SPI1->CR1|=1<<6; 		//SPI�豸ʹ��
  SPI1_ReadWriteByte(0xff);//��������	
  
}
*/

void n2401_init()
{
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOG, ENABLE);	 //ʹ��PB,G�˿�ʱ��
   
//���յ�2401          UAV  B 5 6 7
  RCC->APB2ENR|=1<<3;  	//PORTBʱ��ʹ�� 
  
  GPIOB->CRL&=0X000FFFFF;       //PB7��/��������,PB6������� PB5�������
  GPIOB->CRL|=0X83300000;		
  GPIOB->ODR|=7<<5;     	//PB5 6 7 ����
  
  GPIOB->CRH&=0XFFF0FFFF; 
  GPIOB->CRH|=0X00030000;	//PB12 ���� 	    
  GPIOB->ODR|=1<<12;    	//PB12����   ����ͷ����ô��
  
  /*
  GPIOG->CRL&=0X00FFFFFF;       //PG6��/��������,PG7�������
  GPIOG->CRL|=0X38000000;	
  GPIOG->CRH&=0XFFFFFFF0;
  GPIOG->CRH|=0X00000003;	//PG8 ������� 	    
  GPIOG->ODR|=7<<6;     	//PG6 7 8 ����	
  */
  
   SPI2_init();          //����SPI2
   
  SPI2->CR1&=~(1<<6); 	//SPI�豸ʧ��
  SPI2->CR1&=~(1<<1); 	//����ģʽ��SCKΪ0 CPOL=0
  SPI2->CR1&=~(1<<0); 	//���ݲ����ӵ�1��ʱ����ؿ�ʼ,CPHA=0  
  SPI2->CR1|=1<<6; 		//SPI�豸ʹ��
  
  NRF24L01_CE=0; 			//ʹ��24L01
  NRF24L01_CSN=1;			//SPIƬѡȡ��
  delay_us(20);
  
  /*
     //�����2401
  GPIOB->CRH&=0XFFF0FFFF;         //����GPIOB_CRH  ��PB12����Ϊ�������ģʽ ����ٶ�2Mhz   
  GPIOB->CRH|=0X00030000;
  GPIOB->ODR|=1<<12;    	//PB12���� ��ֹW25Qxx�ĸ��� 
  
  GPIOG->CRL&=0X00FFFFFF;       //PG6��/��������,PG7�������
  GPIOG->CRL|=0X38000000;	
  GPIOG->CRH&=0XFFFFFFF0;
  GPIOG->CRH|=0X00000003;	//PG8 ������� 	    
  GPIOG->ODR|=7<<6;     	//PG6 7 8 ����	 
  
  
  SPI2_init();                   //����SPI2
  
  //���NRF���ص��޸�SPI������
  SPI2->CR1&=~(1<<6); 	//SPI�豸ʧ��
  SPI2->CR1&=~(1<<1); 	//����ģʽ��SCKΪ0 CPOL=0
  SPI2->CR1&=~(1<<0); 	//���ݲ����ӵ�1��ʱ����ؿ�ʼ,CPHA=0  
  SPI2->CR1|=1<<6; 		//SPI�豸ʹ��
 
  NRF24L01_M_CE=0; 			//ʹ��24L01
  NRF24L01_M_CSN=1;			//SPIƬѡȡ��	
  */
  
}

void LED_init()    
{      
  RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��	   	 
  RCC->APB2ENR|=1<<6;    //ʹ��PORTEʱ��	
                   
  GPIOB->CRL&=0XFF0FFFFF; 
  GPIOB->CRL|=0X00300000;//PB.5 �������   	 
  GPIOB->ODR|=1<<5;      //PB.5 �����
                                                                                            
  GPIOE->CRL&=0XFF0FFFFF;
  GPIOE->CRL|=0X00300000;//PE.5�������
  GPIOE->ODR|=1<<5;      //PE.5����� 
  
}


void EXIT_init()
{
  
   Ex_NVIC_Config(GPIO_B,7,FTIR); 	//B.7  �ⲿ�ж� �½��ش���
   
   MY_NVIC_Init(1,2,EXTI9_5_IRQn,2);//��ռ1�������ȼ�3����2	
  
}


void sys_config()
{
  //Stm32_Clock_Init(9);
  SystemInit();
  delay_init();
  
  delay_ms(4000);  //delay 4s
  
 // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�    
  
  uart_init(230400);   //115200
  RCC->APB2ENR&=0xFFFFFBFF;    //�ص�uart1ʱ��
  
  TIM3_PWM_init(65535,0);         //����Ƶ  72000000/7200=10khz   ����APB2��Ƶ��  ���Բ���10k  5k   1k
  TIM4_PWM_init(65535,0);         //����Ƶ  72000000/7200=10khz   ����APB2��Ƶ��  ���Բ���10k
  
  n2401_init();
  NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
  NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
  while(NRF24L01_Check());
  //NRF24L01_RX_Mode();//��������
  MPU_Init();
  Adc_Init();
  
  //HMC5883_init();   //����ų�����
  
  RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��	 
  GPIOB->CRL&=0XFFFF0F0F; 
  GPIOB->CRL|=0X00003030;//PB.1 3 �������   	 
  GPIOB->ODR|=1<<1;      //PB.1 3 �����
  GPIOB->ODR|=1<<3;

  SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��
  
  delay_ms(1000);
  
  PID_init();
  
  //EXIT_init();
  
  TIM2_init(19,7199);  //6ms
  
}


