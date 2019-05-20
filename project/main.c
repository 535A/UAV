/***************************** (C) COPYRIGHT NULL С�ƺ� *************************

 * �ļ���       ��
 * ����         ��
 *
 * ʵ��ƽ̨     ��ս��V3������
 * ��汾       ���ٷ���   ԭ�ӿ�  
 *
 * ����         ��С�ƺ�
 * ʱ��         ��2018.04.18
 * ˵��         ��ĳЩ�ط����Ǯ����   ϵͳʱ��Ƶ��72Mhz
   
                  û����������  �ܷ�  �������µĵ����ת

**********************************************************************************/

#include "com_def.h"

void run();//��ִ�к���
void update_data();
float abs(float i);
void prepare_data();

u16 PWM1 = 65535;  //���ֵ  arr
u16 PWM2 = 65535;
u16 PWM3 = 65535;
u16 PWM4 = 65535;

u8 TimeCount=0,nrf_count=0; 

u16 VOL=0;
float pitch=0,roll=0,yaw=0; 		//ŷ����
float pitch0=0,roll0=0,yaw0=0;
float Hx,Hy,Alpha=0;   //Hx Hy �������mx my  Alpha�شŲ���

double mx=0,my=0,mz=0;
double mx_max=0,my_max=0,mz_max=0;
double mx_min=0,my_min=0,mz_min=0;
double mxo=0,myo=0,mzo=0;
float rol_OFFSET=0,pit_OFFSET=0,ACC_OFFSET=0;

u8 RX_buf[8]={0x01,0x00};
u8 TX_buf[8]={0};

short gyro_x,gyro_y,gyro_z;
short angle_x,angle_y,angle_z;
void usart2_send_char(u8 c);

u16 Control_ROLL=50,Control_PITCH=50,Control_YAW=50,R_THROTTLE,start=0,correct=0;   //ң��  ƫ�� ���� ����

  void main()
{
  
  sys_config();
  
  NRF24L01_RX_Mode();//��������
  /*                                                                         //6050�Դ���dmp   ��̫�ȶ�
  while(mpu_dmp_init()); test_rrr=MPU_Read_Byte(MPU_DEVICE_ID_REG);
  temp=MPU_Read_Byte(MPU_USER_CTRL_REG);
  MPU_Write_Byte(MPU_USER_CTRL_REG,temp&0xdf);	
  temp=MPU_Read_Byte(MPU_INTBP_CFG_REG);
  MPU_Write_Byte(MPU_INTBP_CFG_REG,MPU_INTBP_CFG_REG|0x02);	//���¿�������iic
  */
  while(1)
  {
   
     //ANO_DT_Data_Exchange();   //��������
     
  }	
  
}
u8 adccl,hclk,pclk1,pclk2,sysclk;

void run()              ////0.04s  250hz  �Ķ���ע����ʾ������ʱ��////
{
  

  TimeCount++;
  //nrf_count++;
  
  //COUNT=~COUNT;
  
  if(NRF24L01_RxPacket(RX_buf)==1);
  /*
  if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)    //6050dmp
  {
     pitch0=abs(pitch)*10;
     roll0=abs(roll)*10;
     yaw0=abs(yaw)*10;
  }
  */
 
  /*
   if(nrf_count<=5)
   {
     
     //NRF24L01_TX_Mode();
     //if(NRF24L01_TxPacket(TX_buf)==TX_OK);
     
     delay_us(20);
   }

   //if(nrf_count==7)
  //NRF24L01_TX_Mode();//��������
  
  if(nrf_count==20)
  {
    nrf_count=0;
    //NRF24L01_TX_Mode();//��������
    delay_us(20);
        
  } 
  */
  
  
  if(start==0)              //����
          R_THROTTLE=0;
  
  
  if(TimeCount==1)
  {  
    update_data();
      
  }  
  else if(TimeCount==2)    //��ȡmpu6050
  {
    MPU_Get_Accelerometer(&angle_x,&angle_y,&angle_z);          
    
    MPU_Get_Gyroscope(&gyro_x,&gyro_y,&gyro_z);

    //HMC_read();   //����ų�����
  }
  
  else if(TimeCount==3)         //��̬����
  {
    
    Yijielvbo(angle_x, gyro_x,angle_y,gyro_y,angle_z,gyro_z);      //�����˲�
  
    //Prepare_Data(angle_x,angle_y,angle_z);
    
    IMUupdate(gyro_x, gyro_y, gyro_z, anglex, angley, anglez);     //��Ԫ��
   
  }
  else if(TimeCount==4)
  {
    n2401_init();
     NRF24L01_RX_Mode();//��������
  }
   else if(TimeCount==5)
   {
      if(start==0)              //����
      {
          R_THROTTLE=0;
          MOTO_PWMRFLASH(0,0,0,0);
      }
      if(start==1)
        CONTROL( );  //�������
      
     TimeCount=0;

   }

}


//����ֵ
float abs_sz(float i)
{
  if(i>=0) return i;
  else return -i;
}


//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart2_send_char(u8 c)
{
	while((USART2->SR&0X40)==0);//�ȴ���һ�η������   
        
	USART2->DR=c;   	
}
/*
void prepare_data()
{
  //TX_buf[1]=Control_PITCH;
  TX_buf[2]=Control_ROLL;
  TX_buf[4]=Control_YAW;
  TX_buf[3]=4096-(R_THROTTLE/16);
  TX_buf[5]=VOL;
}
*/

void update_data()
{
  start=RX_buf[5];
  correct=RX_buf[6];

  if(RX_buf[3]<=0x01) RX_buf[3]=1;
   R_THROTTLE     =  (4096-RX_buf[3]*40)*16;
   if(start==0) R_THROTTLE=0;
   Control_PITCH  =  RX_buf[1];
   Control_ROLL   =  RX_buf[2];
   Control_YAW    =  RX_buf[4];
  if(Control_PITCH<60&&Control_PITCH>40) Control_PITCH=50;
  if(Control_ROLL<60&&Control_ROLL>40) Control_ROLL=50;
  if(Control_YAW<60&&Control_YAW>40) Control_YAW=50;
}
