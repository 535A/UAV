/***************************** (C) COPYRIGHT NULL С�ƺ� *************************

 * �ļ���       ��
 * ����         ��
 *
 * ����         ��С�ƺ�
 * ʱ��         ��2016.08.17

**********************************************************************************/

//����ͷ�ļ�����   #include "com_def.h"

#include "delay.h"
#include "spi.h"
#include "usart.h"
#include "24l01.h"
#include "sys_init.h"
#include "mpu6050.h"
#include "sys.h"
#include "inv_mpu.h"
#include "mpuiic.h"
#include "HMC5883.h"
#include "calculation.h"
#include "transfer.h"
#include <math.h>
#include "adc.h"





//����
extern float pitch,roll,yaw;
extern float q0, q1, q2, q3;
extern float anglex,angley,anglez;  //�˲�֮��
extern double moto1,moto2,moto3,moto4;
extern double mx,my,mz;   
extern double mx_max,my_max,mz_max;
extern double mx_min,my_min,mz_min;
extern double mxo,myo,mzo;

extern u16 Control_ROLL,Control_PITCH,Control_YAW,R_THROTTLE,start,correct;   //ң��  ƫ�� ���� ����
extern float Pitch_shell_kp;//30 140
extern float Pitch_shell_kd;//
extern float Pitch_shell_ki;//
//////////////////////////////////////////////////////////
extern float Roll_shell_kp;//30
extern float Roll_shell_kd;//10                 
extern float Roll_shell_ki;//0.08
///////////////////////////////////////////////////////////
extern float Yaw_shell_kp;//10;//30
extern float Yaw_shell_kd;//10                 
extern float Yaw_shell_ki;//0.08;//0.08

extern float rol_OFFSET,pit_OFFSET,ACC_OFFSET;
extern u16 test_rrr;
extern short gyro_x,gyro_y,gyro_z;
extern short angle_x,angle_y,angle_z;
extern float pitch0,roll0,yaw0;
extern float Hx,Hy,Alpha;   //Hx Hy �������mx my  Alpha�شŲ���

extern uint16_t PWM1 ;
extern uint16_t PWM2 ;
extern uint16_t PWM3 ;
extern uint16_t PWM4 ;

extern u8 RX_buf[8];
extern u8 TX_buf[8];


extern PID PID_ROL,PID_PIT,PID_YAW;

//����
extern void run();
extern float abs_sz(float i);
extern void update_data();
extern void n2401_init();


//һЩ�궨��
#define LED0 PBout(5)	// DS0
#define LED1 PEout(5)	// DS1	
   
#define Angletorad    0.01745329252f  // �ȵ��Ƕ�
#define Radtoangle    57.295779513f   // ���ȵ��Ƕ�
#define Gyr_Gain 	    0.015267f       // ���ٶȱ�ɶ�                       
#define ACC_Gain 	    0.0011963f      // ���ٶȱ��G 
#define MAG_Gain	    0.1		//��ǿ��
   
#define Gyro_G 		0.0610351f