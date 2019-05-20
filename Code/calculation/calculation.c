#include "com_def.h"

float K1 =0.11; // 对加速度计取值的权重
float dt=0.005;//注意：dt的取值为滤波器采样时间
float anglex,angley,anglez;

void Yijielvbo(float angle_x, float gyro_x,float angle_y, float gyro_y,float angle_z, float gyro_z)//采集后计算的角度和角加速度
{
	//static float x,y,z;

     anglex = K1 * angle_x+ (1-K1) * (anglex + gyro_x * dt);
	 angley = K1 * angle_y+ (1-K1) * (angley + gyro_y * dt);
	 anglez = K1 * angle_z+ (1-K1) * (anglez + gyro_z * dt);
     
}

#define FILTER_NUM 	5
void Prepare_Data(short angle_x,short angle_y,short angle_z)
{
	static uint8_t 	filter_cnt=0;
	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = angle_x;
	ACC_Y_BUF[filter_cnt] = angle_y;
	ACC_Z_BUF[filter_cnt] = angle_z;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	anglex = temp1 / FILTER_NUM;
	angley = temp2 / FILTER_NUM;
	anglez = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
}


//卡尔曼滤波参数与函数
float Dt=0.001;//注意：Dt的取值为kalman滤波器采样时间
float angle, angle_dot;//角度和角速度
float P[2][2] = {{ 1, 0 },
              { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度
float R_angle=0.5 ,C_0 = 1; 
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;


//卡尔曼滤波
void Kalman_Filter(float angle_m, float gyro_m)//angleAx 和 gyroGy
{
    angle+=(gyro_m-q_bias) * Dt;
    angle_err = angle_m - angle;
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]= - P[1][1];
    Pdot[2]= - P[1][1];
    Pdot[3]=Q_gyro;
    P[0][0] += Pdot[0] * Dt;
    P[0][1] += Pdot[1] * Dt;
    P[1][0] += Pdot[2] * Dt;
    P[1][1] += Pdot[3] * Dt;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angle += K_0 * angle_err; //最优角度
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m-q_bias;//最优角速度
}






void MOTO_PWMRFLASH(double moto1,double moto2,double moto3,double moto4);


#define Kp 1.7f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0000f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0121f                   // half the sample period采样周期的一半
#define Gyro_Gr		0.0010653f	

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
float Yaw,Pitch,Roll,yaw_old[5];  //偏航角，俯仰角，翻滚角
u8 correct_count=0;
float pitch_offset=0,roll_offset=0,yaw_check;
float PIT_OFFSET=0,ROL_OFFSET=0;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
//  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
  
  
	
	if(ax*ay*az==0)
 		return;
		
  gx *= Gyro_Gr;
  gy *= Gyro_Gr;
  gz *= Gyro_Gr;
        
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
  vx = 2*(q1q3 - q0q2);												//四元素中xyz的表示
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //向量外积在相减得到差分就是误差
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //对误差进行积分
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//将误差PI后补偿到陀螺仪，即补偿零点漂移
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

  // integrate quaternion rate and normalise						   //四元素的微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //Q_ANGLE.Yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
  //Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  //Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
  
  
   pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
  
  pitch=-pitch;
  roll=-roll;
 //if((pitch*1000)>-1||(pitch*1000)<1) pitch=0;
  //  if((roll*1000)>-1||(roll*1000)<1)    roll=0;
    if(correct==0) correct_count=0;
  if(correct==1&&correct_count<=50)
  {
    pitch_offset+=pitch*100;
    roll_offset+=roll*100;
    correct_count++;
    if(correct_count==51)
    {
      pitch_offset=pitch_offset/50*0.01;
      roll_offset=roll_offset/50*0.01;
      correct_count=52;
      PIT_OFFSET=pitch_offset;
      ROL_OFFSET=roll_offset;
      pitch_offset=0;
      roll_offset=0;
    }
  }
  
  pitch-=PIT_OFFSET;
  roll-=ROL_OFFSET;
  
   pitch0=pitch*3.1415926/180;
   roll0=roll*3.1415926/180;
    
     
 /*
  mx*=MAG_Gain;
  my*=MAG_Gain;
  mz*=MAG_Gain;
  
  //if(pitch0>=0)
  //  pitch0=-pitch0;
  //if(roll0<=0)
   // roll0=-roll0;
    
  Hy=my*cos(roll0)+mz*sin(roll0);
  Hx=mx*cos(pitch0)+my*sin(pitch0)*sin(roll0)+mz*sin(pitch0)*cos(roll0);
  
  if(Hx>0&&Hy<0)  
    yaw=180+atan2(Hy,Hx)*(180/3.14159265);
  if(Hx<0&&Hy<0)   
    yaw=180+atan2(Hy,Hx)*(180/3.14159265);
  if(Hx<0&&Hy>0)   
    yaw=-180+atan2(Hy,Hx)*(180/3.14159265);
  if(Hx>0&&Hy>0)   
    yaw=-180+atan2(Hy,Hx)*(180/3.14159265);
  if(Hx>0&&Hy==0)  
    yaw=0;
  if(Hx==0&&Hy<0)  
    yaw=90;
  if(Hx<0&&Hy==0)   
    yaw=180;
  if(Hx==0&&Hy>0) 
    yaw=270;
  //yaw=(yaw+Alpha)%360.0;
   yaw-=23;
   
   yaw_old[4]=yaw_old[3];
   yaw_old[3]=yaw_old[2];
   yaw_old[2]=yaw_old[1];
   yaw_old[1]=yaw_old[0];
   yaw_old[0]=yaw;
   yaw=(yaw_old[0]+yaw_old[1]+yaw_old[2]+yaw_old[3]+yaw_old[4])/5;
  */
  yaw_check-=0.00535;
  //yaw=atan2(my*cos(pitch0)+mz*sin(pitch0),mx*cos(roll0)+my*sin(roll0)*sin(pitch0)-mz*sin(roll0)*cos(pitch0))*(180/3.1415926)+180;
  
  //if(pitch0>=0&&roll>=0)
  //yaw=atan2(my*cos(roll0)+mz*sin(roll0),mx*cos(pitch0)+my*sin(pitch0)*sin(roll0)-mz*sin(pitch0)*cos(roll0))*(180/3.14159265);
  //yaw=atan2(my*cos(roll)+mz*sin(roll),mx*cos(pitch)+my*sin(pitch)*sin(roll)-mz*sin(pitch)*cos(roll))*(180/3.14159265)+180;
  
  //yaw=atan((my*cos(roll)+mx*sin(pitch)*sin(roll)-mz*sin(pitch)*cos(roll))/(my*cos(roll)+mz*sin(roll)))*(180/3.14159265);
  //yaw=atan2((double)my,(double)mx)*(180/3.1415926)+180;
  
   yaw=-(atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3);	  //yaw  
  
   //yaw += gyro_z*Gyro_G*0.002f;
  yaw+=yaw_check;


  
}

/*
void updatePID(float rol_P,float rol_I,float rol_D,float pit_P,float pit_I,float pit_D,float yaw_P,float yaw_I,float yaw_D)
{
 //由上位机给出
  
        PID_ROL.P = rol_P;
	PID_ROL.I = rol_I;
	PID_ROL.D = rol_D;
	
	PID_PIT.P = pit_P;
	PID_PIT.I = pit_I;
	PID_PIT.D = pit_D;
	
	PID_YAW.P = yaw_P;
	PID_YAW.I = yaw_I;
	PID_YAW.D = yaw_D;

}
*/

void PID_init()
{
 //由上位机给出
  
        PID_ROL.P = 2.8;
	PID_ROL.I = 0;
	PID_ROL.D = 2.0;
	
	PID_PIT.P = 2.8;
	PID_PIT.I = 0;
	PID_PIT.D = 2.0;
	
	PID_YAW.P = 0.5;
	PID_YAW.I = 0;
	PID_YAW.D = 0.1;

}

/*
extern T_RC_Data                         Rc_D;                //遥控通道数据;

extern u8 txbuf[4];         //发送缓冲
extern u8 rxbuf[4];         //接收缓冲
extern u16 test1[3]; //接收到NRf24L01数据
extern S_INT16_XYZ ACC_F,GYRO_F;

PID PID_ROL,PID_PIT,PID_YAW;

extern S_INT16_XYZ        MPU6050_ACC_LAST,MPU6050_GYRO_LAST;      
*/


int Motor_Ele=0;                                           //俯仰期望
int Motor_Ail=0;                                           //横滚期望

//u8 ARMED = 0;

//float rol_i=0,pit_i=0,yaw_p=0;
float thr=0;

//S_FLOAT_XYZ EXP_ANGLE ,DIF_ANGLE;
//PID1 PID_Motor;
///////////////////////////////////////////////////////////
float Pitch_i,Roll_i,Yaw_i,rol_i,pit_i,yaw_i;                                   //积分项
float Pitch_old,Roll_old,Yaw_old;                 //角度保存
float Pitch_d,Roll_d,Yaw_d;          //微分项
float RC_Pitch=0,RC_Roll=0,RC_Yaw=0;                       //姿态角
float Pitch_shell_out,Roll_shell_out,Yaw_shell_out;//外环总输出
        //外环PID参数
float Pitch_shell_kp=32.4;//1860;//30 140
float Pitch_shell_kd=0;//
float Pitch_shell_ki=0.10;//6.5;//
//////////////////////////////////////////////////////////
float Roll_shell_kp=32.4;//1860;//30
float Roll_shell_kd=0;//10                 
float Roll_shell_ki=0.10;//6.5;//0.08
///////////////////////////////////////////////////////////
float Yaw_shell_kp=22.0;//23;//24.4;//1400;//10;//30
float Yaw_shell_kd=0;//10                 
float Yaw_shell_ki=0.06;//4;//0.08;//0.08
float Gyro_radian_old_x=0,Gyro_radian_old_y=0,Gyro_radian_old_z=0;//陀螺仪保存
float Pitch_core_kp_out,Pitch_core_ki_out,Pitch_core_kd_out,Roll_core_kp_out,Roll_core_ki_out,Roll_core_kd_out,Yaw_core_kp_out,Yaw_core_ki_out,Yaw_core_kd_out;//内环单项输出
float Pitch_core_out,Roll_core_out,Yaw_core_out;//内环总输出       
///////////////内环PID
PID PID_ROL,PID_PIT,PID_YAW;

double moto1=0,moto2=0,moto3=0,moto4=0;

float tempjd=0;

void CONTROL()
{
       
  //if((roll*10)>-1&&(roll*10)<1) roll0=0;
  //if((pitch*10)>-1&&(pitch*10)<1) pitch0=0;
  
  RC_Pitch=(Control_PITCH-50)*1;
       
        ////////////////////////外环角度环(PID)///////////////////////////////
  Pitch_i+=(RC_Pitch-pitch);
//-------------Pitch积分限幅----------------//
  if(Pitch_i>150) Pitch_i=150;
  else if(Pitch_i<-150) Pitch_i=-150;
//-------------Pitch微分--------------------//
  //   无  Pitch_d=pitch0-Pitch_old;
//-------------Pitch  PID-------------------//
  Pitch_shell_out = Pitch_shell_kp*(RC_Pitch-pitch) + Pitch_shell_ki*Pitch_i ;//+ Pitch_shell_kd*Pitch_d;
//角度保存
  //Pitch_old=pitch0;
///////////////////////////////////////////////////////////      
       
        RC_Roll=(Control_ROLL-50)*1;
        Roll_i+=(RC_Roll-roll);
//-------------Roll积分限幅----------------//
  if(Roll_i>150) Roll_i=150;
  else if(Roll_i<-150) Roll_i=-150;
//-------------Roll微分--------------------//
  //Roll_d=roll0-Roll_old;
//-------------Roll  PID-------------------//
  Roll_shell_out  = Roll_shell_kp*(RC_Roll-roll) + Roll_shell_ki*Roll_i ;//+ Roll_shell_kd*Roll_d;
//------------Roll角度保存------------------//
  //Roll_old=roll0;    
       
  RC_Yaw=(Control_YAW-50)*0.8;
  Yaw_i+=(RC_Yaw-yaw);
  if(Yaw_i>300) Yaw_i=300;
  else if(Yaw_i<-300) Yaw_i=-300;
//-------------Yaw微分--------------------//
  //Yaw_d=gyro_z-Yaw_old;
//-------------YAW  PID-------------------//
  Yaw_shell_out  = Yaw_shell_kp*(RC_Yaw-yaw) + Yaw_shell_ki*Yaw_i ;//+ Yaw_shell_kd*Yaw_d;
//------------YAW角度保存------------------//
  //Yaw_old=gyro_z;
       
       
        ////////////////////////内环角速度环(PD)///////////////////////////////    
  //积分//
  pit_i+=(Pitch_shell_out - gyro_y * 1);
  if(pit_i>300) pit_i=300;
  else if (pit_i<-300) pit_i=-300;
  Pitch_core_ki_out = PID_PIT.I * pit_i;
  Pitch_core_kp_out = PID_PIT.P * (Pitch_shell_out + gyro_y * 1);  //3.5
  Pitch_core_kd_out = PID_PIT.D * (gyro_y   - Gyro_radian_old_y);
  //积分//
  rol_i+=(Roll_shell_out  + gyro_x *1);
  if(rol_i>300) rol_i=300;
  else if (rol_i<-300) rol_i=-300;
  Roll_core_ki_out  = -PID_ROL.I * rol_i;
  Roll_core_kp_out  = PID_ROL.P  * (Roll_shell_out + gyro_x *1);  //3.5
  Roll_core_kd_out  = PID_ROL.D  * (gyro_x  - Gyro_radian_old_x);
  //积分//
  pit_i+=(Pitch_shell_out - gyro_y * 1);
  if(pit_i>300) pit_i=300;
  else if (pit_i<-300) pit_i=-300;
  Yaw_core_ki_out  = upload_PID_yaw_I * pit_i;
  Yaw_core_kp_out  = upload_PID_yaw_P  * (Yaw_shell_out + gyro_z * 1);
  Yaw_core_kd_out  = upload_PID_yaw_D  * (gyro_z - Gyro_radian_old_z);
       
       
  Pitch_core_out = Pitch_core_kp_out + Pitch_core_kd_out+Pitch_core_ki_out;
  Roll_core_out  = Roll_core_kp_out  + Roll_core_kd_out+Roll_core_ki_out;
  Yaw_core_out   = Yaw_core_kp_out   + Yaw_core_kd_out+Yaw_core_ki_out;

  Gyro_radian_old_y =  gyro_y;
  Gyro_radian_old_x = gyro_x;
  Gyro_radian_old_z = gyro_z ;   //储存历史值
       
//--------------------将输出值融合到四个电机--------------------------------//

       
    if(R_THROTTLE>1200)
  {
          thr=R_THROTTLE- 1000;       
        
                moto1 = R_THROTTLE - 1000 + Roll_core_out - Pitch_core_out - Yaw_core_out-2;
		moto2 = R_THROTTLE - 1000 + Roll_core_out + Pitch_core_out + Yaw_core_out;
		moto3 = R_THROTTLE - 1000 - Roll_core_out + Pitch_core_out - Yaw_core_out+8;
		moto4 = R_THROTTLE - 1000 - Roll_core_out - Pitch_core_out + Yaw_core_out;
        
        if(moto1<=10)
          moto1=10;
        if(moto2<=10)
          moto2=10;
        if(moto3<=10)
          moto3=10;
        if(moto4<=10)
          moto4=10;
                       
  }
  
        else
        {
                moto1 = 0;
                moto2 = 0;
                moto3 = 0;
                moto4 = 0;
        }
              
        MOTO_PWMRFLASH(moto1,moto2,moto3,moto4);//        Moto_PwmRflash(moto1,moto2,moto3,moto4);
}


/////////  第 二 套 ////////

/*
void CONTROL()
{
	//u16 moto1=0,moto2=0,moto3=0,moto4=0;
  

	
	Roll_core_kp_out = PID_ROL.P * roll0;
	Roll_core_kd_out = PID_ROL.D * gyro_x;
	
	pitch_core_kp_out = PID_PIT.P * pitch0;
	pitch_core_kd_out = PID_PIT.D * gyro_y;
	
	Yaw_core_kd_out = PID_YAW.D * gyro_z;
	
	Roll_core_out = Roll_core_kp_out + Roll_core_kd_out;
	Pitch_core_out = pitch_core_kp_out + pitch_core_kd_out;
	Yaw_core_out = Yaw_core_kd_out;
	
	if(R_THROTTLE>1200)
	{
		moto1 = R_THROTTLE - 1000 - Roll_core_out - Pitch_core_out + Yaw_core_out;
		moto2 = R_THROTTLE - 1000 + Roll_core_out - Pitch_core_out - Yaw_core_out;
		moto3 = R_THROTTLE - 1000 + Roll_core_out + Pitch_core_out + Yaw_core_out;
		moto4 = R_THROTTLE - 1000 - Roll_core_out + Pitch_core_out - Yaw_core_out;
	}
	else
	{
		moto1 = 0;
		moto2 = 0;
		moto3 = 0;
		moto4 = 0;
	}
        
        
        
        MOTO_PWMRFLASH(moto1,moto2,moto3,moto4);
        
        
	                      //加一句  遥控电机开关
}
*/
/////第三套/////
/*
int16_t getlast_roll=0,geilast_pitch=0;
float rol_i=0,pit_i=0,yaw_p=0;

void CONTROL()
{
	float a_rol,a_pit,a_yaw;
	a_rol = roll0 - (Control_ROLL-1500)/12;
	a_pit = pitch0 + (Control_PITCH-1500)/12;
	
	rol_i += roll0;
	if(rol_i>500)
	rol_i=500;
	if(rol_i<-500)
	rol_i=-500;

	PID_ROL.pout = PID_ROL.P * a_rol;
	PID_ROL.dout = -(PID_ROL.D/100) * gyro_y;
	PID_ROL.iout = (PID_ROL.I/100) * rol_i;//PID_ROL.dout;

	pit_i += a_pit;
	if(pit_i>2000)
	pit_i=2000;
	if(pit_i<-2000)
	pit_i=-2000;

	PID_PIT.pout = PID_PIT.P * a_pit;
	PID_PIT.dout = (PID_PIT.D/100) * gyro_x;
	PID_PIT.iout = (PID_PIT.I/100) * pit_i;
	if(Control_YAW<1400||Control_YAW>1600)
	{
          gyro_z = gyro_z + ( Control_YAW - 1500 )*2;
        }
	yaw_p += gyro_z * 0.0609756f * 0.002f;// +(Rc_Get.YAW-1500)*30
	if(yaw_p>20)
		yaw_p=20;
	if(yaw_p<-20)
		yaw_p=-20;


	PID_YAW.pout=PID_YAW.P*yaw_p;
	PID_YAW.dout = PID_YAW.D * gyro_z;				   
	
	if(R_THROTTLE<1200)
	{		
		pit_i=0;
		rol_i=0;
		yaw_p=0;
	}

	PID_ROL.OUT =  (-PID_ROL.pout)-PID_ROL.iout +PID_ROL.dout;//
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
	PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;
 
	if(R_THROTTLE>1200)
	{
		moto1 = R_THROTTLE - 1000 + PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;
		moto2 = R_THROTTLE - 1000 + PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;
		moto3 = R_THROTTLE - 1000 - PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;
		moto4 = R_THROTTLE - 1000 - PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;
	}
	else
	{
		moto1 = 0;
		moto2 = 0;
		moto3 = 0;
		moto4 = 0;
	}
        if(moto1<=10)
          moto1=10;
        if(moto2<=10)
          moto2=10;
        if(moto3<=10)
          moto3=10;
        if(moto4<=10)
          moto4=10;
	MOTO_PWMRFLASH(moto1,moto2,moto3,moto4);
}
*/

u16 Motor1,Motor2,Motor3,Motor4;
void MOTO_PWMRFLASH(double motor1,double motor2,double motor3,double motor4)
{
  ///////限幅////////
  if(motor1>=65500)
    motor1=65500;
  if(motor2>=65500)
    motor2=65500;
  if(motor3>=65500)
    motor3=65500;
  if(motor4>=65500)
    motor4=65500;
  
  
  
  
  Motor1=65535-motor1;
  Motor2=65535-motor2;
  Motor3=65535-motor3;
  Motor4=65535-motor4;
  
  TIM_SetCompare1(TIM3,Motor2);
  TIM_SetCompare2(TIM3,Motor3);
  
  TIM_SetCompare3(TIM4,Motor4);
  TIM_SetCompare4(TIM4,Motor1);

}


