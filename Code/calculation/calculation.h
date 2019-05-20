#ifndef _calculation_H
#define _calculation_H

typedef struct PID{float P,pout,I,iout,D,dout,IMAX,OUT;}PID;

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void IMUupdate(float gxi, float gyi, float gzi, float axi, float ayi, float azi);
void Yijielvbo(float angle_x, float gyro_x,float angle_y, float gyro_y,float angle_z, float gyro_z);//采集后计算的角度和角加速度
void Prepare_Data(short angle_x,short angle_y,short angle_z);

void CONTROL();
void MOTO_PWMRFLASH(double motor1,double motor2,double motor3,double motor4);
void PID_init();
//void updatePID(float rol_P,float rol_I,float rol_D,float pit_P,float pit_I,float pit_D,float yaw_P,float yaw_I,float yaw_D);
void CONTROL();

#endif
