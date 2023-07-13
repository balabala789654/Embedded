#include "main.h"
/*
	a = Range * G * ACC / 32768 某个轴的加速度  其中G是当地重力加速度，ACC是该轴的加速度原始数据，Range是由配置加速度量程时写入的参数决定的
	g = Range * Gyro / 32768 	某个轴的角速度 	Gyro是这个轴的角速度原始数据，Range是由配置角速度量程时写入的参数决定的
*/

float old_yaw, new_yaw;
float pitch=0;
float yaw=0;
float roll=0;

short accx,accy,accz; 	//各个轴的加速度原始值
float ax,ay,az;		//各个轴的计算后的加速度值

short gyrox,gyroy,gyroz;
int a;

void MPU_task(void *pvParameters)
{
	
	while(1)
	{
		
		a = mpu_dmp_get_data(&pitch,&roll,&new_yaw);
		MPU_Get_Accelerometer(&accx, &accy, &accz);
		
		float Difference=new_yaw-old_yaw;
		ax = 9.8*accx*2/32768.0f;
		ay = 9.8*accy*2/32768.0f;
		az = 9.8*accz*2/32768.0f;
		
//		_vx += ax * (double)0.00001;
//		_vy += ay * (double)0.00001;
		
		/* 由于mpu_dmp_get_data()函数中取值范围为(-180,180)导致姿态角有突变，加入下两行代码使姿态角累加即可解决 */
		if(Difference<-300) Difference = 360+new_yaw-old_yaw;
		else if(Difference>300) Difference = (new_yaw-360)-old_yaw;
		
		yaw += Difference;
		
		old_yaw=new_yaw;
		
		current_yaw = yaw*PI/180.0f;//用于计算yaw变化速率
		
		vTaskDelay(1);

	}
}

