#include "pid.h"
#include "encoder.h"
#include "Motor.h"
#include <stdio.h>
float vkp = 0.5;
float vki = 0.05;
float vkd = 0;
float xkp = 1.0;
float xki = 0.0;
float xkd = 0.25;
Pid pid;
extern Encoder encoder;
extern Motor motor;



void Pid_Task(void* parameter){
	pid.control1.target_v = 0;
	pid.control2.target_v = 0;
	while(1){
		if(1){
			/*
			float target_v1 = pid.control1.target_v;
			pid.control1.xe = pid.control1.target_x - encoder.m1.Odometer;
			target_v1 += xkp*(pid.control1.xe-pid.control1.xe1)+xki*pid.control1.xe+xkd*(pid.control1.xe-2*pid.control1.xe1+pid.control1.xe2);
			
			if(target_v1>=130){
				pid.control1.target_v = 130;
			}else{
				pid.control1.target_v = target_v1;
			}
			if(target_v1<=-130){
				pid.control1.target_v = -130;
			}else{
				pid.control1.target_v = target_v1;
			}
			pid.control1.xe2 = pid.control1.xe1;
			pid.control1.xe1 = pid.control1.xe;
			
			float target_v2 = pid.control2.target_v;
			pid.control2.xe = pid.control2.target_x - encoder.m2.Odometer;
			target_v2 += xkp*(pid.control2.xe-pid.control2.xe1)+xki*pid.control2.xe+xkd*(pid.control2.xe-2*pid.control2.xe1+pid.control2.xe2);
			
			if(target_v2>=130){
				pid.control2.target_v = 130;
			}else{
				pid.control2.target_v = target_v2;
			}
			if(target_v2<=-130){
				pid.control2.target_v = -130;
			}else{
				pid.control2.target_v = target_v2;
			}
			pid.control2.xe2 = pid.control2.xe1;
			pid.control2.xe1 = pid.control2.xe;
			
			*/
			pid.control1.ve = pid.control1.target_v - encoder.m1.Vel;
			motor.motor1.pwmVel -= vkp*(pid.control1.ve-pid.control1.ve1)+vki*pid.control1.ve+vkd*(pid.control1.ve-2*pid.control1.ve1+pid.control1.ve2);
			pid.control1.ve2 = pid.control1.ve1;
			pid.control1.ve1 = pid.control1.ve;
			
			pid.control2.ve = pid.control2.target_v - encoder.m2.Vel;
			motor.motor2.pwmVel += vkp*(pid.control2.ve-pid.control2.ve1)+vki*pid.control2.ve+vkd*(pid.control2.ve-2*pid.control2.ve1+pid.control2.ve2);
			pid.control2.ve2 = pid.control2.ve1;
			pid.control2.ve1 = pid.control2.ve;
			//motor.motor1.pwmVel=-motor.motor2.pwmVel;
		}
		vTaskDelay(20);
			//printf("Num=%llu,Pos=%f,Vel=%f,ZF=%d\n",encoder.m1.Encoder_Num,encoder.m1.Odometer,encoder.m1.Vel,encoder.m1.Encoder_Dir);
	}
}






