#include "encoder.h"
/*
定时器倍频：1
电机：MG513X（霍尔编码器）
减速比：1/28
编码器线数：13
轮胎直径d=65mm
轮胎转一圈计数器脉冲数为：1*28*13 = 364
轮胎周长πd=0.065*3.1415926=0.204203519
每一个脉冲等于轮胎运动0.000560998678571米，0.0560998678571厘米
*/
extern Encoder encoder;
static float l =  0.0560998678571;

void Encoder_Task(void* parameter)
{		
    while (1)
    {	
				float d1 = (encoder.m1.Encoder_Num - encoder.m1.last_Val)*l*((int)(encoder.m1.Encoder_Dir));
				encoder.m1.d = 0.5*d1+0.5*encoder.m1.last_d;
				encoder.m1.last_d = encoder.m1.d;
				encoder.m1.last_Val = encoder.m1.Encoder_Num;
				float vel1 = encoder.m1.d*50; 
				encoder.m1.Vel = 0.5*vel1+0.5*encoder.m1.last_Vel;
				encoder.m1.last_Vel = encoder.m1.Vel;
				encoder.m1.Odometer += d1;
				
				float d2 = (encoder.m2.Encoder_Num - encoder.m2.last_Val)*l*((int)(encoder.m2.Encoder_Dir));
				encoder.m2.d = 0.5*d2+0.5*encoder.m2.last_d;
				encoder.m2.last_d = encoder.m2.d;
				encoder.m2.last_Val = encoder.m2.Encoder_Num;
				float vel2 = encoder.m2.d*50; 
				encoder.m2.Vel = 0.5*vel2+0.5*encoder.m2.last_Vel;
				encoder.m2.last_Vel = encoder.m2.Vel;
				encoder.m2.Odometer += d2;
			
				vTaskDelay(20);   
    }
}


void RstEncoderVal(uint8_t m){
	switch(m){
		case 1:
			encoder.m1.Encoder_Val = 0;
		break;
		case 2:
			encoder.m2.Encoder_Val = 0;
		break;
		default:
		break;
	}
}

void RstAllEncoderVal(){
	encoder.m1.Encoder_Val = 0;
	encoder.m2.Encoder_Val = 0;
}

void RstEncoderNum(uint8_t m){
	switch(m){
		case 1:
			encoder.m1.Encoder_Num = 0;
		break;
		case 2:
			encoder.m2.Encoder_Num = 0;
		break;
		default:
		break;
	}
}
void RstAllEncoderNum(){
	encoder.m1.Encoder_Num = 0;
	encoder.m2.Encoder_Num = 0;
}
void RstEncoder(uint8_t m){
	switch(m){
		case 1:
			encoder.m1.Encoder_Val = 0;
			encoder.m1.Encoder_Num = 0;
		break;
		case 2:
			encoder.m2.Encoder_Val = 0;
			encoder.m2.Encoder_Num = 0;
		break;
		default:
		break;
	}
}
void RstAllEncoder(){
	encoder.m1.Encoder_Val = 0;
	encoder.m2.Encoder_Val = 0;
	encoder.m1.Encoder_Num = 0;
	encoder.m2.Encoder_Num = 0;
}
