#include "encoder.h"
/*
��ʱ����Ƶ��1
�����MG513X��������������
���ٱȣ�1/28
������������13
��ֱ̥��d=65mm
��̥תһȦ������������Ϊ��1*28*13 = 364
��̥�ܳ���d=0.065*3.1415926=0.204203519
ÿһ�����������̥�˶�0.000560998678571�ף�0.0560998678571����
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
