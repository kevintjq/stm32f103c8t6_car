#include "encoder.h"
#include "Serial.h"
#include "Motor.h"
extern Encoder encoder;
extern Motor motor;

void Motor_Test(int8_t vel)
{	
		if(vel >= 100){
			vel = 100;
		}
		if(vel <= -100){
			vel = -100;
		}
		
		if(vel >= 0 ){
			HAL_GPIO_WritePin(M1_1_GPIO_Port, M1_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M1_2_GPIO_Port, M1_2_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,vel);
		}else{
			HAL_GPIO_WritePin(M1_1_GPIO_Port, M1_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(M1_2_GPIO_Port, M1_2_Pin, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,-vel);
		}
}


void Motor_Task(void* parameter)
{	
	while(1){
		if(1){
			float pwm1 = motor.motor1.pwmVel;
			float pwm2 = motor.motor2.pwmVel;
			
			if(pwm1 >= 100){
				pwm1 = 100;
			}
			if(pwm1 <= -100){
				pwm1 = -100;
			}
			
			if(pwm2 >= 100){
				pwm2 = 100;
			}
			if(pwm2 <= -100){
				pwm2 = -100;
			}
			
			if(pwm1 >= 0 ){
				HAL_GPIO_WritePin(M1_1_GPIO_Port, M1_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M1_2_GPIO_Port, M1_2_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,pwm1);
			}else{
				HAL_GPIO_WritePin(M1_1_GPIO_Port, M1_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M1_2_GPIO_Port, M1_2_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,-pwm1);
			}
			
			if(pwm2 >= 0 ){
				HAL_GPIO_WritePin(M2_1_GPIO_Port, M2_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(M2_2_GPIO_Port, M2_2_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,pwm2);
			}else{
				HAL_GPIO_WritePin(M2_1_GPIO_Port, M2_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(M2_2_GPIO_Port, M2_2_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,-pwm2);
			}
		}
		vTaskDelay(20); 

	}
}