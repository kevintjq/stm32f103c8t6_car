#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

void Motor_Task(void* parameter);
void Motor_Test(int8_t vel);
static TaskHandle_t Motor_Task_Handle;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
	
typedef struct
{
	float pwmVel;
	float vel;
	float trk;
	uint64_t last_Val;
}Motor1;

typedef struct
{
	float pwmVel;
	float vel;
	float trk;
	uint64_t last_Val;
}Motor2;

typedef struct
{
	Motor1 motor1;
	Motor2 motor2;
}Motor;


