#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
void Pid_Task(void* parameter);
static TaskHandle_t Pid_Task_Handle;

typedef struct
{
	float ve;
	float ve1;
	float ve2;
	float xe;
	float xe1;
	float xe2;
	float target_v;
	float target_x;
}Control1;

typedef struct
{
	float ve;
	float ve1;
	float ve2;
	float xe;
	float xe1;
	float xe2;
	float target_v;
	float target_x;
}Control2;

typedef struct
{
	Control1 control1;
	Control2 control2;
}Pid;