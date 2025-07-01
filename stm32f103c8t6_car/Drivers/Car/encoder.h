#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

void Encoder_Task(void* parameter);
static TaskHandle_t Encoder_Task_Handle;

typedef struct
{
	uint64_t Encoder_Num;
	int64_t Encoder_Val;
	uint64_t last_Val;
	int8_t Encoder_Dir;
	float d;
	float last_d;
	float Vel;
	float last_Vel;
	float Odometer;
}M1;

typedef struct
{
	uint64_t Encoder_Num;
	int64_t Encoder_Val;
	uint64_t last_Val;
	int8_t Encoder_Dir;
	float d;
	float last_d;
	float Vel;
	float last_Vel;
	float Odometer;
}M2;

typedef struct
{
	M1 m1;
	M2 m2;
}Encoder;

void RstEncoderVal(uint8_t m);
void RstAllEncoderVal();
void RstEncoderNum(uint8_t m);
void RstAllEncoderNum();
void RstEncoder(uint8_t m);
void RstAllEncoder();

