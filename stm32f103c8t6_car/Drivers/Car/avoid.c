#include "main.h"
#include <string.h>
#include <stdio.h>
#include "avoid.h"
#include "Serial.h"
//extern UART_HandleTypeDef huart3;  // USART3: ����ģ��ͨ��
extern UART_HandleTypeDef huart2;  // USART2: ���ڴ�ӡ����


void Avoid_Task(void* parameter)
{
    while (1) 
    {   const char *testStr = "Avoid_Task running\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)testStr, strlen(testStr), 100);
        printf("Avoid_Task running\r\n");
        
        vTaskDelay(20);  // ������ʱ 20ms
    }
}
