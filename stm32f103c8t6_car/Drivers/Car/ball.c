#include "pid.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"
#include "Motor.h"
#include "stdio.h"
#include "ball.h"

extern UART_HandleTypeDef huart2;  // ����2���
extern Pid pid;            // PID���ƽṹ��
extern Encoder encoder;
extern Motor motor;

uint8_t rx_buffer[8];             // ���ջ�����,��չΪ8�ֽ�
uint8_t rx_index = 0;             // ������������
uint8_t rx_temp;                  // ��ʱ���ձ���

// ��������ٶȺͷ���
void parseMotorControl(uint8_t *data, int *speed, uint8_t isRightWheel)
{
    // ��ȡ����λ(ǰ��λ)���ٶ�ֵ(����λ)
    int direction = (data[0] - '0') * 10 + (data[1] - '0');
    int value = (data[2] - '0') * 10 + (data[3] - '0');
    
    // ���û����ٶ�
    if(direction == 1)  // 01��ʾǰ��
    {
        *speed = value;
    }
    else if(direction == 2)  // 02��ʾ����
    {
        *speed = -value;
    }
    else  // ����ֵͣ
    {
        *speed = 0;
    }
    
    // ��������֣���Ҫ��ת����
    if(isRightWheel)
    {
        *speed = -(*speed);
    }
}

// ���ڽ��ջص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // ����Ƿ��������ַ�
        if(rx_temp >= '0' && rx_temp <= '9')
        {
            rx_buffer[rx_index++] = rx_temp;
            
            // ���յ�8�����ֺ���д���
            if(rx_index >= 8)
            {
                int left_speed = 0;
                int right_speed = 0;
                
                // �������ֿ���(ǰ4λ)�����ֲ���Ҫ��ת
                parseMotorControl(&rx_buffer[0], &left_speed, 0);
                
                // �������ֿ���(��4λ)��������Ҫ��ת
                parseMotorControl(&rx_buffer[4], &right_speed, 1);
                
                // ���õ���ٶ�
                pid.control1.target_v = left_speed;
                pid.control2.target_v = right_speed;
                
                // ���ý�������
                rx_index = 0;
            }
        }
        else
        {
            // ����յ��������ַ�,���ý�������
            rx_index = 0;
        }
        
        // ������������
        HAL_UART_Receive_IT(huart, &rx_temp, 1);
    }
}

// ͨ������
void Ball_Task(void *argument)
{
    // �������ڽ���
    HAL_UART_Receive_IT(&huart2, &rx_temp, 1);
    
    while(1)
    {
        vTaskDelay(20);
    }
}