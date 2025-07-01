#include "pid.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "encoder.h"
#include "Motor.h"
#include "stdio.h"
#include "ball.h"

extern UART_HandleTypeDef huart2;  // 串口2句柄
extern Pid pid;            // PID控制结构体
extern Encoder encoder;
extern Motor motor;

uint8_t rx_buffer[8];             // 接收缓冲区,扩展为8字节
uint8_t rx_index = 0;             // 接收数据索引
uint8_t rx_temp;                  // 临时接收变量

// 解析电机速度和方向
void parseMotorControl(uint8_t *data, int *speed, uint8_t isRightWheel)
{
    // 获取方向位(前两位)和速度值(后两位)
    int direction = (data[0] - '0') * 10 + (data[1] - '0');
    int value = (data[2] - '0') * 10 + (data[3] - '0');
    
    // 设置基础速度
    if(direction == 1)  // 01表示前进
    {
        *speed = value;
    }
    else if(direction == 2)  // 02表示后退
    {
        *speed = -value;
    }
    else  // 其他值停
    {
        *speed = 0;
    }
    
    // 如果是右轮，需要反转方向
    if(isRightWheel)
    {
        *speed = -(*speed);
    }
}

// 串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // 检查是否是数字字符
        if(rx_temp >= '0' && rx_temp <= '9')
        {
            rx_buffer[rx_index++] = rx_temp;
            
            // 接收到8个数字后进行处理
            if(rx_index >= 8)
            {
                int left_speed = 0;
                int right_speed = 0;
                
                // 解析左轮控制(前4位)，左轮不需要反转
                parseMotorControl(&rx_buffer[0], &left_speed, 0);
                
                // 解析右轮控制(后4位)，右轮需要反转
                parseMotorControl(&rx_buffer[4], &right_speed, 1);
                
                // 设置电机速度
                pid.control1.target_v = left_speed;
                pid.control2.target_v = right_speed;
                
                // 重置接收索引
                rx_index = 0;
            }
        }
        else
        {
            // 如果收到非数字字符,重置接收索引
            rx_index = 0;
        }
        
        // 重新启动接收
        HAL_UART_Receive_IT(huart, &rx_temp, 1);
    }
}

// 通信任务
void Ball_Task(void *argument)
{
    // 启动串口接收
    HAL_UART_Receive_IT(&huart2, &rx_temp, 1);
    
    while(1)
    {
        vTaskDelay(20);
    }
}