#include "main.h"
#include "Serial.h"
#ifdef __GNUC__

  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}

int fputc(int ch, struct __FILE *f)  
{  
  /* e.g. write a character to the USART */  
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);   
  return ch;  
}  