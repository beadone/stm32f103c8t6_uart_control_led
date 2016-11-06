//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "ExceptionHandlers.h"
#include "stm32f10x_conf.h"
#include "misc.h"

#define NUM 10
 int i,j;
  char name[NUM+1] = {'\0'};
  ErrorStatus HSEStartUpStatus;

  uint8_t Usart1Get(void);
  void Usart1Put(uint8_t ch);
  void Usart1Init(void);
  void UART1Send(const unsigned char *pucBuffer, unsigned long ulCount);

  uint8_t Usart2Get(void);
  void Usart2Put(uint8_t ch);
  void Usart2Init(void);
  void UART2Send(const unsigned char *pucBuffer, unsigned long ulCount);
  void USART2_IRQHandler(void);


// ----------------------------------------------------------------------------

//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


 unsigned char gotachar;

int
main(int argc, char* argv[])
{

  timer_start();

  blink_led_init();
  Usart1Init();
  Usart2Init();




  // Infinite loop
  while (1)
    {

    }
  // Infinite loop, never return.
}



void blink_led_init()
{
  // Enable GPIO Peripheral clock
  RCC_APB2PeriphClockCmd(BLINK_RCC_MASKx(BLINK_PORT_NUMBER), ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin in output push/pull mode
  GPIO_InitStructure.GPIO_Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BLINK_GPIOx(BLINK_PORT_NUMBER), &GPIO_InitStructure);

  // Start with led turned off
  blink_led_off();
}

#if defined(USE_HAL_DRIVER)
void HAL_IncTick(void);
#endif

// Forward declarations.

void
timer_tick (void);

// ----------------------------------------------------------------------------

volatile timer_ticks_t timer_delayCount;

// ----------------------------------------------------------------------------

void
timer_start (void)
{
  // Use SysTick as reference for the delay loops.
  SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);
}

void
timer_sleep (timer_ticks_t ticks)
{
  timer_delayCount = ticks;

  // Busy wait until the SysTick decrements the counter to zero.
  while (timer_delayCount != 0u)
    ;
}

void
timer_tick (void)
{
  // Decrement to zero the counter used by the delay routine.
  if (timer_delayCount != 0u)
    {
      --timer_delayCount;
    }
}

// ----- SysTick_Handler() ----------------------------------------------------

void
SysTick_Handler (void)
{
#if defined(USE_HAL_DRIVER)
  HAL_IncTick();
#endif
  timer_tick ();
}





void Usart1Init(void){


  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStructure;
  /* Enable clock for USART1, AFIO and GPIOA */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  //Set USART1 Tx (PA.09) as AF push-pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
   /* GPIOA PIN9 alternative function Tx */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


   /* Baud rate 9600, 8-bit data, One stop bit
    * No parity, Do both Rx and Tx, No HW flow control
    */
   /* USART configuration structure for USART1 */
   USART_ClockStructInit(&USART_ClockInitStructure);
    USART_ClockInit(USART1, &USART_ClockInitStructure);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //Write USART1 parameters
    USART_Init(USART1, &USART_InitStructure);
    //Enable USART1
    USART_Cmd(USART1, ENABLE);




}
void Usart1Put(uint8_t ch)
{
      USART_SendData(USART1, (uint8_t) ch);
      //Loop until the end of transmission
      while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
      {
      }
}

uint8_t Usart1Get(void){
     while ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
        return (uint8_t)USART_ReceiveData(USART1);
}


void UART1Send(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART1, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}


void Usart2Init(void){


  GPIO_InitTypeDef GPIO_InitStructure;

  USART_InitTypeDef USART_InitStructure;

  USART_ClockInitTypeDef USART_ClockInitStructure;
  /* Enable clock for USART2, AFIO and GPIOA */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  /* GPIOA PIN 2  alternative function Tx */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  //output push-pull
  //GPIO_Mode_Out_PP
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //
   //setup the receive pin
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

   /* Baud rate 9600, 8-bit data, One stop bit
    * No parity, Do both Rx and Tx, No HW flow control
    */
   /* USART configuration structure for USART2 */
    USART_ClockStructInit(&USART_ClockInitStructure);
    USART_ClockInit(USART2, &USART_ClockInitStructure);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    //Write USART2 parameters

    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig( USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig( USART2, USART_IT_TXE, ENABLE);
    //Enable USART2>>>>>>>>>>>>check this
    USART_Cmd(USART2, ENABLE);

    //    //NVIC
        NVIC_InitTypeDef NVIC_InitStructure;
    //
    //        /* Enable the USARTx Interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        NVIC_EnableIRQ(USART2_IRQn);

}
void USART2_IRQHandler()
{

  unsigned char received;



    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // Received characters modify string
      {

      received = (unsigned char)USART_ReceiveData(USART2);
      gotachar = received;
      USART_ClearITPendingBit(USART2, USART_IT_RXNE);
      //USART_SendData(USART2, gotachar);  //stdout
      USART_SendData(USART2, gotachar);
      USART_SendData(USART1, received);  //debug
      //USART_SendData(USART1, 0xD);
      //USART_SendData(USART1, 0xA);

      }



}

void Usart2Put(uint8_t ch)
{
      USART_SendData(USART2, (uint8_t) ch);
      //Loop until the end of transmission
      while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
      {
      }
}

uint8_t Usart2Get(void){
     while ( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
        return (uint8_t)USART_ReceiveData(USART2);
}

void UART2Send(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART2, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        {
        }
    }
}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
