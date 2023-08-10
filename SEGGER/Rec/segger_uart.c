/**
 * @file segger_uart.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Terminal control for Flasher using USART1(PA9/PA10)(default) /USART2 (PA2/PA3)
 * @date 2023-02-09
 */

/*
 * This source file is modified from
 * SysView_UARTSample_STM32F407/Application/HIF_UART.c
 * (https://wiki.segger.com/images/0/06/SysView_UARTSample_STM32F407.zip)
 */

#include "SEGGER_SYSVIEW.h"

#if (SEGGER_UART_REC == 1)
#include "SEGGER_RTT.h"
#include "stm32f4xx.h"

#define UART_BASECLK          (84000000) //note that max clock of APB1 (USART2) is 42 MHz
#define USART_RX_ERROR_FLAGS  0x0B

// #define SEGGER_USE_USART2

#ifdef SEGGER_USE_USART2
  #define USART_REG             USART2
  #define USART_RX_BIT          3
  #define USART_TX_BIT          2
  #define USART_IRQn            USART2_IRQn
#else
  #define USART_REG             USART1
  #define USART_RX_BIT          10
  #define USART_TX_BIT          9
  #define USART_IRQn            USART1_IRQn
#endif

typedef void UART_ON_RX_FUNC(uint8_t Data);
typedef int  UART_ON_TX_FUNC(uint8_t* pChar);

typedef UART_ON_TX_FUNC* UART_ON_TX_FUNC_P;
typedef UART_ON_RX_FUNC* UART_ON_RX_FUNC_P;

static UART_ON_RX_FUNC_P _cbOnRx;
static UART_ON_TX_FUNC_P _cbOnTx;

#define _SERVER_HELLO_SIZE (4)
#define _TARGET_HELLO_SIZE (4)

void HIF_UART_Init(uint32_t Baudrate, UART_ON_TX_FUNC_P cbOnTx, UART_ON_RX_FUNC_P cbOnRx);

/* 
 * "Hello" message expected by SysView: [ 'S', 'V',
 * <PROTOCOL_MAJOR>, <PROTOCOL_MINOR> ]
 */
static const uint8_t _abHelloMsg[_TARGET_HELLO_SIZE] = {
	'S', 'V', (SEGGER_SYSVIEW_VERSION / 10000),
	(SEGGER_SYSVIEW_VERSION / 1000) % 10};

static struct {
  uint8_t NumBytesHelloRcvd;
  uint8_t NumBytesHelloSent;
  int           ChannelID;
} _SVInfo = {0,0,1};

static void _StartSysView(void)
{
  int r;

  r = SEGGER_SYSVIEW_IsStarted();
  if (r == 0) {
    SEGGER_SYSVIEW_Start();
  }
}

static void _cbOnUARTRx(uint8_t Data)
{
  if (_SVInfo.NumBytesHelloRcvd < _SERVER_HELLO_SIZE) {  // Not all bytes of <Hello> message received by SysView yet?
    _SVInfo.NumBytesHelloRcvd++;
    goto Done;
  }
  _StartSysView();
  SEGGER_RTT_WriteDownBuffer(_SVInfo.ChannelID, &Data, 1);  // Write data into corresponding RTT buffer for application to read and handle accordingly
Done:
  return;
}

static int _cbOnUARTTx(uint8_t* pChar)
{
  int r;

  if (_SVInfo.NumBytesHelloSent < _TARGET_HELLO_SIZE) {  // Not all bytes of <Hello> message sent to SysView yet?
    *pChar = _abHelloMsg[_SVInfo.NumBytesHelloSent];
    _SVInfo.NumBytesHelloSent++;
    r = 1;
    goto Done;
  }
  r = SEGGER_RTT_ReadUpBufferNoLock(_SVInfo.ChannelID, pChar, 1);
  if (r < 0) {  // Failed to read from up buffer?
    r = 0;
  }
Done:
  return r;
}

void SEGGER_UART_init(unsigned long baud)
{
  HIF_UART_Init(baud, _cbOnUARTTx, _cbOnUARTRx);
}

/*********************************************************************
*
*       HIF_UART_WaitForTxEnd
*/
void HIF_UART_WaitForTxEnd(void)
{
  //
  // Wait until transmission has finished (e.g. before changing baudrate).
  //
  while (!(USART_REG->SR & USART_SR_TXE));  // Wait until transmit buffer empty (Last byte shift from data to shift register)
  while (!(USART_REG->SR & USART_SR_TC));   // Wait until transmission is complete
}

/*********************************************************************
*
*       USARTx_IRQHandler
*
*  Function description
*    Interrupt handler.
*    Handles both, Rx and Tx interrupts
*
*  Notes
*    (1) This is a high-prio interrupt so it may NOT use embOS functions
*        However, this also means that embOS will never disable this interrupt
*/
#ifdef SEGGER_USE_USART2
  void USART2_IRQHandler(void)
#else
  void USART1_IRQHandler(void)
#endif
{
  uint32_t UsartStatus;
  uint8_t v;
  int r;

  UsartStatus = USART_REG->SR;                              // Examine status register
  if (UsartStatus & USART_SR_RXNE) {               // Data received?
    v = USART_REG->DR;                                      // Read data
    if ((UsartStatus & USART_RX_ERROR_FLAGS) == 0) {   // Only process data if no error occurred
      (void)v;                                         // Avoid warning in BTL
      if (_cbOnRx) {
        _cbOnRx(v);
      }
    }
  }
  if (UsartStatus & USART_SR_TXE) {                // Tx (data register) empty? => Send next character Note: Shift register may still hold a character that has not been sent yet.
    //
    // Under special circumstances, (old) BTL of Flasher does not wait until a complete string has been sent via UART,
    // so there might be an TxE interrupt pending *before* the FW had a chance to set the callbacks accordingly which would result in a NULL-pointer call...
    // Therefore, we need to check if the function pointer is valid.
    //
    if (_cbOnTx == NULL) {  // No callback set? => Nothing to do...
      return;
    }
    r = _cbOnTx(&v);
    if (r == 0) {                          // No more characters to send ?
      USART_REG->CR1 &= ~USART_CR1_TXEIE;  // Disable further tx interrupts
    } else {
      USART_REG->SR;      // Makes sure that "transmission complete" flag in USART_SR is reset to 0 as soon as we write USART_DR. If USART_SR is not read before, writing USART_DR does not clear "transmission complete". See STM32F4 USART documentation for more detailed description.
      USART_REG->DR = v;  // Start transmission by writing to data register
    }
  }
}

/*********************************************************************
*
*       HIF_UART_EnableTXEInterrupt()
*/
void HIF_UART_EnableTXEInterrupt(void) {
  USART_REG->CR1 |= USART_CR1_TXEIE;  // enable Tx empty interrupt => Triggered as soon as data register content has been copied to shift register
}

/*********************************************************************
*
*       HIF_UART_Init()
*/
void HIF_UART_Init(uint32_t Baudrate, UART_ON_TX_FUNC_P cbOnTx, UART_ON_RX_FUNC_P cbOnRx) {
  uint32_t v;
  uint32_t Div;
  //
  // Configure USART RX/TX pins for alternate function AF7
  //
  #ifdef SEGGER_USE_USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 clock
  #else
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock
  #endif
  
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable IO port A clock
  v  = GPIOA->AFR[USART_TX_BIT >> 3];
  v &= ~(15UL << ((USART_TX_BIT & 0x7) << 2));
  v |=   (7UL << ((USART_TX_BIT & 0x7) << 2));
  GPIOA->AFR[USART_TX_BIT >> 3] = v;
  v  = GPIOA->AFR[USART_RX_BIT >> 3];
  v &= ~(15UL << ((USART_RX_BIT & 0x7) << 2));
  v |=   (7UL << ((USART_RX_BIT & 0x7) << 2));
  GPIOA->AFR[USART_RX_BIT >> 3] = v;
  //
  // Configure USART RX/TX pins output speed to high speed
  //
  v = GPIOA->OSPEEDR;
  v &= ~((3UL << (USART_TX_BIT << 1)) | (3UL << (USART_RX_BIT << 1)));
  v |= ((3UL << (USART_TX_BIT << 1)) | (3UL << (USART_RX_BIT << 1)));
  GPIOA->OSPEEDR = v;
  //
  // Configure USART RX/TX pins for alternate function usage
  v  = GPIOA->MODER;
  v &= ~((3UL << (USART_TX_BIT << 1)) | (3UL << (USART_RX_BIT << 1)));
  v |=  ((2UL << (USART_TX_BIT << 1)) | (2UL << (USART_RX_BIT << 1)));         // PA10: alternate function
  GPIOA->MODER = v;
  //
  // Initialize USART
  //
  USART_REG->CR1 = 0
            | (1 << 15)                         // OVER8  = 1; Oversampling by 8
            | (1 << 13)                         // UE     = 1; USART enabled
            | (0 << 12)                         // M      = 0; Word length is 1 start bit, 8 data bits
            | (0 << 10)                         // PCE    = 0; No parity control
            | (1 <<  5)                         // RXNEIE = 1; RXNE interrupt enabled
            | (1 <<  3)                         // TE     = 1; Transmitter enabled
            | (1 <<  2)                         // RE     = 1; Receiver enabled
            ;
  USART_REG->CR2 = 0
            | (0 << 12)                         // STOP = 00b; 1 stop bit
            ;
  USART_REG->CR3 = 0
            | (0 << 11)                         // ONEBIT = 0; Three sample bit method
            | (1 <<  7)                         // DMAT   = 1; DMA for transmitter enabled
            ;
  //
  // Set baudrate
  //
  Div = (UART_BASECLK << 1) / Baudrate;
  Div += (Div & 0x1) << 1; // rounding
  Div = (Div & 0xFFF0) | ((Div >> 1) & 0x7); // int[15:4], fraction[2:0]
  if (Div > 0xFFF7)
    Div = 0xFFF7;        // Limit maximum div = 255.125, if div over max
  else if (!Div)
    Div = 0xFFF0 & (Div << 4);	// Limit div = 0.125, if div is zero
  USART_REG->BRR = Div;
  //
  // Setup callbacks which are called by ISR handler and enable interrupt in NVIC
  //
  _cbOnRx = cbOnRx;
  _cbOnTx = cbOnTx;
  NVIC_SetPriority(USART_IRQn, 6);  // Highest prio, so it is not disabled by embOS
  NVIC_EnableIRQ(USART_IRQn);
}

#endif
