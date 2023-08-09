
/*
 * This source file is modified by
 * SysView_UARTSample_STM32F407/Application/HIF_UART.c
 * (https://wiki.segger.com/images/0/06/SysView_UARTSample_STM32F407.zip)
 */

#include "SEGGER_SYSVIEW.h"

#if (SEGGER_UART_REC == 1)
#include "SEGGER_RTT.h"
#include "stm32f4xx.h"
#include "string.h"

#define UART_BASECLK   (84000000) /* APB2 clock is 84 MHz */
#define USART_RX_ERROR_FLAGS  0x0B

#define USART_REG       USART1
#define USART_TX_BIT    9
#define USART_RX_BIT    10
#define USART_IRQn      USART1_IRQn

#define _SERVER_HELLO_SIZE (4)
#define _TARGET_HELLO_SIZE (4)

#define TX_BUFFER_SIZE  256

static char rx_byte;
static char tx_buffer[256];
/* 
 * "Hello" message expected by SysView: [ 'S', 'V',
 * <PROTOCOL_MAJOR>, <PROTOCOL_MINOR> ]
 */
static const char _sv_hello_msg[_TARGET_HELLO_SIZE] = {
	'S', 'V', (SEGGER_SYSVIEW_VERSION / 10000),
	(SEGGER_SYSVIEW_VERSION / 1000) % 10
};

static struct {
        uint8_t num_bytes_hello_rx;
        uint8_t hello_tx_finish;
        unsigned int buffer_id;
} _SV_info = {0, 0, 1};

/**
 * @brief Interrupt trigger when received byte in DR has been moved to rx_byte
 * 
 */
void DMA2_Stream5_IRQHandler(void)
{
        DMA2->HIFCR |= DMA_HISR_TCIF5; /* Clear TCIF */
        
        /* Only precess data if no error occured */
        if ((USART_REG->SR & USART_RX_ERROR_FLAGS) == 0) {
                /* Not all bytes of <Hello> message received by SysView yet? */
                if (_SV_info.num_bytes_hello_rx < _SERVER_HELLO_SIZE) {
                        _SV_info.num_bytes_hello_rx++;
                        return;
                }
                if (SEGGER_SYSVIEW_IsStarted() == 0)
                        SEGGER_SYSVIEW_Start();
                SEGGER_RTT_WriteDownBuffer(_SV_info.buffer_id, &rx_byte, 1);
        }
}

/**
 * @brief Transmit data in tx_buffer by USART DMA
 * 
 * @param size tx buffer size
 */
void USART_DMA_transmit(uint16_t size)
{
        DMA2->HIFCR |= DMA_HISR_TCIF7; /* Clear TCIF */

        /* Set number of data items to transfer */
        DMA2_Stream7->NDTR = (uint32_t)size;

        /* Read SR to clear TC and Check if last transmission is not done */
        while (!(USART_REG->SR & USART_SR_TC));

        /* Enable DMA */
        DMA2_Stream7->CR |= DMA_SxCR_EN;
}

void SEGGER_UART_TX_Callback(void)
{
        uint16_t tx_size;

        // Transmit <Hello> message to SysView not yet?
        if (_SV_info.hello_tx_finish == 0) {
                memcpy(tx_buffer, _sv_hello_msg, _TARGET_HELLO_SIZE);
                tx_size = _TARGET_HELLO_SIZE;
                _SV_info.hello_tx_finish = 1;
        } else {
                SEGGER_RTT_ReadUpBufferNoLock(_SV_info.buffer_id, tx_buffer,
                                              TX_BUFFER_SIZE);
        }
        if (tx_size > 0)
                USART_DMA_transmit(tx_size);
}

/**
 * @brief Interrupt trigger when the last TX byte has been moved to DR
 * 
 */
void DMA2_Stream7_IRQHandler(void)
{
        SEGGER_UART_TX_Callback();  
}

/**
 * @brief Initialize USART1 TX/RX in DMA mode to communicate with SysView
 *    TX: normal mode
 *    RX: circular mode (receive one byte immediately if data comes)
 * @param baudrate 
 */
void SEGGER_UART_init(unsigned long baudrate)
{
        uint32_t v, div;

        /* Enable PORTA and DMA2 clock */
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;

        /* Enable USART1 clock */
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

        /*   Initialize TX DMA (Stream 7, CH4)   */

        /* Wait DMA stream to disable if current tranfers are not finished */
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream7->CR & DMA_SxCR_EN);
        DMA2_Stream7->CR = 0;    

        /* Set peripheral address */
        DMA2_Stream7->PAR = (uint32_t)&USART1->DR;

        /* Set memory address */
        DMA2_Stream7->M0AR = (uint32_t)tx_buffer;

        /* Set DMA configuration */
        v = DMA_CHANNEL_4
          | DMA_MBURST_SINGLE   /* Memory burst single */
          | DMA_PBURST_SINGLE   /* Peripheral burst single */
          | DMA_PRIORITY_LOW
          | 0                   /* Peripheral increment offset: by PSIZE */
          | DMA_MDATAALIGN_BYTE /* Mememory data size */
          | DMA_PDATAALIGN_BYTE /* Peripheral data size */
          | DMA_MINC_ENABLE     /* Memory increment mode enable */
          | DMA_PINC_DISABLE    /* Peripheral increment mode disble */
          | DMA_NORMAL          /* Circular mode: disable */
          | DMA_MEMORY_TO_PERIPH /* Data transfer direction */
          | 0                   /* Flow controller: DMA */
          | DMA_SxCR_TCIE;      /* Transfer complete interrupt enable */
        DMA2_Stream7->CR |= v;

        /* 
         * Set priority of ISR handler and Enable interrupt in NVIC
         * Because of buffer mode, priority can be lower.
         * Priority in embOS: 1
         */
        NVIC_SetPriority(DMA2_Stream7_IRQn, 6);
        NVIC_EnableIRQ(DMA2_Stream7_IRQn);

        /*   Initialize RX DMA (Stream 5, CH4)   */

        /* Wait DMA stream to disable if current tranfers are not finished */
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        while (DMA2_Stream5->CR & DMA_SxCR_EN);
        DMA2_Stream5->CR = 0;
        /* Set peripheral address */
        DMA2_Stream5->PAR = (uint32_t)&USART1->DR;

        /* Set memory address */
        DMA2_Stream5->M0AR = (uint32_t)&rx_byte;

        /* Set the total number of data items to be transferred */
        DMA2_Stream5->NDTR = 1U;

        /* Set DMA configuration */
        v = DMA_CHANNEL_4
          | DMA_MBURST_SINGLE   /* Memory burst single */
          | DMA_PBURST_SINGLE   /* Peripheral burst single */
          | DMA_PRIORITY_VERY_HIGH
          | 0                   /* Peripheral increment offset: by PSIZE */
          | DMA_MDATAALIGN_BYTE /* Mememory data size */
          | DMA_PDATAALIGN_BYTE /* Peripheral data size */
          | DMA_MINC_DISABLE    /* Memory increment mode disable */
          | DMA_PINC_DISABLE    /* Peripheral increment mode disble */
          | DMA_CIRCULAR        /* Circular mode: disable */
          | DMA_PERIPH_TO_MEMORY /* Data transfer direction */
          | 0                   /* Flow controller: DMA */
          | DMA_SxCR_TCIE;      /* Transfer complete interrupt enable */
        DMA2_Stream5->CR |= v;

        /* 
         * Set priority of ISR handler and Enable interrupt in NVIC
         * Received data is stored in buffer immediately, so the priority
         * must be higher.
         * Priority in embOS: 4
         */
        NVIC_SetPriority(DMA2_Stream5_IRQn, 6);
        NVIC_EnableIRQ(DMA2_Stream5_IRQn);

        /* Configure USART alternate function */
        v  = GPIOA->AFR[USART_TX_BIT >> 3];
        v &= ~(15UL << ((USART_TX_BIT & 0x7) << 2));
        v |=   (7UL << ((USART_TX_BIT & 0x7) << 2));
        GPIOA->AFR[USART_TX_BIT >> 3] = v;
        v  = GPIOA->AFR[USART_RX_BIT >> 3];
        v &= ~(15UL << ((USART_RX_BIT & 0x7) << 2));
        v |=   (7UL << ((USART_RX_BIT & 0x7) << 2));
        GPIOA->AFR[USART_RX_BIT >> 3] = v;

        /* Configure USART RX/TX pins for alternate function usage */
        v  = GPIOA->MODER;
        v &= ~((3UL << (USART_TX_BIT << 1)) | (3UL << (USART_RX_BIT << 1)));
        v |=  ((2UL << (USART_TX_BIT << 1)) | (2UL << (USART_RX_BIT << 1)));         // PA10: alternate function
        GPIOA->MODER = v;

        /* Initialize USART */
        v = USART_CR1_OVER8     /* oversampline by 8 */
          | 0                   /* USART disable */
          | 0                   /* 1 Start, 8 Data, n Stop bits */
          | 0                   /* Parity control disable */
          | 0                   /* TX empty interrupt disable */
          | 0                   /* TX complete interrupt disable */
          | 0                   /* RX not empty interrupt disable */
          | USART_CR1_TE        /* Transmitter enable */
          | USART_CR1_RE;       /* Receiver enable */
        USART_REG->CR1 = v;

        USART_REG->CR2 = 0;     /* STOP bits: 1 Stop bit */
        
        /* Set baudrate */
        div = (UART_BASECLK << 1) / baudrate;
        div += (div & 0x1) << 1; /* rounding */
        div = (div & 0xFFF0) | ((div >> 1) & 0x7); /* int[15:4], frac[2:0] */
        if (div > 0xFFF7) /* Limit maximum div = 255.125, if div over max */
                div = 0xFFF7;
        else if (!div)    /* Limit div = 0.125, if div is zero */
                div = 0xFFF0 & (div << 4);
        USART_REG->BRR = div;

        /* Enable USART */
        USART_REG->CR1 |= USART_CR1_UE;

        /* Enable RX DMA */
        DMA2_Stream5->CR |= DMA_SxCR_EN;

        /* Clear TC bit in the SR */
        USART_REG->SR &= ~USART_SR_TC;

        /* Select three sample bit method and Enable DMA TX/RX */
        USART_REG->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;

}
#endif
