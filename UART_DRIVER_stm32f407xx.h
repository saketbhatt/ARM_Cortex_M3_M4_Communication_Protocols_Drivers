#ifndef _HAL_UART_DRIVER_H
#define _HAL_UART_DRIVER_H

#include <stdint.h>
#include "stm32f407xx.h"


/////************************************************************************* MACROS *****************************************************************/////

//********************************************************************* STATE *********************************************************************//


typedef enum
{
	
	HAL_UART_STATE_RESET        =0x00,
	HAL_UART_STATE_READY        =0x01,
	HAL_UART_STATE_BUSY         =0x02,
	HAL_UART_STATE_BUSY_TX      =0x12,
	HAL_UART_STATE_BUSY_RX      =0x22,
	HAL_UART_STATE_BUSY_TX_RX   =0x32,
	
}hal_uart_state_t;


//******************************************************************* ERROR ************************************************************************//


#define HAL_UART_ERROR_NONE    ((uint32_t)0x00000000)
#define HAL_UART_ERROR_PE      ((uint32_t)0x00000001)
#define HAL_UART_ERROR_NE      ((uint32_t)0x00000002)
#define HAL_UART_ERROR_FE      ((uint32_t)0x00000004)
#define HAL_UART_ERROR_ORE     ((uint32_t)0x00000008)
#define HAL_UART_ERROR_DMA     ((uint32_t)0x00000010)


//**************************************************************** BASE ADDRESS *********************************************************************//


#define USART_1 USART1
#define USART_2 USART2
#define USART_3 USART3
#define USART_4 UART4
#define USART_5 USART5
#define USART_6 USART6


//************************************************************* CLOCK *****************************************************************************//


#define _HAL_RCC_USART1_CLK_ENABLE ()             ( RCC->APB2ENR |= (1<<4) )
#define _HAL_RCC_USART2_CLK_ENABLE ()             ( RCC->APB1ENR |= (1<<17))
#define _HAL_RCC_USART3_CLK_ENABLE ()             ( RCC->APB1ENR |= (1<<18))
#define _HAL_RCC_UART4_CLK_ENABLE  ()             ( RCC->APB1ENR |= (1<<19))
#define _HAL_RCC_UART5_CLK_ENABLE  ()             ( RCC->APB1ENR |= (1<<20))
#define _HAL_RCC_USART6_CLK_ENABLE ()             ( RCC->APB2ENR |= (1<<5) )


//***************************************************** SR ********************************************************************//


#define USART_REG_SR_TXE_FLAG                     ((uint32_t) (1<<7))
#define USART_REG_SR_TC_FLAG                      ((uint32_t) (1<<6))
#define USART_REG_SR_RXNE_FLAG                    ((uint32_t) (1<<5))
#define USART_REG_SR_IDLE_FLAG                    ((uint32_t) (1<<4))
#define USART_REG_SR_ORE_FLAG                     ((uint32_t) (1<<3))
#define USART_REG_SR_NE_FLAG                      ((uint32_t) (1<<2))
#define USART_REG_SR_FE_FLAG                      ((uint32_t) (1<<1))
#define USART_REG_SR_PE_FLAG                      ((uint32_t) (1<<0))


//**************************************************** BRR ********************************************************************//


#define USART_REG_BRR_MANTISSA                    ((uint32_t) (1<<4))
#define USART_REG_BRR_FRACTION                    ((uint32_t) (1<<0))


//******************************************************* CR1 *****************************************************************//

#define USART_REG_CR1_OVER8                       ((uint32_t) (1<<15)) 
#define USART_OVER8_ENABLE                        1
#define USART_OVER16_ENABLE                       0

#define USART_REG_CR1_USART_EN                    ((uint32_t) (1<<13))

#define USART_REG_CR1_UART_WL                     ((uint32_t) (1<<12))
#define USART_WL_1S8B                             0
#define USART_WL_1S9B                             1

#define USART_REG_CR1_TXE_INT_EN                  ((uint32_t) (1<<7))
#define USART_REG_CR1_TCIE_INT_EN                 ((uint32_t) (1<<6))
#define USART_REG_CR1_RXNE_INT_EN                 ((uint32_t) (1<<5))
#define USART_REG_CR1_PEIE_INT_EN                 ((uint32_t) (1<<8))
#define USART_REG_CR1_TE                          ((uint32_t) (1<<3))
#define USART_REG_CR1_RE                          ((uint32_t) (1<<2))


//********************************************************* CR2 ****************************************************************//


#define USART_REG_CR2_STOP_BITS                                       12
#define USART_STOP_BITS_1                         ((uint32_t)         0)
#define USART_STOP_BITS_HALF                      ((uint32_t)         1)
#define USART_STOP_BITS_2                         ((uint32_t)         2)
#define USART_STOP_BITS_1NHALF                    ((uint32_t)         3)


//********************************************************* CR3 ****************************************************************//


#define USART_REG_CR3_ERR_INT_EN                  ((uint32_t) (1<<0))

#define UART_STOPBITS_1                           ((uint32_t)   0x00)
#define UART_STOPBITS_HALF                        ((uint32_t)   0x01)
#define UART_STOPBITS_2                           ((uint32_t)   0x02)
#define UART_STOPBITS_ONENHALF                    ((uint32_t)   0x03)

#define UART_PARITY_NONE                          ((uint32_t)0x00000000)
#define UART_HWCONTROL_NONE                       ((uint32_t)0x00000000)

#define UART_MODE_TX_RX                           ((uint32_t) (USAR T_REG_CR1_TE | USART_REG_CR1_RE))

#define UART_MODE_TX                              ((uint32_t) (USART_REG_CR1_TE))

#define USART_BAUD_9600                           (uint32_t)9600
#define USART_BAUD_115200                         (uint32_t)115200
#define USART_BAUD_2000000                        (uint32_t)2000000


#define UNUSED(x)                                 ((void)(x))
	




/////******************************************************* DATA STRUCTURES ***************************************************************/////



//******************************************************* INITIALISATION STRUCTURE **********************************************//

typedef struct
{
	
	uint32_t BaudRate;
	uint32_t WordLength;
	uint32_t StopBits;
	uint32_t Parity;
	uint32_t Mode;
	uint32_t OverSampling;
	
}uart_init_t;


//***************************************************** CALLBACK TYPEDEF'S ****************************************************//

typedef void ( TX_COMP_CB_t ) ( void *ptr );
typedef void ( RX_COMP_CB_t ) ( void *ptr );


//******************************************************* HANDLE STRUCTURE *****************************************************//

typedef struct
{
	
	USART_TypeDef          *Instance;
	uart_init_t             Init;
	uint8_t                *pTxBuffPtr;
	uint16_t                TxXferSize;
	uint16_t                TxXferCount;
	uint8_t                *pRxBuffPtr;
  uint16_t                RxXferSize;
  uint16_t                RxXferCount;
  hal_uart_state_t        rx_state;
  hal_uart_state_t        tx_state;
  uint32_t                ErrorCode;
  TX_COMP_CB_t            *tx_cmp_cb;
  RX_COMP_CB_t            *rx_cmp_cb;
  	
}uart_handle_t;






/////*************************************************** API PROTOTYPES *********************************************************/////


void hal_uart_init(uart_handle_t*handle);

void hal_uart_tx(uart_handle_t *handle , uint8_t *buffer , uint32_t len);

void hal_uart_rx(uart_handle_t *handle , uint8_t *buffer , uint32_t len);

/////************************************************** INTERRUPT HANDLER PROTOTYPE ******************************************/////


void hal_uart_handle_interrupt(uart_handle_t *huart);



#endif
