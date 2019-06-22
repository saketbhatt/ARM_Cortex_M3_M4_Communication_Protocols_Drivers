#include "hal_uart_driver.h"
#include "stdint.h"
#include "stm32f4xx.h"

/////***************************************************************** HELPER FUNCTIONS **********************************************************/////
void hal_uart_enable(USART_TypeDef *uartx)
{
	
	uartx->CR1 |= USART_REG_CR1_USART_EN;

}



void hal_uart_disable(USART_TypeDef *uartx)
{
	
	uartx->CR1 &= ~(USART_REG_CR1_USART_EN); 

}



void hal_uart_enable_disable_tx(USART_TypeDef * uartx,uint32_t te)
{
	if(te & USART_REG_CR1_TE)
	{
		uartx->CR1 |= USART_REG_CR1_TE;
	}
	
	else
	{
		uartx->CR1 &= ~(USART_REG_CR1_TE);
	}
}



void hal_uart_enable_disable_rx(USART_TypeDef * uartx,uint32_t re)
{
	if(re & USART_REG_CR1_RE)
	{
		uartx->CR1 |= USART_REG_CR1_RE;
	}
	
	else
	{
		uartx->CR1 &= ~(USART_REG_CR1_RE);
	}
}



void hal_uart_configure_word_length(USART_TypeDef * uartx, uint32_t wl)
{
	if(wl)
	{
		uartx->CR1 |=USART_REG_CR1_UART_WL;        // 9 bit word length
	}
	else
  {
		uartx->CR1 &= ~(USART_REG_CR1_UART_WL);    // 8 bit word length
	}
	
}




void hal_uart_configure_stop_bits ( USART_TypeDef *uartx , uint32_t nstop )
{
	
	uartx->CR2 &= ~(0x3<<USART_REG_CR2_STOP_BITS);
	
	if (nstop==USART_STOP_BITS_HALF)
	{
		uartx->CR2 |= (0x01<<USART_REG_CR2_STOP_BITS);
	}
	else if(nstop == USART_STOP_BITS_2)
	{
		uartx->CR2 |= (0x02<< USART_REG_CR2_STOP_BITS);

	}
	else if(nstop == USART_STOP_BITS_1NHALF)
	{
		uartx->CR2 |= (0x03<< USART_REG_CR2_STOP_BITS);

	}
	else
	{
		uartx->CR2 |= (0x00<< USART_REG_CR2_STOP_BITS);

	}

}


void hal_uart_set_baud_rate(USART_TypeDef *uartx,uint32_t baud)
{
	uint32_t val;
	
	if(baud==USART_BAUD_9600)
	{
		val=0x683;
		
	}
	else if(baud==USART_BAUD_115200)
	{
		val=0x8A;
	}
	else
  {
		val=0x8A; 
	
	}
	uartx->BRR = val;
	
}


void hal_uart_configure_over_sampling(USART_TypeDef*uartx,uint32_t over8)

{
	if(over8)
	{
	  uartx->CR1 |= USART_REG_CR1_OVER8;         // Default is 16     
	}
	
}



void hal_uart_configure_txe_interrupt(USART_TypeDef *uartx, uint32_t txe_en)
{
	
	if(txe_en)
	  uartx->CR1 |= USART_REG_CR1_TXE_INT_EN;
	else
		uartx->CR1 &= ~(USART_REG_CR1_TXE_INT_EN);

}



void hal_uart_configure_rxne_interrupt(USART_TypeDef *uartx, uint32_t rxne_en)
{
	
	if(rxne_en)
	  uartx->CR1 |= USART_REG_CR1_RXNE_INT_EN;
	else
		uartx->CR1 &= ~(USART_REG_CR1_RXNE_INT_EN);

}



void hal_uart_configure_error_interrupt(USART_TypeDef *uartx, uint32_t er_en)
{
	
	if(er_en)
	  uartx->CR3 |= USART_REG_CR3_ERR_INT_EN;
	else
		uartx->CR3 &= ~(USART_REG_CR3_ERR_INT_EN);

}




void hal_uart_configure_parity_error_interrupt(USART_TypeDef *uartx, uint32_t pe_en)
{
	
	if(pe_en)
	  uartx->CR1 |= USART_REG_CR1_PEIE_INT_EN;
	else
		uartx->CR1 &= ~(USART_REG_CR1_PEIE_INT_EN);

}



void hal_uart_clear_error_flag(uart_handle_t*huart)
{
	uint32_t tmpreg = 0x00;
	tmpreg = huart->Instance->SR;
	tmpreg = huart->Instance->DR;
}



static void hal_uart_handle_TXE_interrupt(uart_handle_t *huart)
{
	uint32_t tmp1 = 0;
	uint8_t val;
	
	tmp1 = huart->tx_state;
	if(tmp1 == HAL_UART_STATE_BUSY_TX)
	{
		val = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF );
		huart->Instance->DR = val;
		
		if(--huart->TxXferCount==0)
		{
			huart->Instance->CR1 &= ~(USART_REG_CR1_TXE_INT_EN);
			huart->Instance->CR1 |= USART_REG_CR1_TCIE_INT_EN;
		}
	}
}



static void hal_uart_handle_TC_interrupt(uart_handle_t*huart)
{
	
	huart->Instance->CR1 &= ~(USART_REG_CR1_TCIE_INT_EN);
	huart->tx_state = HAL_UART_STATE_READY;

}



static void hal_uart_handle_RXNE_interrupt(uart_handle_t*huart)
{
	uint32_t tmp1 =0;
	tmp1 =huart->rx_state;
	
	if(tmp1 == HAL_UART_STATE_BUSY_RX)
	{
		if(huart->Init.Parity == UART_PARITY_NONE)
		{
			*huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
		
		}
		else
		{
			*huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x007F);
		}
		
		if(--huart->RxXferCount ==0)
		{
			huart->Instance->CR1 &= ~(USART_REG_CR1_RXNE_INT_EN);
			
			huart->Instance->CR1 &= ~(USART_REG_CR1_PEIE_INT_EN);
			
			huart->Instance->CR3 &= ~(USART_REG_CR3_ERR_INT_EN);
			
			huart->rx_state = HAL_UART_STATE_READY;
		}
	}
}






/////******************************************************** API DEFINITIONS ****************************************************************/////



void hal_uart_init(uart_handle_t *uart_handle)
{
	
	hal_uart_configure_word_length(uart_handle->Instance,uart_handle->Init.WordLength);
	
	hal_uart_configure_stop_bits(uart_handle->Instance,uart_handle->Init.StopBits);
	
	hal_uart_configure_over_sampling(uart_handle->Instance,uart_handle->Init.OverSampling);
	
	hal_uart_set_baud_rate(uart_handle->Instance,uart_handle->Init.BaudRate);
	
	hal_uart_enable_disable_tx(uart_handle->Instance,uart_handle->Init.Mode);
	
	hal_uart_enable_disable_rx(uart_handle->Instance,uart_handle->Init.Mode);
	
	hal_uart_enable(uart_handle->Instance);
	
	uart_handle->tx_state = HAL_UART_STATE_READY;
	
	uart_handle->rx_state = HAL_UART_STATE_READY;
	
	uart_handle->ErrorCode = HAL_UART_ERROR_NONE;
	
}




void hal_uart_tx(uart_handle_t *uart_handle,uint8_t *buffer,uint32_t len)
{
	
	uart_handle->pTxBuffPtr = buffer;
	uart_handle->TxXferCount = len;
	uart_handle->TxXferSize = len;
	
	uart_handle->tx_state = HAL_UART_STATE_BUSY_TX;
	
	hal_uart_enable(uart_handle->Instance);
	
	hal_uart_configure_txe_interrupt(uart_handle->Instance,1);
	
}



void hal_uart_rx(uart_handle_t *uart_handle,uint8_t *buffer,uint32_t len)
{
	uint32_t val;
	
	uart_handle->pRxBuffPtr = buffer;
	uart_handle->RxXferCount = len;
	uart_handle->RxXferSize = len;
	
	uart_handle->rx_state = HAL_UART_STATE_BUSY_RX;
	
	hal_uart_configure_parity_error_interrupt(uart_handle->Instance,1);
	
	hal_uart_configure_error_interrupt(uart_handle->Instance,1);
	
	val = uart_handle->Instance->DR;
	
	hal_uart_enable(uart_handle->Instance);
	
	hal_uart_configure_rxne_interrupt(uart_handle->Instance,1);
	
}




/////********************************************************** INTERRUPT HANDLER DEFINITION ****************************************************/////



void hal_uart_handle_interrupt(uart_handle_t *huart)
{
	uint32_t tmp1=0,tmp2=0;
  
  tmp1 = huart->Instance->SR & USART_REG_SR_PE_FLAG;
	tmp2 = huart->Instance->CR1 & USART_REG_CR1_PEIE_INT_EN;
	
	if(tmp1 && tmp2)                                                        // Parity Error
	{
		hal_uart_clear_error_flag(huart);
		huart->ErrorCode |= HAL_UART_ERROR_PE;
	}
	
	tmp1 = huart->Instance->SR & USART_REG_SR_FE_FLAG;
	tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERR_INT_EN;
	
	if(tmp1 && tmp2)                                                        // Frame Error
	{
		hal_uart_clear_error_flag(huart);
		huart->ErrorCode |= HAL_UART_ERROR_FE;
	}

	tmp1 = huart->Instance->SR & USART_REG_SR_NE_FLAG;
	tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERR_INT_EN;
	
	if(tmp1 && tmp2)                                                        // Noise Error
	{
		hal_uart_clear_error_flag(huart);
		huart->ErrorCode |= HAL_UART_ERROR_NE;
	}

	tmp1 = huart->Instance->SR & USART_REG_SR_ORE_FLAG;
	tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERR_INT_EN;
	
	if(tmp1 && tmp2)                                                        // Overrun Error
	{
		hal_uart_clear_error_flag(huart);
		huart->ErrorCode |= HAL_UART_ERROR_ORE;
	}

	tmp1 = huart->Instance->SR & USART_REG_SR_RXNE_FLAG;
	tmp2 = huart->Instance->CR1 & USART_REG_CR1_RXNE_INT_EN;
	
	if(tmp1 && tmp2)                                                        // RXNE 
	{
	  hal_uart_handle_RXNE_interrupt(huart);
	}

	tmp1 = huart->Instance->SR & USART_REG_SR_TXE_FLAG;
	tmp2 = huart->Instance->CR1 & USART_REG_CR1_TXE_INT_EN;
	
	if(tmp1 && tmp2)                                                        // TXE
	{
		hal_uart_handle_TXE_interrupt(huart);
	}

	tmp1 = huart->Instance->SR & USART_REG_SR_TC_FLAG;
	tmp2 = huart->Instance->CR1 & USART_REG_CR1_TCIE_INT_EN;
	
	if(tmp1 && tmp2)                                                        // TX Complete
	{
		hal_uart_handle_TC_interrupt(huart);
	}

	if(huart->ErrorCode != HAL_UART_ERROR_NONE)
	{
		huart->tx_state = HAL_UART_STATE_READY;
		huart->rx_state = HAL_UART_STATE_READY;
	}		
	
	
}
