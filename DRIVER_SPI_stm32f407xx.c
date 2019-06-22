#include <stdint.h>
#include "hal_spi_driver.h"                                                  // include the driver header file //


//*************************** HELPER FUNCTIONS *************************************************************//
// to accomplish the work in the spi init function develop these helper functions which do individual tasks , private static functions //



static void hal_spi_configure_device_mode(SPI_TypeDef *SPIx,uint32_t master)
	
{
    if(master)
		{
			SPIx->CR1 |= SPI_REG_CR1_MSTR;                                         // setting of master/slave is by CR1 register //
		}
		else
		{
			SPIx->CR1 &= ~SPI_REG_CR1_MSTR;
		}

}





static void hal_spi_configure_phase_and_polarity(SPI_TypeDef *SPIx,uint32_t phase_value,uint32_t polarity)
	
{
	if(phase_value)
	{
		SPIx->CR1 |= SPI_REG_CR1_CPHA;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_CPHA;
	}
	if(polarity)
	{
		SPIx->CR1 |= SPI_REG_CR1_CPOL;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_CPOL;
	}
}




static void hal_spi_configure_datasize_dierection(SPI_TypeDef*SPIx,uint32_t datasize_16,uint32_t lsbfirst)
{
	
	if(datasize_16)                                                            // enabling 16 bit data transmission , default: 8 bit data transmission //
	{
		SPIx->CR1 |= SPI_REG_CR1_DFF;
	}
	else
  {
		SPIx->CR1 &= ~SPI_REG_CR1_DFF;
	}
	
	if(lsbfirst)                                                               // enabling lsbfirst transmission , default: msbfirst data transmission //
	{
		SPIx->CR1 |= SPI_CR1_LSBFIRST;
	}
	else
  {
		SPIx->CR1 &= ~SPI_CR1_LSBFIRST;
	}
	
}





static void hal_spi_configure_nss_master(SPI_TypeDef *SPIx,uint32_t ssm_enable)
{
	if(ssm_enable)
	{
		
		SPIx->CR1 |= SPI_REG_CR1_SSM;
		SPIx->CR1 |= SPI_REG_CR1_SSI;
	}
	else
  {
		SPIx->CR1 &= ~SPI_REG_CR1_SSM;
	}
}





static void hal_spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm_enable)
	
{
	if(ssm_enable)
	{
		
		SPIx->CR1 |= SPI_REG_CR1_SSM;                                            // making software management//
	}
	else
  {
		SPIx->CR1 &= ~SPI_REG_CR1_SSM;                                           // making hardware management//
	}
	
}





static void hal_spi_enable(SPI_TypeDef *SPIx)
{
	if(!(SPIx->CR1 & SPI_REG_CR1_SPE))
	{
		SPIx->CR1 |= SPI_REG_CR1_SPE;
		
	}
}




static void hal_spi_disable(SPI_TypeDef *SPIx)
{
		SPIx->CR1 &= ~SPI_REG_CR1_SPE;
}

 
// FOR TXE INTERRUPT //


static void hal_spi_enable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= SPI_REG_CR2_TXEIE_ENABLE;
}

static void hal_spi_disable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_TXEIE_ENABLE;
}

// FOR RXNE INTERRUPT //


static void hal_spi_enable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= SPI_REG_CR2_RXNEIE_ENABLE;
}

static void hal_spi_disable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_RXNEIE_ENABLE;
}





static void hal_spi_tx_close_interrupt(spi_handle_t*hspi)
{
	
	hal_spi_disable_txe_interrupt(hspi->Instance);                             // Disable TXE interrupt //
	if(hspi->Init.Mode && (hspi->State != HAL_SPI_STATE_BUSY_RX))              // change the state only if in master and not recieving //
		hspi->State = HAL_SPI_STATE_READY;
}



static void hal_spi_close_rx_interrupt(spi_handle_t *hspi)
{
	
	while(hal_spi_is_bus_busy(hspi->Instance));
	
	hal_spi_disable_rxne_interrupt(hspi->Instance);                            // Disable RXNE interrupt //
	hspi->State = HAL_SPI_STATE_READY;                                         // Make state as READY //

}





//********************************************************** NON-BLOCKING MASTER/SLAVE TX/RX APIs *************************************************************//


//**NOTE:** master TX code similar to slave RX code ***** master RX code similar to slave TX code ********//



void hal_spi_master_tx(spi_handle_t * spi_handle, uint8_t *buffer, uint32_t len)
{
	
	spi_handle->pTxBuffPtr = buffer;                                           // saving the passed global buffer pointer to the required variable in spi_handle structure//
	spi_handle->TxXferCount = len;                                             // saving the passed TX count to the required variable in spi_handle structure//
	spi_handle->TxXferSize = len;                                              // saving the passed TX size/length to the required variable in spi_handle structure//
	
	spi_handle->State = HAL_SPI_STATE_BUSY_TX;                                 // set the state as BUSY while TX //
	
	hal_spi_enable(spi_handle->Instance);                                      // enable the SPi peripheral if not enabled //
	 
	hal_spi_enable_txe_interrupt(spi_handle->Instance);                        // enable the TX buffer empty (TXE) interrupt // 
	                                                                           // after this interrupt handler will take care the rest and actual TX ing //

}



void hal_spi_master_rx(spi_handle_t * spi_handle, uint8_t *rx_buffer, uint32_t len)
{
	
	uint32_t i=0,val;
	
	spi_handle->pTxBuffPtr = rx_buffer;                                        // dummy tx for clock , so u can use rx_buffer pointer anyways //
	spi_handle->TxXferCount = len; 
	spi_handle->TxXferSize = len;
	
	spi_handle->pRxBuffPtr = rx_buffer;                                        // data rx ing in RX buffer//
	spi_handle->RxXferCount = len; 
	spi_handle->RxXferSize = len;
	
  spi_handle->State = HAL_SPI_STATE_BUSY_RX;                                 // set the state as BUSY while RX //
	
	hal_spi_enable(spi_handle->Instance); 
	
	val=spi_handle->Instance->DR;   	                                         // read Data register once before enabling rxne interreupt ,used further in the IH code //
	

  hal_spi_enable_rxne_interrupt(spi_handle->Instance);                       //enable txe and rxne interrupts //
	hal_spi_enable_txe_interrupt(spi_handle->Instance);
	
	
}


void hal_spi_slave_tx(spi_handle_t * spi_handle ,uint8_t *tx_buffer, uint32_t len)
{
	
	spi_handle->pTxBuffPtr = tx_buffer;                   
	spi_handle->TxXferCount = len; 
	spi_handle->TxXferSize = len;
	
	spi_handle->pRxBuffPtr = tx_buffer;                                        // dummy rx , u can use tx_buffer pointer  //
	spi_handle->RxXferCount = len; 
	spi_handle->RxXferSize = len;
	
	spi_handle->State = HAL_SPI_STATE_BUSY_TX;	
	hal_spi_enable(spi_handle->Instance); 
	
  
	hal_spi_enable_rxne_interrupt(spi_handle->Instance);                       //enable txe and rxne interrupts //
	hal_spi_enable_txe_interrupt(spi_handle->Instance);
	
}

void hal_spi_slave_rx(spi_handle_t * spi_handle, uint8_t *rcv_buffer, uint32_t len)
{
	
	spi_handle->pRxBuffPtr = rcv_buffer;               
	spi_handle->RxXferCount = len;                   
	spi_handle->RxXferSize = len;                    
	
	spi_handle->State = HAL_SPI_STATE_BUSY_RX;       
	
	hal_spi_enable(spi_handle->Instance);            
	 
	hal_spi_enable_rxne_interrupt(spi_handle->Instance);    

}



//******************************** IRQ HANDLERS ***********************************************************************************//



void hal_spi_handle_tx_interrupt(spi_handle_t*hspi)
{
	
	if(hspi->Init.DataSize == SPI_8BIT_DF_ENABLE)
	{
		hspi->Instance->DR = (*hspi->pTxBuffPtr++);
		hspi->TxXferCount--;
	}
	else
	{
		hspi->Instance->DR = *((uint16_t*)*hspi->pTxBuffPtr);
		hspi->pTxBuffPtr+=2;
		hspi->TxXferCount-=2;
	}
	if(hspi->TxXferCount == 0)
	{
		hal_spi_tx_close_interrupt(hspi);
	}
}







void hal_spi_handle_rx_interrupt(spi_handle_t*hspi)
{
	if(hspi->Init.DataSize == SPI_8BIT_DF_ENABLE)
	{
		if(hspi->pRxBuffPtr++)
			(*hspi->pRxBuffPtr++) = hspi->Instance->DR;
		 hspi->RxXferCount--;
	}
	else
	{
		 *((uint16_t*)*hspi->pRxBuffPtr) = hspi->Instance->DR;
		hspi->pRxBuffPtr+=2;
		hspi->RxXferCount-=2;
	}
	
	if(hspi->RxXferCount == 0)
	{
		hal_spi_close_rx_interrupt(hspi);
	}
}








void hal_spi_irq_handler(spi_handle_t * hspi)
{
	uint32_t tm1 = 0,tm2 = 0;
	
	tm1=(hspi->Instance->SR & SPI_REG_SR_RXNE_FLAG);	                         // check if rxne is set in SR, 0 th bit in SR holds about rxne event //
	tm2=(hspi->Instance->CR2 & SPI_REG_CR2_RXNEIE_ENABLE);                     // check if RXNEIE bit is enabled in the CR2 //
	
	if((tm1!=RESET) && (tm2!=RESET))
	{
		hal_spi_handle_rx_interrupt(hspi);
		return;
	}
	
	tm1=(hspi->Instance->SR & SPI_REG_SR_TXE_FLAG);	                           // check if txeis set in SR, 1 th bit in SR holds about txe event //
	tm2=(hspi->Instance->CR2 & SPI_REG_CR2_TXEIE_ENABLE);                      // check if TXEIE bit is enabled in the CR2 //
	
	if((tm1!=RESET) && (tm2!=RESET))
	{
		hal_spi_handle_tx_interrupt(hspi);
		return;
	}
	
}

