#ifndef _HAL_I2C_DRIVER_H
#define _HAL_I2C_DRIVER_H

#include <stdint.h>
#include "stm32f407xx.h"


/////********************************************************** 1. MACROS *******************************************************************/////

//*********************************************************** CR1 ************************************************************************//

#define I2C_REG_CR1_POS                      ((uint32_t)1<<11)

#define I2C_REG_CR1_ACK                      ((uint32_t)1<<10)
#define I2C_ACK_ENABLE                       1
#define I2C_ACK_DISABLE                      0

#define I2C_REG_CR1_STOP_GEN                 ((uint32_t)1<<9)
#define I2C_REG_CR1_START_GEN                ((uint32_t)1<<8)

#define I2C_REG_CR1_NOSTRETCH                ((uint32_t)1<<7)
#define I2C_ENABLE_CLK_STRETCH               0
#define I2C_DISABLE_CLK_STRETCH              1

#define I2C_REG_CR1_ENABLE_I2C               ((uint32_t)1<<0)


//******************************************************** CR2 ****************************************************************************//

#define I2C_REG_CR2_BUF_INT_ENABLE           ((uint32_t)1<<10)
#define I2C_REG_CR2_EVT_INT_ENABLE           ((uint32_t)1<<9)
#define I2C_REG_CR2_ERR_INT_ENABLE           ((uint32_t)1<<8)

#define I2C_PERIPHERAL_CLK_FREQ_2MHZ         ((uint32_t)2)
#define I2C_PERIPHERAL_CLK_FREQ_3MHZ         ((uint32_t)3)
#define I2C_PERIPHERAL_CLK_FREQ_4MHZ         ((uint32_t)4)
#define I2C_PERIPHERAL_CLK_FREQ_5MHZ         ((uint32_t)5)
#define I2C_PERIPHERAL_CLK_FREQ_6MHZ         ((uint32_t)6)
#define I2C_PERIPHERAL_CLK_FREQ_7MHZ         ((uint32_t)7)
#define I2C_PERIPHERAL_CLK_FREQ_8MHZ         ((uint32_t)8)
#define I2C_PERIPHERAL_CLK_FREQ_9MHZ         ((uint32_t)9)
#define I2C_PERIPHERAL_CLK_FREQ_10MHZ        ((uint32_t)10)
#define I2C_PERIPHERAL_CLK_FREQ_11MHZ        ((uint32_t)11)
#define I2C_PERIPHERAL_CLK_FREQ_12MHZ        ((uint32_t)12)
#define I2C_PERIPHERAL_CLK_FREQ_13MHZ        ((uint32_t)13)
#define I2C_PERIPHERAL_CLK_FREQ_14MHZ        ((uint32_t)14)
#define I2C_PERIPHERAL_CLK_FREQ_15MHZ        ((uint32_t)15)
#define I2C_PERIPHERAL_CLK_FREQ_16MHZ        ((uint32_t)16)

//******************************************************** OAR1 *******************************************************************************//

#define I2C_REG_OAR1_ADDRMODE                 ((uint32_t)1<<15) 
#define I2C_ADDRMODE_7BIT                     0
#define I2C_ADDRMODE_10BIT                    1

#define I2C_REG_OAR1_14TH_BIT                 ((uint32_t)1<<14) 
#define I2C_REG_OAR1_7BIT_ADDRESS_POS         1

//******************************************************** SR1 ******************************************************************************//

#define I2C_REG_SR1_TIMEOUT_FLAG              ((uint32_t)1<<14)
#define I2C_REG_SR1_OVR_FLAG                  ((uint32_t)1<<11)
#define I2C_REG_SR1_AF_FAILURE_FLAG           ((uint32_t)1<<10)
#define I2C_REG_SR1_ARLO_FLAG                 ((uint32_t)1<<9 )
#define I2C_REG_SR1_BUS_ERROR_FLAG            ((uint32_t)1<<8 )
#define I2C_REG_SR1_TXE_FLAG                  ((uint32_t)1<<7 )
#define I2C_REG_SR1_RXNE_FLAG                 ((uint32_t)1<<6 )
#define I2C_REG_SR1_STOP_DETECTION_FLAG       ((uint32_t)1<<4 )            // For Slave
#define I2C_REG_SR1_BTF_FLAG                  ((uint32_t)1<<2 )
#define I2C_REG_SR1_ADDR_FLAG                 ((uint32_t)1<<1 )
#define I2C_REG_SR1_ADDR_SENT_FLAG            ((uint32_t)1<<1 )            // For Master 
#define I2C_REG_SR1_ADDR_MATCHED_FLAG         ((uint32_t)1<<1 )            // For Slave
#define I2C_REG_SR1_SB_FLAG                   ((uint32_t)1<<0 )


//******************************************************* SR2 *********************************************************************************//

#define I2C_REG_SR2_BUS_BUSY_FLAG             ((uint32_t)1<<1)
#define I2C_BUS_IS_BUSY                       1
#define I2C_BUS_IS_FREE                       0

#define I2C_REG_SR2_MSL_FLAG                  ((uint32_t)1<<0)
#define I2C_MASTER_MODE                       1
#define I2C_SLAVE_MODE                        0

#define I2C_REG_SR2_TRA_FLAG                  ((uint32_t)1<<2)
#define I2C_RX_MODE                           0
#define I2C_TX_MODE                           1

//****************************************************** CCR ***********************************************************************************//

#define I2C_REG_CCR_ENABLE_FM                 ((uint32_t)1<<15)
#define I2C_ENABLE_SM                         0
#define I2C_Enable_FM                         1

#define I2C_REG_CCR_DUTY                      ((uint32_t)1<<14)
#define I2C_FM_DUTY_16BY9                     1
#define I2C_FM_DUTY_2                         0

//****************************************************** PERIPHERAL BASE ADDRESSES *************************************************************//

#define I2C_1 I2C1
#define I2C_2 I2C2
#define I2C_3 I2C3

//******************************************************* CLOCK ENABLING MACROS *****************************************************************//

#define _HAL_RCC_I2C1_CLK_ENABLE()            (RCC->APB1ENR |=(1<<21))
#define _HAL_RCC_I2C2_CLK_ENABLE()            (RCC->APB1ENR |=(1<<22))
#define _HAL_RCC_I2C3_CLK_ENABLE()            (RCC->APB1ENR |=(1<<23))


//********************************************************** OTHERS ****************************************************************************//

// ERROR POSSIBILITIES //
#define HAL_I2C_ERROR_NONE        ((uint32_t)0x00000000)
#define HAL_I2C_ERROR_BERR        ((uint32_t)0x00000001)
#define HAL_I2C_ERROR_ARLO        ((uint32_t)0x00000002)
#define HAL_I2C_ERROR_AF          ((uint32_t)0x00000004)
#define HAL_I2C_ERROR_OVR         ((uint32_t)0x00000008)
#define HAL_I2C_ERROR_DMA         ((uint32_t)0x00000010)
#define HAL_I2C_ERROR_TIMEOUT     ((uint32_t)0x00000020)

// SET RESET //
#define RESET           0;
#define SET             !RESET;



/////***************************************************** 2. DATA STRUCTURES *******************************************************************/////

//***************************************************** STATE ENUM *****************************************************************************//

typedef enum
{
	
	HAL_I2C_STATE_RESET             = 0x00,
  HAL_I2C_STATE_READY             = 0x01,
  HAL_I2C_STATE_BUSY              = 0x02,
  HAL_I2C_STATE_BUSY_TX           = 0x03,
  HAL_I2C_STATE_BUSY_RX           = 0x04,
  HAL_I2C_STATE_ERROR             = 0x05,	

}hal_i2c_state_t;


 
//************************************************* CONFIGURATION STRUCTURE ********************************************************************//

typedef struct
{

	uint32_t ClockSpeed;
	uint32_t DutyCycle;
	uint32_t OwnAddress1;
	uint32_t AddressingMode;
	uint32_t DualAddressMode;
	uint32_t OwnAddress2;
	uint32_t GeneralCallMode;
	uint32_t NoStretchMode;
	uint32_t ack_enable;
	
}i2c_init_t;


//**************************************************** HANDLE STRUCTURE ************************************************************************//

typedef struct
{
	
	I2C_TypeDef          *Instance;
	i2c_init_t            Init;
	uint8_t              *pBuffPtr;
	uint32_t              XferSize;
	uint32_t              XferCount;
	hal_i2c_state_t       State;
	uint32_t              ErrorCode;

}i2c_handle_t;

/////************************************************* 3. API PROTOTYPES *************************************************************************/////


// INITIALISATION //
void hal_i2c_init(i2c_handle_t *handle);

// MASTER TRANSMISSION //
void hal_i2c_master_tx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer,uint32_t len);

// MASTER RECEPTION //
void hal_i2c_master_rx(i2c_handle_t *handle, uint8_t slave_adr, uint8_t *buffer,uint32_t len);

// SLAVE TRANSMISSION //
void hal_i2c_slave_tx(i2c_handle_t *handle, uint8_t *buffer,uint32_t len);

// SLAVE RECEPTION //
void hal_i2c_slave_rx(i2c_handle_t *handle, uint8_t *buffer,uint32_t len);

// ERROR INTERRUPT HANDLER //
void hal_i2c_handle_error_interrupt(i2c_handle_t *hi2c);

// EVENT INTERRUPT HANDLER //
void hal_i2c_handle_evt_interrupt(i2c_handle_t *hi2c);


#endif
