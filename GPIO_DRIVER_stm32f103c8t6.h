#ifndef DRIVER_HAL_GPIO
#define DRIVER_HAL_GPIO

#include "stm32f10x.h"                                       // board specific header file //

#define LOW    0
#define HIGH   1

//************************************************** 1.MACROS ***************************************************************//

//port names

#define PORTA GPIOA
#define PORTB GPIOB
#define PORTC GPIOC
#define PORTD GPIOD
#define PORTE GPIOE
#define PORTF GPIOF
#define PORTG GPIOG

//pin mode

#define OUTPUT_MODE ((uint32_t)0x01)                         //numbers dont matter since we gonna just put them into if statements ,but they should be different.
#define INPUT_MODE  ((uint32_t)0x02)

//input mode types

#define INPUT_ANALOG    ((uint32_t)0x00)                     //numbers matter
#define INPUT_FLOATING  ((uint32_t)0x01)
#define INPUT_PU_PD     ((uint32_t)0x02)


//output mode types

#define OUTPUT_GEN_PURPOSE           ((uint32_t)0x00)
#define OUTPUT_OD                    ((uint32_t)0x01)
#define OUTPUT_ALT_FUNCTION          ((uint32_t)0x02)
#define OUTPUT_ALT_FUNCTION_OD       ((uint32_t)0x03)



//pin speed

#define SPEED_2MHZ                    ((uint32_t)0x02)       //numbers matter
#define SPEED_10MHZ                   ((uint32_t)0x01)
#define SPEED_50MHZ                   ((uint32_t)0x03)

//clock enabling

#define GPIO_CLOCK_ENABLE_ALT_FUNC    (RCC->APB2ENR |= (1<<0))
#define GPIO_CLOCK_ENABLE_PORTA       (RCC->APB2ENR |= (1<<2))
#define GPIO_CLOCK_ENABLE_PORTB       (RCC->APB2ENR |= (1<<3))
#define GPIO_CLOCK_ENABLE_PORTC       (RCC->APB2ENR |= (1<<4))
#define GPIO_CLOCK_ENABLE_PORTD       (RCC->APB2ENR |= (1<<5))


//bit positions for CNFY part of CNF register

#define CNF_POS_BIT1   (PINPOS[pinNumber]+2)
#define CNF_POS_BIT2   (PINPOS[pinNumber]+3)



//*************************************************** 2.DATA STRUCTURES ***********************************************//



//configuration structure

typedef struct
{
	GPIO_TypeDef *port;
	
	uint32_t pin;
	
	uint32_t mode;
	
  uint32_t mode_type;
	
	uint32_t pull;
	
	uint32_t speed;
	
	uint32_t alt_func;
	
}GPIO_TYPE;


//enum structure

typedef enum
	{
  
		RISING_EDGE,
		FALLING_EDGE,
		RISING_FALLING_EDGE

	}edge_select;
 





//*************************************** 3.EXPOSED API PROTOTYPES *************************************************************//
	
//                                  GPIO CONFIGURATION

static void config_pin (GPIO_TypeDef*port , uint32_t pinNumber , uint32_t mode_type);

static void config_pin_speed (GPIO_TypeDef*port , uint32_t pinNumber ,uint32_t pinspeed, uint32_t mode);



//                                  GPIO user pin functions 

void gpio_write(GPIO_TypeDef *port,uint32_t pinNumber , uint8_t state);

void gpio_toggle(GPIO_TypeDef *port,uint32_t pinNumber);

void gpio_init(GPIO_TYPE gpio_type);                         //passing GPIO_TYPE structure here.



//                                        Interrupt Functions 
															
void confiigure_gpio_interrupt( GPIO_TypeDef *port, uint32_t pinNumber, edge_select edge );

void enable_gpio_interrupt(uint32_t pinNumber,IRQn_Type irqNumber);

void clear_gpio_interrupt (uint32_t pinNumber);

#endif

