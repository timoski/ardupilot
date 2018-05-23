/*
(c) 2017 night_ghost@ykoctpa.ru
 
based on: LeafLabs

*/

#pragma GCC optimize ("O2")

#include <gpio_hal.h>


/*
 * GPIO devices
 */

const gpio_dev gpioa = {
    .regs     = GPIOA,
    .clk       = RCC_AHB1Periph_GPIOA,
    .exti_port = AFIO_EXTI_PA,
};
/** GPIO port A device. */
const gpio_dev* const _GPIOA = &gpioa;

const gpio_dev gpiob = {
    .regs      = GPIOB,
    .clk       = RCC_AHB1Periph_GPIOB,
    .exti_port = AFIO_EXTI_PB,
};
/** GPIO port B device. */
const gpio_dev* const _GPIOB = &gpiob;

const gpio_dev gpioc = {
    .regs      = GPIOC,
    .clk       = RCC_AHB1Periph_GPIOC,
    .exti_port = AFIO_EXTI_PC,
};
/** GPIO port C device. */
const gpio_dev* const _GPIOC = &gpioc;

const gpio_dev gpiod = {
    .regs      = GPIOD,
    .clk       = RCC_AHB1Periph_GPIOD,
    .exti_port = AFIO_EXTI_PD,
};
/** GPIO port D device. */
const gpio_dev* const _GPIOD = &gpiod;

const gpio_dev gpioe = {
    .regs      = GPIOE,
    .clk       = RCC_AHB1Periph_GPIOE,
    .exti_port = AFIO_EXTI_PE,
};
/** GPIO port E device. */
const gpio_dev* const _GPIOE = &gpioe;

const gpio_dev gpiof = {
    .regs      = GPIOF,
    .clk       = RCC_AHB1Periph_GPIOF,
    .exti_port = AFIO_EXTI_PF,
};
/** GPIO port F device. */
const gpio_dev* const _GPIOF = &gpiof;

const gpio_dev gpiog = {
    .regs      = GPIOG,
    .clk       = RCC_AHB1Periph_GPIOG,
    .exti_port = AFIO_EXTI_PG,
};
/** GPIO port G device. */
const gpio_dev* const _GPIOG = &gpiog;



static const gpio_dev* _gpios[] =  { &gpioa, &gpiob, &gpioc, &gpiod, &gpioe, &gpiof, &gpiog };


void gpio_init_all(void)
{
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOA, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOB, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOC, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOD, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOE, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOF, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOG, DISABLE);

    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
}



void gpio_set_mode(const gpio_dev* const dev, uint8_t pin, gpio_pin_mode pin_mode)
{

    /* Enable the GPIO Clock  */
    RCC_enableAHB1_clk(dev->clk);

    uint32_t mode, pull, type;
  
    /* Configure the pin */
    uint32_t Speed = GPIO_speed_2MHz; // low noise by default
    
	
    switch(pin_mode) {
    case GPIO_OUTPUT_PP:
	mode = GPIO_Mode_OUT;
	pull = GPIO_PuPd_NOPULL;
	type = GPIO_OType_PP;
	break;
    case GPIO_OUTPUT_OD:
	mode = GPIO_Mode_OUT;
	pull = GPIO_PuPd_NOPULL;
	type = GPIO_OType_OD;
	break;
    case GPIO_OUTPUT_OD_PU:
	mode = GPIO_Mode_OUT;
	pull = GPIO_PuPd_UP;
	type = GPIO_OType_OD;
	break;
    case GPIO_INPUT_FLOATING:
	mode = GPIO_Mode_IN;
	pull = GPIO_PuPd_NOPULL;
	type = GPIO_OType_PP;
	break;
    case GPIO_INPUT_ANALOG:
	mode = GPIO_Mode_AN;
	pull = GPIO_PuPd_NOPULL;
	type = GPIO_OType_PP;
	break;
    case GPIO_INPUT_PU:
	mode = GPIO_Mode_IN;
	pull = GPIO_PuPd_UP;
	type = GPIO_OType_PP;
	break;
    case GPIO_INPUT_PD:
	mode = GPIO_Mode_IN;
	pull = GPIO_PuPd_DOWN;
	type = GPIO_OType_PP;
	break;
    case GPIO_AF_OUTPUT_PP:
	mode = GPIO_Mode_AF;
	pull = GPIO_PuPd_NOPULL;
	type = GPIO_OType_PP;
	break;
    case GPIO_AF_OUTPUT_OD:
	mode = GPIO_Mode_AF;
	pull = GPIO_PuPd_NOPULL;
	type = GPIO_OType_OD;
	break;
    case GPIO_AF_OUTPUT_OD_PU:
	mode = GPIO_Mode_AF;
	pull = GPIO_PuPd_UP;
	type = GPIO_OType_OD;
	break;
    default:
	return;
    }

    dev->regs->MODER  &= ~(GPIO_MODER_MODER0 << pin * 2);
    dev->regs->MODER |= (uint32_t)mode << pin * 2;

    if ((mode == GPIO_Mode_OUT) || (mode == GPIO_Mode_AF)) {
        // Speed 
        dev->regs->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << pin * 2);
        dev->regs->OSPEEDR |= (uint32_t)Speed << pin * 2;

        // Output mode
        dev->regs->OTYPER  &= ~(GPIO_OTYPER_OT_0 << pin) ;
        dev->regs->OTYPER |= (uint16_t)((uint16_t)type << pin);
    }
      
    // Pull-up/Pull down 
    dev->regs->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << pin * 2);
    dev->regs->PUPDR |= (uint32_t)pull << pin * 2;
}
