/*
(c) 2017 night_ghost@ykoctpa.ru
 
based on: LeafLabs

*/

#pragma GCC optimize ("O2")

#include <gpio_hal.h>
#include "rcc.h"


/*
 * GPIO devices
 */

const gpio_dev gpioa = {
    .GPIOx     = GPIOA,
    .clk       = RCC_AHB1Periph_GPIOA,
    .exti_port = AFIO_EXTI_PA,
};
/** GPIO port A device. */
const gpio_dev* const _GPIOA = &gpioa;

const gpio_dev gpiob = {
    .GPIOx      = GPIOB,
    .clk       = RCC_AHB1Periph_GPIOB,
    .exti_port = AFIO_EXTI_PB,
};
/** GPIO port B device. */
const gpio_dev* const _GPIOB = &gpiob;

const gpio_dev gpioc = {
    .GPIOx      = GPIOC,
    .clk       = RCC_AHB1Periph_GPIOC,
    .exti_port = AFIO_EXTI_PC,
};
/** GPIO port C device. */
const gpio_dev* const _GPIOC = &gpioc;

const gpio_dev gpiod = {
    .GPIOx      = GPIOD,
    .clk       = RCC_AHB1Periph_GPIOD,
    .exti_port = AFIO_EXTI_PD,
};
/** GPIO port D device. */
const gpio_dev* const _GPIOD = &gpiod;

const gpio_dev gpioe = {
    .GPIOx      = GPIOE,
    .clk       = RCC_AHB1Periph_GPIOE,
    .exti_port = AFIO_EXTI_PE,
};
/** GPIO port E device. */
const gpio_dev* const _GPIOE = &gpioe;

const gpio_dev gpiof = {
    .GPIOx      = GPIOF,
    .clk       = RCC_AHB1Periph_GPIOF,
    .exti_port = AFIO_EXTI_PF,
};
/** GPIO port F device. */
const gpio_dev* const _GPIOF = &gpiof;

const gpio_dev gpiog = {
    .GPIOx      = GPIOG,
    .clk       = RCC_AHB1Periph_GPIOG,
    .exti_port = AFIO_EXTI_PG,
};
/** GPIO port G device. */
const gpio_dev* const _GPIOG = &gpiog;



static const gpio_dev* _gpios[] =  { &gpioa, &gpiob, &gpioc, &gpiod, &gpioe, &gpiof, &gpiog };


void gpio_init_all(void)
{
    RCC_doAHB1_reset(RCC_AHB1Periph_GPIOA);
    RCC_doAHB1_reset(RCC_AHB1Periph_GPIOB);
    RCC_doAHB1_reset(RCC_AHB1Periph_GPIOC);
    RCC_doAHB1_reset(RCC_AHB1Periph_GPIOD);
    RCC_doAHB1_reset(RCC_AHB1Periph_GPIOE);
    RCC_doAHB1_reset(RCC_AHB1Periph_GPIOF);
    RCC_doAHB1_reset(RCC_AHB1Periph_GPIOG);
}



void gpio_set_mode(const gpio_dev* const dev, uint8_t pin, gpio_pin_mode mode)
{
    GPIO_Init_t config;

    /* Enable the GPIO Clock  */
    RCC_enableAHB1_clk(dev->clk);
  
    /* Configure the pin */
    config.GPIO_Speed = GPIO_speed_2MHz; // low noise by default
	
    switch(mode) {
    case GPIO_OUTPUT_PP:
	config.GPIO_Mode = GPIO_Mode_OUT;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_OUTPUT_OD:
	config.GPIO_Mode = GPIO_Mode_OUT;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_OD;
	break;
    case GPIO_OUTPUT_OD_PU:
	config.GPIO_Mode = GPIO_Mode_OUT;
	config.GPIO_PuPd = GPIO_PuPd_UP;
	config.GPIO_OType = GPIO_OType_OD;
	break;
    case GPIO_INPUT_FLOATING:
	config.GPIO_Mode = GPIO_Mode_IN;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_INPUT_ANALOG:
	config.GPIO_Mode = GPIO_Mode_AN;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_INPUT_PU:
	config.GPIO_Mode = GPIO_Mode_IN;
	config.GPIO_PuPd = GPIO_PuPd_UP;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_INPUT_PD:
	config.GPIO_Mode = GPIO_Mode_IN;
	config.GPIO_PuPd = GPIO_PuPd_DOWN;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_AF_OUTPUT_PP:
	config.GPIO_Mode = GPIO_Mode_AF;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_AF_OUTPUT_OD:
	config.GPIO_Mode = GPIO_Mode_AF;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_OD;
	break;
    case GPIO_AF_OUTPUT_OD_PU:
	config.GPIO_Mode = GPIO_Mode_AF;
	config.GPIO_PuPd = GPIO_PuPd_UP;
	config.GPIO_OType = GPIO_OType_OD;
	break;
    default:
	return;
    }

    config.GPIO_Pin = BIT(pin);
    gpio_init(dev->GPIOx, &config);
}

void gpio_init(GPIO_TypeDef* GPIOx, GPIO_Init_t* conf)
{
  uint32_t pinpos;

  /* ------------------------- Configure the port pins ---------------- */
  /*-- GPIO Mode Configuration --*/
  for (pinpos = 0x00; pinpos < 0x10; pinpos++) {
    uint32_t pos = ((uint32_t)0x01) << pinpos;
    /* Get the port pins position */
    uint32_t currentpin = (conf->GPIO_Pin) & pos;

    if (currentpin == pos) {
      GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
      GPIOx->MODER |= (((uint32_t)conf->GPIO_Mode) << (pinpos * 2));

      if ((conf->GPIO_Mode == GPIO_Mode_OUT) || (conf->GPIO_Mode == GPIO_Mode_AF)) {
        /* Speed mode configuration */
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
        GPIOx->OSPEEDR |= ((uint32_t)(conf->GPIO_Speed) << (pinpos * 2));

        /* Output mode configuration*/
        GPIOx->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos)) ;
        GPIOx->OTYPER |= (uint16_t)(((uint16_t)conf->GPIO_OType) << ((uint16_t)pinpos));
      }
      
      /* Pull-up Pull down resistor configuration*/
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));
      GPIOx->PUPDR |= (((uint32_t)conf->GPIO_PuPd) << (pinpos * 2));
    }
  }
}

