#ifndef _GPIO_H
#define _GPIO_H

#include "hal_types.h"
#include "exti.h"


/**
 * @brief GPIO Pin modes.
 *
 * These only allow for 50MHZ max output speeds; if you want slower,
 * use direct register access.
 */
 
/*
    we should define modes to be compatible with HAL_GPIO_ defines from HAL.h
#define HAL_GPIO_INPUT  0
#define HAL_GPIO_OUTPUT 1
#define HAL_GPIO_ALT    2

*/


#define GPIO_Pin_0                 ((uint16_t)0x0001)  /* Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /* Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /* Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /* Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /* Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /* Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /* Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /* Pin 7 selected */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /* Pin 8 selected */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /* Pin 9 selected */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /* Pin 10 selected */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /* Pin 11 selected */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /* Pin 12 selected */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /* Pin 13 selected */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /* Pin 14 selected */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /* Pin 15 selected */
#define GPIO_Pin_All               ((uint16_t)0xFFFF)  /* All pins selected */

#define GPIO_PinSource0            ((uint8_t)0x00)
#define GPIO_PinSource1            ((uint8_t)0x01)
#define GPIO_PinSource2            ((uint8_t)0x02)
#define GPIO_PinSource3            ((uint8_t)0x03)
#define GPIO_PinSource4            ((uint8_t)0x04)
#define GPIO_PinSource5            ((uint8_t)0x05)
#define GPIO_PinSource6            ((uint8_t)0x06)
#define GPIO_PinSource7            ((uint8_t)0x07)
#define GPIO_PinSource8            ((uint8_t)0x08)
#define GPIO_PinSource9            ((uint8_t)0x09)
#define GPIO_PinSource10           ((uint8_t)0x0A)
#define GPIO_PinSource11           ((uint8_t)0x0B)
#define GPIO_PinSource12           ((uint8_t)0x0C)
#define GPIO_PinSource13           ((uint8_t)0x0D)
#define GPIO_PinSource14           ((uint8_t)0x0E)
#define GPIO_PinSource15           ((uint8_t)0x0F)

// AF 0 (default) selection  
#define GPIO_AF_RTC_50Hz      ((uint8_t)0x00)  // RTC_50Hz 
#define GPIO_AF_MCO           ((uint8_t)0x00)  // MCO (MCO1 and MCO2) 
#define GPIO_AF_TAMPER        ((uint8_t)0x00)  // TAMPER (TAMPER_1 and TAMPER_2) 
#define GPIO_AF_SWJ           ((uint8_t)0x00)  // SWJ (SWD and JTAG) 
#define GPIO_AF_TRACE         ((uint8_t)0x00)  // TRACE 


#define GPIO_AF_TIM1          ((uint8_t)0x01)  // TIM1 
#define GPIO_AF_TIM2          ((uint8_t)0x01)  // TIM2 

// AF 2 selection  
#define GPIO_AF_TIM3          ((uint8_t)0x02)  // TIM3 
#define GPIO_AF_TIM4          ((uint8_t)0x02)  // TIM4 
#define GPIO_AF_TIM5          ((uint8_t)0x02)  // TIM5 

// AF 3 selection  
#define GPIO_AF_TIM8          ((uint8_t)0x03)  // TIM8 
#define GPIO_AF_TIM9          ((uint8_t)0x03)  // TIM9 
#define GPIO_AF_TIM10         ((uint8_t)0x03)  // TIM10 
#define GPIO_AF_TIM11         ((uint8_t)0x03)  // TIM11 

// AF 4 selection  
#define GPIO_AF_I2C1          ((uint8_t)0x04)  // I2C1 
#define GPIO_AF_I2C2          ((uint8_t)0x04)  // I2C2 
#define GPIO_AF_I2C3          ((uint8_t)0x04)  // I2C3 

// AF 5 selection  
#define GPIO_AF_SPI1          ((uint8_t)0x05)  // SPI1      
#define GPIO_AF_SPI2          ((uint8_t)0x05)  // SPI2/I2S2 
#define GPIO_AF_SPI4          ((uint8_t)0x05)  // SPI4      
#define GPIO_AF_SPI5          ((uint8_t)0x05)  // SPI5      
#define GPIO_AF_SPI6          ((uint8_t)0x05)  // SPI6      

// AF 6 selection  
#define GPIO_AF_SPI3          ((uint8_t)0x06)  // SPI3/I2S3 

// AF 7 selection  
#define GPIO_AF_USART1        ((uint8_t)0x07)  // USART1  
#define GPIO_AF_USART2        ((uint8_t)0x07)  // USART2  
#define GPIO_AF_USART3        ((uint8_t)0x07)  // USART3  
#define GPIO_AF_I2S3ext       ((uint8_t)0x07)  // I2S3ext 

// AF 8 selection  
#define GPIO_AF_UART4         ((uint8_t)0x08)  // UART4  
#define GPIO_AF_UART5         ((uint8_t)0x08)  // UART5  
#define GPIO_AF_USART6        ((uint8_t)0x08)  // USART6 
#define GPIO_AF_UART7         ((uint8_t)0x08)  // UART7  
#define GPIO_AF_UART8         ((uint8_t)0x08)  // UART8  

//   AF 9 selection 
#define GPIO_AF_CAN1          ((uint8_t)0x09)  // CAN1  
#define GPIO_AF_CAN2          ((uint8_t)0x09)  // CAN2  
#define GPIO_AF_TIM12         ((uint8_t)0x09)  // TIM12 
#define GPIO_AF_TIM13         ((uint8_t)0x09)  // TIM13 
#define GPIO_AF_TIM14         ((uint8_t)0x09)  // TIM14 

// AF 10 selection  
#define GPIO_AF_OTG_FS         ((uint8_t)0xA)  // OTG_FS 
#define GPIO_AF_OTG_HS         ((uint8_t)0xA)  // OTG_HS 

//  AF 11 selection  
#define GPIO_AF_ETH             ((uint8_t)0x0B)  // ETHERNET 

// AF 12 selection  
#define GPIO_AF_FSMC             ((uint8_t)0xC)  // FSMC                     

#define GPIO_AF_OTG_HS_FS        ((uint8_t)0xC)  // OTG HS configured in FS, 
#define GPIO_AF_SDIO             ((uint8_t)0xC)  // SDIO                     

//     AF 13 selection  
#define GPIO_AF_DCMI          ((uint8_t)0x0D)  // DCMI


#define GPIO_AF_OTG1_FS         GPIO_AF_OTG_FS
#define GPIO_AF_OTG2_HS         GPIO_AF_OTG_HS
#define GPIO_AF_OTG2_FS         GPIO_AF_OTG_HS_FS
 
typedef enum gpio_pin_mode {
    GPIO_INPUT_FLOATING, 	/**< Input floating. */
    GPIO_OUTPUT_PP, 		/**< Output push-pull. */
    GPIO_AF_OUTPUT_PP, 		/**< Alternate function output push-pull. */
// more complex modes
    GPIO_INPUT_ANALOG, 		/**< Analog input. */
    GPIO_INPUT_PD, 		/**< Input pull-down. */
    GPIO_INPUT_PU, 		/**< Input pull-up. */
    /* GPIO_INPUT_PU treated as a special case, for ODR twiddling */
    GPIO_OUTPUT_OD, 		/**< Output open-drain. */
    GPIO_OUTPUT_OD_PU, 		/**< Output open-drain with pullUp */
    GPIO_AF_OUTPUT_OD, 		/**< Alternate function output open drain. */
    GPIO_AF_OUTPUT_OD_PU, 	/**< Alternate function output open drain with pullup */
    GPIO_PIN_MODE_LAST
} gpio_pin_mode;

#define  Bit_RESET 0
#define  Bit_SET   1

typedef enum
{
  GPIO_speed_2MHz   = 0x00, /*!< Low speed */
  GPIO_speed_25MHz  = 0x01, /*!< Medium speed */
  GPIO_speed_50MHz  = 0x02, /*!< Fast speed */
  GPIO_speed_100MHz = 0x03  /*!< High speed on 30 pF (80 MHz Output max speed on 15 pF) */
} GPIOSpeed_t;

typedef enum
{
  GPIO_Mode_IN   = 0x00, /*!< GPIO Input Mode */
  GPIO_Mode_OUT  = 0x01, /*!< GPIO Output Mode */
  GPIO_Mode_AF   = 0x02, /*!< GPIO Alternate function Mode */
  GPIO_Mode_AN   = 0x03  /*!< GPIO Analog Mode */
}GPIOMode_t;

typedef enum
{
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
}GPIOOType_t;

typedef enum
{
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
}GPIOPuPd_t;

/** GPIO device type */
typedef struct gpio_dev {
    GPIO_TypeDef *GPIOx;      /**< Register map */
    uint32_t clk; 	      /**< RCC clock information */
    afio_exti_port exti_port; /**< AFIO external interrupt port value */
} gpio_dev;


//  GPIO Init structure definition  
typedef struct{
  uint32_t GPIO_Pin;              // Specifies the GPIO pins to be configured.           This parameter can be any value of @ref GPIO_pins_define 
  GPIOMode_t GPIO_Mode;     // Specifies the operating mode for the selected pins. This parameter can be a value of @ref GPIOMode_t 
  GPIOSpeed_t GPIO_Speed;   // Specifies the speed for the selected pins.          This parameter can be a value of @ref GPIOSpeed_t 
  GPIOOType_t GPIO_OType;   // Specifies the operating output type for the selected pins.  This parameter can be a value of @ref GPIOOType_t 
  GPIOPuPd_t GPIO_PuPd;     // Specifies the operating Pull-up/Pull down for the selected pins.  This parameter can be a value of @ref GPIOPuPd_t */
}GPIO_Init_t;


#ifdef __cplusplus
  extern "C" {
#endif

extern const gpio_dev gpioa;
extern const gpio_dev* const _GPIOA;
extern const gpio_dev gpiob;
extern const gpio_dev* const _GPIOB;
extern const gpio_dev gpioc;
extern const gpio_dev* const _GPIOC;
extern const gpio_dev gpiod;
extern const gpio_dev* const _GPIOD;
extern const gpio_dev gpioe;
extern const gpio_dev* const _GPIOE;
extern const gpio_dev gpiof;
extern const gpio_dev* const _GPIOF;
extern const gpio_dev gpiog;
extern const gpio_dev* const _GPIOG;

/**
 * Initialize and reset all available GPIO devices. 
 */
extern void gpio_init_all(void);

/**
 * Set the mode of a GPIO pin. 
 */
extern void gpio_set_mode(const gpio_dev* const dev, uint8_t pin, gpio_pin_mode mode);

extern void gpio_init(GPIO_TypeDef* GPIOx, GPIO_Init_t* conf);

/**
 * Set the alternate function mode of a GPIO pin.
 *
 * @param dev GPIO device.
 * @param pin Pin on the device whose mode to set, 0--15.
 * @param mode alternate function mode to set the pin to.
 * @see gpio_pin_mode
 */
static inline void gpio_set_af_mode(const gpio_dev* const dev, uint8_t pin, uint8_t mode)
{
        /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
    assert_param(IS_GPIO_AF(mode));
        
//    GPIO_PinAFConfig(dev->GPIOx, pin, mode);
    uint32_t temp = dev->GPIOx->AFR[pin >> 0x03] & ~((uint32_t)0xF << ((uint32_t)((uint32_t)pin & (uint32_t)0x07) * 4));
    dev->GPIOx->AFR[pin >> 0x03] = temp | ((uint32_t)(mode) << ((uint32_t)((uint32_t)pin & (uint32_t)0x07) * 4));
}
    

static INLINE void gpio_write_bit(const gpio_dev* const dev, uint8_t pin, uint8_t val)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));

    uint16_t bv = BIT(pin);
    
    if (val) {
	dev->GPIOx->BSRRL = bv;
    } else {
	dev->GPIOx->BSRRH = bv;
    }    
}

static INLINE uint8_t gpio_read_bit(const gpio_dev* const dev, uint8_t pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
 
    if ((dev->GPIOx->IDR & BIT(pin)) != Bit_RESET){
	return  (uint8_t)Bit_SET;
    } 

    return (uint8_t)Bit_RESET;

	
}

static inline void gpio_toggle_bit(const gpio_dev* const dev, uint8_t pin)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
    dev->GPIOx->ODR ^= BIT(pin);	
}

static inline afio_exti_port gpio_exti_port(const gpio_dev* const dev)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    return dev->exti_port;
}


static inline void gpio_set_speed(const gpio_dev* const dev, uint8_t pin, GPIOSpeed_t gpio_speed){
/* Speed mode configuration */
    dev->GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
    dev->GPIOx->OSPEEDR |=  ((uint32_t)(gpio_speed) << (pin * 2));
}


static inline void afio_exti_select(afio_exti_port gpio_port, afio_exti_num pin)
{
  uint32_t tmp = ((uint32_t)0x0F) << (0x04 * (pin & (uint8_t)0x03));
  SYSCFG->EXTICR[pin >> 0x02] &= ~tmp;
  SYSCFG->EXTICR[pin >> 0x02] |= (((uint32_t)gpio_port) << (0x04 * (pin & (uint8_t)0x03)));

}


#ifdef __cplusplus
  }
#endif
 
#endif


