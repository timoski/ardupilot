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
 

// AF 0 (default) selection  
#define GPIO_AF_RTC_50Hz      (0x00)  // RTC_50Hz 
#define GPIO_AF_MCO           (0x00)  // MCO (MCO1 and MCO2) 
#define GPIO_AF_TAMPER        (0x00)  // TAMPER (TAMPER_1 and TAMPER_2) 
#define GPIO_AF_SWJ           (0x00)  // SWJ (SWD and JTAG) 
#define GPIO_AF_TRACE         (0x00)  // TRACE 


#define GPIO_AF_TIM1          (0x01)  // TIM1 
#define GPIO_AF_TIM2          (0x01)  // TIM2 

// AF 2 selection  
#define GPIO_AF_TIM3          (0x02)  // TIM3 
#define GPIO_AF_TIM4          (0x02)  // TIM4 
#define GPIO_AF_TIM5          (0x02)  // TIM5 

// AF 3 selection  
#define GPIO_AF_TIM8          (0x03)  // TIM8 
#define GPIO_AF_TIM9          (0x03)  // TIM9 
#define GPIO_AF_TIM10         (0x03)  // TIM10 
#define GPIO_AF_TIM11         (0x03)  // TIM11 

// AF 4 selection  
#define GPIO_AF_I2C1          (0x04)  // I2C1 
#define GPIO_AF_I2C2          (0x04)  // I2C2 
#define GPIO_AF_I2C3          (0x04)  // I2C3 

// AF 5 selection  
#define GPIO_AF_SPI1          (0x05)  // SPI1      
#define GPIO_AF_SPI2          (0x05)  // SPI2/I2S2 
#define GPIO_AF_SPI4          (0x05)  // SPI4      
#define GPIO_AF_SPI5          (0x05)  // SPI5      
#define GPIO_AF_SPI6          (0x05)  // SPI6      

// AF 6 selection  
#define GPIO_AF_SPI3          (0x06)  // SPI3/I2S3 

// AF 7 selection  
#define GPIO_AF_USART1        (0x07)  // USART1  
#define GPIO_AF_USART2        (0x07)  // USART2  
#define GPIO_AF_USART3        (0x07)  // USART3  
#define GPIO_AF_I2S3ext       (0x07)  // I2S3ext 

// AF 8 selection  
#define GPIO_AF_UART4         (0x08)  // UART4  
#define GPIO_AF_UART5         (0x08)  // UART5  
#define GPIO_AF_USART6        (0x08)  // USART6 
#define GPIO_AF_UART7         (0x08)  // UART7  
#define GPIO_AF_UART8         (0x08)  // UART8  

//   AF 9 selection 
#define GPIO_AF_CAN1          (0x09)  // CAN1  
#define GPIO_AF_CAN2          (0x09)  // CAN2  
#define GPIO_AF_TIM12         (0x09)  // TIM12 
#define GPIO_AF_TIM13         (0x09)  // TIM13 
#define GPIO_AF_TIM14         (0x09)  // TIM14 

// AF 10 selection  
#define GPIO_AF_OTG_FS         (0xA)  // OTG_FS 
#define GPIO_AF_OTG_HS         (0xA)  // OTG_HS 

//  AF 11 selection  
#define GPIO_AF_ETH             (0x0B)  // ETHERNET 

// AF 12 selection  
#define GPIO_AF_FSMC             (0xC)  // FSMC                     

#define GPIO_AF_OTG_HS_FS        (0xC)  // OTG HS configured in FS, 
#define GPIO_AF_SDIO             (0xC)  // SDIO                     

//     AF 13 selection  
#define GPIO_AF_DCMI          (0x0D)  // DCMI


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
    GPIO_TypeDef *regs;      /**< Register map */
    uint32_t clk; 	      /**< RCC clock information */
    afio_exti_port exti_port; /**< AFIO external interrupt port value */
} gpio_dev;


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
    uint32_t temp = dev->regs->AFR[pin >> 0x03] & ~((uint32_t)0xF << ((uint32_t)((uint32_t)pin & (uint32_t)0x07) * 4));
    dev->regs->AFR[pin >> 0x03] = temp | ((uint32_t)(mode) << ((uint32_t)((uint32_t)pin & (uint32_t)0x07) * 4));
}
    

static INLINE void gpio_write_bit(const gpio_dev* const dev, uint8_t pin, uint8_t val)
{
    uint16_t bv = BIT(pin);
    
    if (val) {
	dev->regs->BSRRL = bv;
    } else {
	dev->regs->BSRRH = bv;
    }    
}

static INLINE uint8_t gpio_read_bit(const gpio_dev* const dev, uint8_t pin)
{
    if ((dev->regs->IDR & BIT(pin)) != 0){
	return  1;
    } 

    return 0;

	
}

static inline void gpio_toggle_bit(const gpio_dev* const dev, uint8_t pin)
{
    dev->regs->ODR ^= BIT(pin);	
}

static inline afio_exti_port gpio_exti_port(const gpio_dev* const dev)
{
    return dev->exti_port;
}


static inline void gpio_set_speed(const gpio_dev* const dev, uint8_t pin, GPIOSpeed_t gpio_speed){
/* Speed mode configuration */
    dev->regs->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << pin * 2);
    dev->regs->OSPEEDR |=     (uint32_t)gpio_speed << pin * 2;
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


