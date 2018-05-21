#ifndef _EXTI_H_
#define _EXTI_H_

#include <string.h>
#include "hal_types.h"

#ifdef __cplusplus
  extern "C" {
#endif
 


#define EXTI_Line0       ((uint32_t)0x00001)     /*!< External interrupt line 0 */
#define EXTI_Line1       ((uint32_t)0x00002)     /*!< External interrupt line 1 */
#define EXTI_Line2       ((uint32_t)0x00004)     /*!< External interrupt line 2 */
#define EXTI_Line3       ((uint32_t)0x00008)     /*!< External interrupt line 3 */
#define EXTI_Line4       ((uint32_t)0x00010)     /*!< External interrupt line 4 */
#define EXTI_Line5       ((uint32_t)0x00020)     /*!< External interrupt line 5 */
#define EXTI_Line6       ((uint32_t)0x00040)     /*!< External interrupt line 6 */
#define EXTI_Line7       ((uint32_t)0x00080)     /*!< External interrupt line 7 */
#define EXTI_Line8       ((uint32_t)0x00100)     /*!< External interrupt line 8 */
#define EXTI_Line9       ((uint32_t)0x00200)     /*!< External interrupt line 9 */
#define EXTI_Line10      ((uint32_t)0x00400)     /*!< External interrupt line 10 */
#define EXTI_Line11      ((uint32_t)0x00800)     /*!< External interrupt line 11 */
#define EXTI_Line12      ((uint32_t)0x01000)     /*!< External interrupt line 12 */
#define EXTI_Line13      ((uint32_t)0x02000)     /*!< External interrupt line 13 */
#define EXTI_Line14      ((uint32_t)0x04000)     /*!< External interrupt line 14 */
#define EXTI_Line15      ((uint32_t)0x08000)     /*!< External interrupt line 15 */
#define EXTI_Line16      ((uint32_t)0x10000)     /*!< External interrupt line 16 Connected to the PVD Output */
#define EXTI_Line17      ((uint32_t)0x20000)     /*!< External interrupt line 17 Connected to the RTC Alarm event */
#define EXTI_Line18      ((uint32_t)0x40000)     /*!< External interrupt line 18 Connected to the USB OTG FS Wakeup from suspend event */
#define EXTI_Line19      ((uint32_t)0x80000)     /*!< External interrupt line 19 Connected to the Ethernet Wakeup event */
#define EXTI_Line20      ((uint32_t)0x00100000)  /*!< External interrupt line 20 Connected to the USB OTG HS (configured in FS) Wakeup event  */
#define EXTI_Line21      ((uint32_t)0x00200000)  /*!< External interrupt line 21 Connected to the RTC Tamper and Time Stamp events */
#define EXTI_Line22      ((uint32_t)0x00400000)  /*!< External interrupt line 22 Connected to the RTC Wakeup event */


typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,
  EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_t;

/**
 * External interrupt line numbers.
 */
typedef enum afio_exti_num {
    AFIO_EXTI_0,                /**< External interrupt line 0. */
    AFIO_EXTI_1,                /**< External interrupt line 1. */
    AFIO_EXTI_2,                /**< External interrupt line 2. */
    AFIO_EXTI_3,                /**< External interrupt line 3. */
    AFIO_EXTI_4,                /**< External interrupt line 4. */
    AFIO_EXTI_5,                /**< External interrupt line 5. */
    AFIO_EXTI_6,                /**< External interrupt line 6. */
    AFIO_EXTI_7,                /**< External interrupt line 7. */
    AFIO_EXTI_8,                /**< External interrupt line 8. */
    AFIO_EXTI_9,                /**< External interrupt line 9. */
    AFIO_EXTI_10,               /**< External interrupt line 10. */
    AFIO_EXTI_11,               /**< External interrupt line 11. */
    AFIO_EXTI_12,               /**< External interrupt line 12. */
    AFIO_EXTI_13,               /**< External interrupt line 13. */
    AFIO_EXTI_14,               /**< External interrupt line 14. */
    AFIO_EXTI_15,               /**< External interrupt line 15. */
} afio_exti_num;

/**
 * External interrupt line port selector.
 *
 * Used to determine which GPIO port to map an external interrupt line
 * onto. */
/* (See AFIO sections, below) */
typedef enum afio_exti_port {
    AFIO_EXTI_PA,               /**< Use port A (PAx) pin. */
    AFIO_EXTI_PB,               /**< Use port B (PBx) pin. */
    AFIO_EXTI_PC,               /**< Use port C (PCx) pin. */
    AFIO_EXTI_PD,               /**< Use port D (PDx) pin. */
    AFIO_EXTI_PE,               /**< Use port E (PEx) pin. */
    AFIO_EXTI_PF,               /**< Use port E (PEx) pin. */
    AFIO_EXTI_PG,               /**< Use port E (PEx) pin. */    
} afio_exti_port;

/** External interrupt trigger mode */
typedef enum exti_trigger_mode {
    EXTI_RISING,         /**< Trigger on the rising edge */
    EXTI_FALLING,        /**< Trigger on the falling edge */
    EXTI_RISING_FALLING  /**< Trigger on both the rising and falling edges */
} exti_trigger_mode;

typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event = 0x04
}EXTIMode_t;

  
void exti_init();

/**
 * @brief Register a handler to run upon external interrupt.
 *
 * This function assumes that the interrupt request corresponding to
 * the given external interrupt is masked.
 *
 * @param num     External interrupt line number.
 * @param port    Port to use as source input for external interrupt.
 * @param handler Function handler to execute when interrupt is triggered.
 * @param mode    Type of transition to trigger on, one of:
 *                EXTI_RISING, EXTI_FALLING, EXTI_RISING_FALLING.
 * @see afio_exti_num
 * @see afio_exti_port
 * @see voidFuncPtr
 * @see exti_trigger_mode
 */
void exti_attach_interrupt(afio_exti_num num,
                           afio_exti_port port,
                           Handler handler,
                           exti_trigger_mode mode);


void exti_attach_interrupt_pri(afio_exti_num num,
                           afio_exti_port port,
                           Handler handler,
                           exti_trigger_mode mode,
                           uint8_t priority);
/**
 * @brief Unregister an external interrupt handler
 * @param num Number of the external interrupt line to disable.
 * @see afio_exti_num
 */
void exti_detach_interrupt(afio_exti_num num);


void exti_enable_irq(afio_exti_num num, bool e); // needed access to internal data

/**
 * Re-enable interrupts.
 *
 * Call this after noInterrupts() to re-enable interrupt handling,
 * after you have finished with a timing-critical section of code.
 *
 * @see noInterrupts()
 */
static INLINE void interrupts() {
        __enable_irq();
}
    
/**
 * Disable interrupts.
 *
 * After calling this function, all user-programmable interrupts will
 * be disabled.  You can call this function before a timing-critical
 * section of code, then call interrupts() to re-enable interrupt
 * handling.
 *
 * @see interrupts()
 */
static INLINE void noInterrupts() {
        __disable_irq();
}



static inline void exti_clear_pending_bit(uint32_t EXTI_Line)
{
  EXTI->PR = EXTI_Line;
}


void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
                       
#ifdef __cplusplus
  }
#endif
 

#endif
