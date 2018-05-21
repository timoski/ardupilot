#ifndef _SYSTICK_H
#define _SYSTICK_H

#include <stm32f4xx.h>
#include <hal_types.h>
#include <hal.h>


#ifdef __cplusplus
  extern "C" {
#endif

/** System elapsed time, in milliseconds */
extern volatile uint64_t systick_uptime_millis;
extern voidFuncPtr boardEmergencyHandler;
extern void emerg_delay(uint32_t);


/**
 * @brief Initialize and enable SysTick.
 *
 * Clocks the system timer with the core clock, turns it on, and
 * enables interrupts.
 *
 * @param reload_val Appropriate reload counter to tick every 1 ms.
 */
void systick_init(uint32_t reload_val);

/**
 * Clock the system timer with the core clock and turn it on;
 * interrupt every 1 ms, for systick_timer_millis.
 */
static inline void systick_enable() {   SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; }

/**
 * Clock the system timer with the core clock, but don't turn it
 * on or enable interrupt.
 */
static inline void systick_disable() {  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk; }

/**
 * @brief Returns the system uptime, in milliseconds.
 */
static inline uint64_t systick_uptime(void) {  return systick_uptime_millis; }
/**
 * @brief Returns the current value of the SysTick counter.
 */
static inline uint32_t systick_get_count(void) {  return SysTick->VAL; }

/**
 * @brief Check for underflow.
 *
 * This function returns 1 if the SysTick timer has counted to 0 since
 * the last time it was called.  However, any reads of any part of the
 * SysTick Control and Status Register SYSTICK_BASE->CSR will
 * interfere with this functionality.  See the ARM Cortex M3 Technical
 * Reference Manual for more details (e.g. Table 8-3 in revision r1p1).
 */
static inline uint32_t systick_check_underflow(void) { return SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk; }

/**
 * @brief Attach a callback to be called from the SysTick exception handler.
 *
 * To detach a callback, call this function again with a null argument.
 */
void systick_attach_callback(Handler callback);
void systick_detach_callback(Handler callback);

uint32_t systick_micros(void);

void SysTick_Handler(void);


void HAL_WriteBackupRegister(uint32_t RTC_BKP_DR, uint32_t Data);
uint32_t HAL_ReadBackupRegister(uint32_t RTC_BKP_DR);


/*
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
*/
void __attribute__((noreturn)) __error(uint32_t pc, uint32_t num, uint32_t lr, uint32_t flag);
void __attribute__((noreturn)) error_throb(uint32_t num);


// --------- PWR registers bit address in the alias region ---------- 
#define PWR_OFFSET               (PWR_BASE - PERIPH_BASE)

// PWR CR Register ---

// Alias word address of DBP bit
#define CR_OFFSET                (PWR_OFFSET + 0x00)
#define DBP_BitNumber            0x08
#define CR_DBP_BB                (PERIPH_BB_BASE + (CR_OFFSET * 32) + (DBP_BitNumber * 4))

// Alias word address of PVDE bit 
#define PVDE_BitNumber           0x04
#define CR_PVDE_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PVDE_BitNumber * 4))

// Alias word address of FPDS bit 
#define FPDS_BitNumber           0x09
#define CR_FPDS_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (FPDS_BitNumber * 4))

// Alias word address of PMODE bit 
#define PMODE_BitNumber           0x0E
#define CR_PMODE_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PMODE_BitNumber * 4))


// --- CSR Register ---

// Alias word address of EWUP bit
#define CSR_OFFSET               (PWR_OFFSET + 0x04)
#define EWUP_BitNumber           0x08
#define CSR_EWUP_BB              (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (EWUP_BitNumber * 4))

// Alias word address of BRE bit 
#define BRE_BitNumber            0x09
#define CSR_BRE_BB              (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (BRE_BitNumber * 4))

// ------------------ PWR registers bit mask ------------------------ 

// CR register bit mask 
#define CR_DS_MASK               ((uint32_t)0xFFFFFFFC)
#define CR_PLS_MASK              ((uint32_t)0xFFFFFF1F)
#define CR_VOS_MASK              ((uint32_t)0xFFFF3FFF)



#ifdef __cplusplus
  }
#endif

#endif
