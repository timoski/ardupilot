#include "rcc.h"
#include <hal.h>


void RCC_configRTC(uint32_t val)
{
  if ((val & 0x00000300) == 0x00000300)  { // If HSE is selected as RTC clock source, configure HSE division factor for RTC clock 
     uint32_t reg = RCC->CFGR & ~RCC_CFGR_RTCPRE;    // Clear RTCPRE[4:0] bits 

    // Configure HSE division factor for RTC clock 
    RCC->CFGR = reg | (val & 0xFFFFCFF);
  }
  
  RCC->BDCR |= (val & 0x00000FFF); // Select the RTC clock source
}


bool RCC_WaitForHSEStartUp(void)
{
  uint32_t counter = 0;
  bool status;
 
  do {           // Wait till HSE is ready and if Time out is reached exit
    status = RCC_GetFlagStatus(RCC_FLAG_HSERDY);
    counter++;
  } while(!status && (counter != HSE_STARTUP_TIMEOUT));

  return status;
}


bool RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
  uint32_t reg;
  
  // Get the RCC register index 
  uint32_t tmp = RCC_FLAG >> 5;
  if (tmp == 1){              // The flag to check is in CR register 
    reg = RCC->CR;
  } else if (tmp == 2) {      // The flag to check is in BDCR register 
    reg = RCC->BDCR;
  }  else {                   // The flag to check is in CSR register 
    reg = RCC->CSR;
  }
  
  if ((reg & ((uint32_t)1 << (RCC_FLAG & FLAG_MASK))) != 0) {
    return true;
  }
  return false;
}

void RCC_enableHSE(uint8_t hse)
{
  *(__IO uint8_t *) CR_BYTE3_ADDRESS = RCC_HSE_OFF; // Reset HSEON and HSEBYP bits before configuring the HSE  
  *(__IO uint8_t *) CR_BYTE3_ADDRESS = hse;         // Set the new HSE configuration 
}


static uint8_t prescalers[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

void RCC_GetClocksFreq(RCC_Clocks_t* RCC_Clocks) // Compute HCLK, PCLK1 and PCLK2 clocks frequencies 
{
  RCC_Clocks->HCLK_Frequency  = SystemCoreClock            >> prescalers[(RCC->CFGR & RCC_CFGR_HPRE)  >> 4]; // HCLK clock frequency 
  RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency >> prescalers[(RCC->CFGR & RCC_CFGR_PPRE1) >> 10]; // PCLK1 clock frequency
  RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency >> prescalers[(RCC->CFGR & RCC_CFGR_PPRE2) >> 13]; // PCLK2 clock frequency
}


void RCC_doAPB1_reset(uint32_t dev_bit){
    RCC->APB1ENR  |=  dev_bit; // endble clock
    RCC->APB1RSTR |=  dev_bit; // pulse reset
    RCC->APB1RSTR &= ~dev_bit;
}

void RCC_doAPB2_reset(uint32_t dev_bit){
    RCC->APB2ENR  |=  dev_bit; // endble clock
    RCC->APB2RSTR |=  dev_bit; // pulse reset
    RCC->APB2RSTR &= ~dev_bit;
}

void RCC_doAHB1_reset(uint32_t dev_bit){
    RCC->AHB1ENR  |=  dev_bit; // endble clock
    RCC->AHB1RSTR |=  dev_bit; // pulse reset
    RCC->AHB1RSTR &= ~dev_bit;
}
