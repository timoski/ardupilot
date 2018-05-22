#include "rcc.h"
#include <hal.h>


void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource)
{
  if ((RCC_RTCCLKSource & 0x00000300) == 0x00000300)  { // If HSE is selected as RTC clock source, configure HSE division factor for RTC clock 
     uint32_t tmpreg = RCC->CFGR & ~RCC_CFGR_RTCPRE;;    // Clear RTCPRE[4:0] bits 

    // Configure HSE division factor for RTC clock 
    RCC->CFGR = tmpreg | (RCC_RTCCLKSource & 0xFFFFCFF);
  }
  
  RCC->BDCR |= (RCC_RTCCLKSource & 0x00000FFF); // Select the RTC clock source
}


ErrorStatus RCC_WaitForHSEStartUp(void)
{
  uint32_t startupcounter = 0;
  bool hsestatus;
  // Wait till HSE is ready and if Time out is reached exit
  do {
    hsestatus = RCC_GetFlagStatus(RCC_FLAG_HSERDY);
    startupcounter++;
  } while((startupcounter != HSE_STARTUP_TIMEOUT) && (hsestatus == false));

  if (RCC_GetFlagStatus(RCC_FLAG_HSERDY)) {
    return SUCCESS;
  } else {
    return ERROR;
  }
}


bool RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
  uint32_t statusreg;
  
  // Get the RCC register index 
  uint32_t tmp = RCC_FLAG >> 5;
  if (tmp == 1){              // The flag to check is in CR register 
    statusreg = RCC->CR;
  } else if (tmp == 2) {      // The flag to check is in BDCR register 
    statusreg = RCC->BDCR;
  }  else {                   // The flag to check is in CSR register 
    statusreg = RCC->CSR;
  }
  
  // Get the flag position 
  tmp = RCC_FLAG & FLAG_MASK;
  if ((statusreg & ((uint32_t)1 << tmp)) != 0) {
    return true;
  }
  return false;
}

void RCC_HSEConfig(uint8_t RCC_HSE)
{
  *(__IO uint8_t *) CR_BYTE3_ADDRESS = RCC_HSE_OFF; // Reset HSEON and HSEBYP bits before configuring the HSE  
  *(__IO uint8_t *) CR_BYTE3_ADDRESS = RCC_HSE; // Set the new HSE configuration 
}


static uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

void RCC_GetClocksFreq(RCC_Clocks_t* RCC_Clocks) // Compute HCLK, PCLK1 and PCLK2 clocks frequencies 
{
  RCC_Clocks->SYSCLK_Frequency = SystemCoreClock;
  
  // Get HCLK prescaler 
  uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE) >> 4;
  RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> APBAHBPrescTable[tmp]; // HCLK clock frequency 

  // Get PCLK1 prescaler 
  tmp = (RCC->CFGR & RCC_CFGR_PPRE1) >> 10;
  RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency >> APBAHBPrescTable[tmp]; // PCLK1 clock frequency

  // Get PCLK2 prescaler
  tmp = (RCC->CFGR & RCC_CFGR_PPRE2) >> 13;
  RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency >> APBAHBPrescTable[tmp]; // PCLK2 clock frequency
}

inline void RCC_RTCCLKCmd(FunctionalState NewState){   *(__IO uint32_t *) BDCR_RTCEN_BB = (uint32_t)NewState; }


inline void RCC_AHB1PeriphClockCmd(uint32_t RCC_AHB1Periph, bool state)
{
  if (state) {
    RCC->AHB1ENR |= RCC_AHB1Periph;
  } else {
    RCC->AHB1ENR &= ~RCC_AHB1Periph;
  }
}

inline void RCC_AHB2PeriphClockCmd(uint32_t RCC_AHB2Periph, bool state)
{
  if (state){
    RCC->AHB2ENR |= RCC_AHB2Periph;
  } else {
    RCC->AHB2ENR &= ~RCC_AHB2Periph;
  }
}

inline void RCC_AHB3PeriphClockCmd(uint32_t RCC_AHB3Periph, bool state)
{
  if (state) {
    RCC->AHB3ENR |= RCC_AHB3Periph;
  } else {
    RCC->AHB3ENR &= ~RCC_AHB3Periph;
  }
}

inline void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, bool state)
{
  if (state) {
    RCC->APB1ENR |= RCC_APB1Periph;
  } else {
    RCC->APB1ENR &= ~RCC_APB1Periph;
  }
}

inline void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, bool state)
{
  if (state) {
    RCC->APB2ENR |= RCC_APB2Periph;
  } else {
    RCC->APB2ENR &= ~RCC_APB2Periph;
  }
}

inline void RCC_AHB1PeriphResetCmd(uint32_t RCC_AHB1Periph, bool state)
{
  if (state) {
    RCC->AHB1RSTR |= RCC_AHB1Periph;
  } else {
    RCC->AHB1RSTR &= ~RCC_AHB1Periph;
  }
}

inline void RCC_AHB2PeriphResetCmd(uint32_t RCC_AHB2Periph, bool state)
{
  if (state) {
    RCC->AHB2RSTR |= RCC_AHB2Periph;
  } else {
    RCC->AHB2RSTR &= ~RCC_AHB2Periph;
  }
}

inline void RCC_AHB3PeriphResetCmd(uint32_t RCC_AHB3Periph, bool state)
{
  if (state) {
    RCC->AHB3RSTR |= RCC_AHB3Periph;
  } else {
    RCC->AHB3RSTR &= ~RCC_AHB3Periph;
  }
}


inline void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, bool state)
{
  if (state) {
    RCC->APB1RSTR |= RCC_APB1Periph;
  } else {
    RCC->APB1RSTR &= ~RCC_APB1Periph;
  }
}

inline void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, bool state)
{
  if (state) {
    RCC->APB2RSTR |= RCC_APB2Periph;
  } else  {
    RCC->APB2RSTR &= ~RCC_APB2Periph;
  }
}

