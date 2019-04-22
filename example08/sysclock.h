/*----------------------------------------------------------------------
  sysclock.h - configure system clock, flash wait states, and bus speeds
 */
#pragma once

template <const unsigned MCLK_FREQ>
static void sysclock_set()
{
  return;
}

#if F_CPU == 8000000
template<>
void sysclock_set<8000000>(void) {
  /* do nothing just run from the default HSI config */
  /* default is Prefetch enabled and 0 wait states */
  /* turn on half cycle flash */
  SET_BIT(FLASH->ACR, FLASH_ACR_HLFCYA);
  return;
}
#endif

#if F_CPU == 24000000
/* 24MHz - PLL, no wait states, SYCLK APB2 APB1 @ 24MHz */
template<>
void sysclock_set<24000000>() {
  /* Enable HSE */
  SET_BIT(RCC->CR, RCC_CR_HSEON);

  /* Wait until HSE is ready */
  while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) {
    ;
  }

  /* default is Prefetch enabled and 0 wait states */

  SET_BIT(RCC->CFGR,( RCC_CFGR_HPRE_DIV1  /* SYSCLK/1 */
                    | RCC_CFGR_PPRE2_DIV1 /* APB2/1 High Speed Bus */
                    | RCC_CFGR_PPRE1_DIV1 /* APB1/1 Low Speed Bus */
                    | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL3));

  /* Enable PLL */
  SET_BIT(RCC->CR, RCC_CR_PLLON);

  /* Wait until PLL is ready */
  while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) {
    ;
  }

  /* Select PLL as system clock source */
  SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);

  /* Wait until PLL is used as system clock source */
  while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_1) == 0) {
    ;
  }
}
#endif

#if F_CPU == 72000000
template<>
void sysclock_set<72000000>() {
  /* Enable HSE */
  SET_BIT(RCC->CR, RCC_CR_HSEON);

  /* Wait until HSE is ready */
  while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) {
    ;
  }

  /* Enable Prefetch Buffer & set Flash access to 2 wait states */
  SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2);

  SET_BIT(RCC->CFGR,( RCC_CFGR_HPRE_DIV1  /* SysClk/1 72 MHz */
                    | RCC_CFGR_PPRE2_DIV1 /* APB2/1 High Speed 72MHz */
                    | RCC_CFGR_PPRE1_DIV2 /* APB1/2 Low Speed  36MHz */
                    | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9));

  /* Enable PLL */
  SET_BIT(RCC->CR, RCC_CR_PLLON);

  /* Wait until PLL is ready */
  while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0) {
    ;
  }

  /* Select PLL as system clock source */
  SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);

  /* Wait until PLL is used as system clock source */
  while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_1) == 0) {
    ;
  }
}
#endif

/* vim: set ts=2 sw=2 expandtab: */
