/**
 * @file systick.c
 * @author Jacob Chisholm (Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date 2023-09-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hal.h"

/**
 * @brief Initialize SysTick timer
 * 
 * @param ticks 
 */
inline void systick_init(uint32_t ticks){
    if((ticks -1) > 0xffffff) return; // SysTick timer is limited to 24 bits
    SYSTICK->LOAD = ticks - 1;
    SYSTICK->VAL = 0;
    SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2); //enable systick
    RCC->APB2ENR |= BIT(14);
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

/**
 * @brief Simple Count Delay Timer
 * 
 * @param count number to count to
 */
void delay(volatile uint32_t count){
    while(count--) asm("nop");
}
