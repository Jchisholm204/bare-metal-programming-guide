/**
 * @file main.c
 * @author Jacob Chisholm (https://jchisholm.github.io) //
 * from bare metal programming guide by --
 * @brief simple led blinking program
 * @date 2023-08-30
 * 
 */
#include "hal.h"

// Macros to make code more readable


static volatile uint32_t s_ticks;
void SysTick_Handler(void){
    s_ticks++;
}

volatile bool led_on;
volatile uint64_t increment = 0;

int main(void){
    uint16_t led1 = PIN('B', 0);
    uint16_t led2 = PIN('B', 1);
    gpio_set_mode(led1, OUTPUT);
    gpio_set_mode(led2, OUTPUT);
    uart_init(UART2, 9600);
    gpio_write(led2, true);
    uint32_t timer, period = 100;
    systick_init(16000000 / 1000);
    for(;;) {
        if(timer_expired(&timer, period, s_ticks)){
            gpio_write(led1, led_on);
            led_on = !led_on;
            uart_write_buf(UART2, "hi\r\n", 4);
        }
    }
    return 0;
}


// Startup Code
// Loads main function from flash into SRAM on startup
__attribute__((naked, noreturn)) void _reset(void) {
    // menset .bss to zero and copy the .data section to the ram region
    extern long _sbss, _ebss, _sdata, _edata, _sidata;
    for (long *dst = &_sbss; dst < &_ebss; dst++) *dst=0;
    for (long *dst =&_sdata, *src = &_sidata; dst < & _edata;) *dst ++ = *src++;

    main(); // call main()
    for (;;) (void) 0; // infinite loop if main returns
}

extern void _estack(void); // defined in link.ld

// 16 standard (mandated by arm arch) and 97 stm32 entries (f446RE vector table (table 38))
__attribute__((section(".vectors"))) void (*const tab[16+97])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler
};