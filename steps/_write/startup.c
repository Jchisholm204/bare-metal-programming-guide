/**
 * @file startup.c
 * @author Jacob Chisholm (Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date 2023-09-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */


extern void main(void);

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

extern void SysTick_Handler(void); // defined in main.c
extern void _estack(void); // defined in link.ld

// 16 standard (mandated by arm arch) and 97 stm32 entries (f446RE vector table (table 38))
__attribute__((section(".vectors"))) void (*const tab[16+97])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler
};
