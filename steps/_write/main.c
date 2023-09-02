/**
 * @file main.c
 * @author Jacob Chisholm (https://jchisholm.github.io) //
 * from bare metal programming guide by --
 * @brief simple led blinking program
 * @date 2023-08-30
 * 
 */

#include "hal.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void){
    s_ticks++;
}

volatile bool led_on;
volatile uint64_t increment = 0;

int main(void){
    uint32_t timer, period = 100;
    uint16_t led1 = PIN('B', 0);
    uint16_t led2 = PIN('B', 1);
    systick_init(FREQ / 1000);
    gpio_set_mode(led1, GPIO_MODE_OUTPUT);
    gpio_set_mode(led2, GPIO_MODE_OUTPUT);
    uart_init(UART2, 9600);
    gpio_write(led2, true);
    for(;;) {
        if(timer_expired(&timer, period, s_ticks)){
            gpio_write(led1, led_on);
            led_on = !led_on;
            uart_write_buf(UART2, "hi\r\n", 4);
        }
    }
    return 0;
}