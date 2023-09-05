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

volatile bool led_on = true;
volatile uint64_t increment = 0;

int main(void){
    uint16_t led1 = PIN('B', 0);
    uint16_t led2 = PIN('B', 1);
    systick_init(FREQ / 1000);
    gpio_set_mode(led1, OUTPUT);
    gpio_set_mode(led2, OUTPUT);
    uart_init(UART2, 9600);
    uint32_t timer = 0, period = 100;
    gpio_write(led2, true);
    for(;;) {
        if(timer_expired(&timer, period, s_ticks)){
            gpio_write(led1, led_on);
            led_on = !led_on;
            //printf("LED: %d, Ticks: %lu\r\n", led_on, s_ticks);//bricks
        }
    }
    return 0;
}