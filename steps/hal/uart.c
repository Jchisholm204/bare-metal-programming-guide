/**
 * @file uart.c
 * @author Jacob Chisholm (Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date 2023-09-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hal.h"

inline void uart_init(struct uart *uart, unsigned long baud) {
    // figure 19. selecting an alternate function (7=spi2/3, usart1..3, uart5, spdif-in)
    uint8_t af = 7;           // Alternate function
    uint16_t rx = 0, tx = 0;  // pins

    if (uart == UART1) RCC->APB2ENR |= BIT(4);
    if (uart == UART2) RCC->APB1ENR |= BIT(17);
    if (uart == UART3) RCC->APB1ENR |= BIT(18);

    if (uart == UART1) tx = PIN('A', 9), rx = PIN('A', 10);
    if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3);
    if (uart == UART3) tx = PIN('D', 8), rx = PIN('D', 9);

    gpio_set_mode(tx, AF);
    gpio_set_af(tx, af);
    gpio_set_mode(rx, AF);
    gpio_set_af(rx, af);
    uart->CR1 = 0;                           // Disable this UART
    uart->BRR = FREQ / baud;                 // FREQ is a UART bus frequency
    uart->CR1 |= BIT(13) | BIT(2) | BIT(3);  // Set UE, RE, TE
}

inline int uart_read_ready(struct uart *uart){
    return uart->SR & BIT(5);
}

inline uint8_t uart_read_byte(struct uart *uart) {
    return ((uint8_t) (uart->DR & 255));
}

inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

inline void uart_write_byte(struct uart * uart, uint8_t byte) {
    uart->DR = byte;
    while ((uart->SR & BIT(7)) == 0) spin(1);
    
}

inline void uart_write_buf(struct uart *uart, char *buf, size_t len){
    while(len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}