/**
 * @file hal.h
 * @author Jacob Chisholm (Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date 2023-09-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

// CPU Frequency
#define FREQ 16000000

// Set Bit x 1
#define BIT(x) (1UL << (x))
// Package a pin bank (U8) and pin number (U8) into single package (U16)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
// Retrieve pin number (U8) from pin package (U16)
#define PINNO(pin) (pin & 255)
// Retrieve pin bank (U8) from pin package (U16)
#define PINBANK(pin) (pin >> 8)

// struct for referencing GPIO memory (ex. GPIO bank A)
struct gpio {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LKR, AFR[2];
};

// STM32 GPIO Modes (for MODER)
enum GPIO_MODE {
    INPUT, OUTPUT, AF, ANALOG
};

// Struct for accessing STM32F446 Power Control Module
struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
};

struct systick {
    volatile uint32_t CTRL, LOAD, VAL, CALIB;
};

struct uart {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};


// Reference memory address for STM32F446RE GPIO banks
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))
// STM32F446RE RCC Memory Addr
#define RCC ((struct rcc *) 0x40023800)
#define SYSTICK ((struct systick *)  0xe000e010)
#define UART1 ((struct uart *) 0x40011000)
#define UART2 ((struct uart *) 0x40004400)
#define UART3 ((struct uart *) 0x40004800)


extern inline void gpio_set_mode(uint32_t pin, uint8_t mode);

extern inline void gpio_set_af(uint16_t pin, uint8_t af_num);

extern inline void gpio_write(uint16_t pin, bool value);

extern inline void systick_init(uint32_t ticks);

extern inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now);

extern void delay(volatile uint32_t count);

extern inline void uart_init(struct uart *uart, unsigned long baud);

extern inline int uart_read_ready(struct uart *uart);

extern inline uint8_t uart_read_byte(struct uart *uart);

extern inline void uart_write_byte(struct uart * uart, uint8_t byte);

extern inline void uart_write_buf(struct uart *uart, char *buf, size_t len);