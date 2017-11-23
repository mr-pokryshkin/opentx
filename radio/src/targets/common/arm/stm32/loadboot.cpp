/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x 
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"
#include "../../../taranis/board.h"

// start address of application in flash
#define APP_START_ADDRESS (uint32_t)0x08008000

typedef void (*voidFunction)(void);

#define enableKeysPeriphClock() {                                       \
        RCC->AHB1ENR |= KEYS_RCC_AHB1Periph;                            \
        /* these two NOPs are needed (see STM32F errata sheet) before the peripheral */ \
        /* register can be written after the peripheral clock was enabled */ \
        __ASM volatile ("nop");                                         \
        __ASM volatile ("nop");                                         \
    }

#define turnPwrOn() {                                             \
        PWR_GPIO->BSRRL = PWR_ON_GPIO_PIN;                        \
        PWR_GPIO->MODER = (PWR_GPIO->MODER & ~PWR_ON_GPIO_MODER)  \
            | PWR_ON_GPIO_MODER;                                  \
    }

#define jumpTo(addr) {                                          \
        uint32_t     jumpAddress = *(uint32_t*)(addr);          \
        voidFunction jumpFn = (voidFunction)jumpAddress;        \
        jumpFn();                                               \
    }

__attribute__ ((section(".bootrodata.*"), used))
const uint8_t BootCode[] = {
#include "bootloader.lbm"
};

__attribute__ ((section(".bootrodata"), used))
void _bootStart()
{
    enableKeysPeriphClock();

  // Turn soft power ON now, but only if we got started because of the watchdog
  // or software reset. If the radio was started by user pressing the power button
  // then that button is providing power and we don't need to enable it here.
  //
  // If we were to turn it on here indiscriminately, then the radio can go into the 
  // power on/off loop after being powered off by the user. (issue #2790)
  if (WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {

      turnPwrOn();
  }

  // turn on pull-ups on trim keys
  TRIMS_GPIO_LHR_PUPDR = 0;
  TRIMS_GPIO_RHL_PUPDR = 0;

  TRIMS_GPIO_LHR_PUPDR |= TRIMS_GPIO_PIN_LHR;
  TRIMS_GPIO_RHL_PUPDR |= TRIMS_GPIO_PIN_RHL;

  // wait for inputs to stabilize
  for (uint32_t i = 0; i < 50000; i += 1) {
    wdt_reset();
  }

  // now the second part of power on sequence
  // If we got here and the radio was not started by the watchdog/software reset,
  // then we must have a power button pressed. If not then we are in power on/off loop
  // and to terminate it, just wait here without turning on PWR pin. The power supply will
  // eventually exhaust and the radio will turn off.
  if (!WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {
    // wait here until the power key is pressed
    while (PWR_SWITCH_GPIO_REG & PWR_SWITCH_GPIO_PIN) {
      wdt_reset();
    }
  }

  if (!(TRIMS_GPIO_REG_LHR & TRIMS_GPIO_PIN_LHR)
      && !(TRIMS_GPIO_REG_RHL & TRIMS_GPIO_PIN_RHL)) {

    // Bootloader needed
    const uint8_t *src;
    uint8_t *dest;
    uint32_t size;

    wdt_reset();
    size = sizeof(BootCode);
    src = BootCode;
    dest = (uint8_t *) SRAM_BASE;

    for (; size; size -= 1) {
      *dest++ = *src++;
    }
    // Could check for a valid copy to RAM here
    // Go execute bootloader
    wdt_reset();
    jumpTo(SRAM_BASE + 4);
  }

  // Start the main application
  SCB->VTOR = APP_START_ADDRESS;                  // set the VTOR
  __set_MSP(*(__IO uint32_t*)APP_START_ADDRESS);  // set stack pointer value

  // Jump to proper application address
  jumpTo(APP_START_ADDRESS + 4);
}

__attribute__ ((section(".isr_boot_vector"), used))
const uint32_t BootVectors[] = {
  (uint32_t) &_estack,
  (uint32_t) (void (*)(void)) ((unsigned long) &_bootStart)
};
