/** @file gpio_irq_dispatch.h
 * @brief Shared core-0 GPIO-bank interrupt ownership.
 */
#ifndef GPIO_IRQ_DISPATCH_H
#define GPIO_IRQ_DISPATCH_H
#include "app.h"

/* Register during core-0 startup, before starting subsystem tasks.
 * Handlers inspect and acknowledge ONLY their own GPIO edge sources.
 * Configure/enable pin edges after registration. No allocation or blocking
 * is permitted inside callbacks. Do not install another bank-0 handler.
 */
typedef void (*app_gpio_irq_handler_t)(UINT intno);
ER app_gpio_irq_register(UW pins, app_gpio_irq_handler_t handler);
#endif
