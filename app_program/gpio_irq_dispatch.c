#include "gpio_irq_dispatch.h"

#define GPIO_BANK_IRQ 13u
#define GPIO_BANK_PRIORITY 2
#define GPIO_SUBSCRIBERS 4u

typedef struct
{
    UW pins;
    app_gpio_irq_handler_t handler;
} gpio_subscriber_t;

static gpio_subscriber_t g_subscribers[GPIO_SUBSCRIBERS];
static UINT g_subscriber_count;

static void app_gpio_irq_dispatch(UINT intno)
{
    UINT index;
    for (index = 0u; index < g_subscriber_count; index++)
    {
        g_subscribers[index].handler(intno);
    }
    ClearInt(intno);
}

ER app_gpio_irq_register(UW pins, app_gpio_irq_handler_t handler)
{
    UINT index;
    UINT status;
    ER error = E_OK;
    const T_DINT definition =
    {
        .intatr = TA_HLNG,
        .inthdr = app_gpio_irq_dispatch
    };
#if TK_SUPPORT_SMP
    if (tk_get_prc() != 1)
    {
        return E_CTX;
    }
#endif
    if (handler == NULL || pins == 0u || (pins & ~0x3FFFFFFFu) != 0u)
    {
        return E_PAR;
    }
    DI(status);
    for (index = 0u; index < g_subscriber_count; index++)
    {
        if ((pins & g_subscribers[index].pins) != 0u)
        {
            EI(status);
            return E_OBJ;
        }
    }
    if (g_subscriber_count >= GPIO_SUBSCRIBERS)
    {
        EI(status);
        return E_LIMIT;
    }
    if (g_subscriber_count == 0u)
    {
        error = tk_def_int(GPIO_BANK_IRQ, &definition);
    }
    if (error == E_OK)
    {
        g_subscribers[g_subscriber_count].pins = pins;
        g_subscribers[g_subscriber_count].handler = handler;
        g_subscriber_count++;
        EnableInt(GPIO_BANK_IRQ, GPIO_BANK_PRIORITY);
    }
    EI(status);
    return error;
}
