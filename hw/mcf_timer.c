/*
 * ColdFire Timer emulation.
 *
 * Copyright (c) 2007 CodeSourcery.
 *
 * This code is licenced under the GPL
 */

#include "hw.h"
#include "mcf.h"
#include "qemu-timer.h"

/* General purpose timer module.  */
typedef struct {
    uint16_t tmr;
    uint16_t trr;
    uint16_t tcr;
    uint16_t ter;
    ptimer_state *timer;
    qemu_irq irq;
    int irq_state;
    unsigned int sysclk;
} mcf_timer_state;

#define TMR_RST 0x01
#define TMR_CLK 0x06
#define TMR_FRR 0x08
#define TMR_ORI 0x10
#define TMR_OM  0x20
#define TMR_CE  0xc0

#define TER_CAP 0x01
#define TER_REF 0x02

static void mcf_timer_update(mcf_timer_state *s)
{
    if ((s->tmr & TMR_ORI) != 0 && (s->ter & TER_REF)) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static void mcf_timer_reset(mcf_timer_state *s)
{
    s->tmr = 0;
    s->trr = 0;
}

static void mcf_timer_recalibrate(mcf_timer_state *s)
{
    int prescale;
    int mode;

    ptimer_stop(s->timer);

    if ((s->tmr & TMR_RST) == 0) {
        return;
    }

    prescale = (s->tmr >> 8) + 1;
    mode = (s->tmr >> 1) & 3;
    if (mode == 2) {
        prescale *= 16;
    }
    if (mode == 3 || mode == 0) {
        hw_error("mcf_timer: mode %d not implemented\n", mode);
    }
    if ((s->tmr & TMR_FRR) == 0) {
        hw_error("mcf_timer: free running mode not implemented\n");
    }

    /* Assume 66MHz system clock.  */
    ptimer_set_freq(s->timer, s->sysclk / prescale);

    ptimer_set_limit(s->timer, s->trr, 0);

    ptimer_run(s->timer, 0);
}

static void mcf_timer_trigger(void *opaque)
{
    mcf_timer_state *s = (mcf_timer_state *)opaque;
    s->ter |= TER_REF;
    mcf_timer_update(s);
}

uint32_t mcf_timer_read(void *opaque, uint32_t addr)
{
    mcf_timer_state * s = (mcf_timer_state *)opaque;
    switch (addr) {
    case 0:
        return s->tmr;
    case 4:
        return s->trr;
    case 8:
        return s->tcr;
    case 0xc:
        return s->trr - ptimer_get_count(s->timer);
    case 0x11:
        return s->ter;
    default:
        return 0;
    }
}

void mcf_timer_write(void *opaque, uint32_t addr, uint32_t val)
{
    mcf_timer_state * s = (mcf_timer_state *)opaque;
    switch (addr) {
    case 0:
        if ((s->tmr & TMR_RST) != 0 && (val & TMR_RST) == 0) {
            mcf_timer_reset(s);
        }
        s->tmr = val;
        mcf_timer_recalibrate(s);
        break;
    case 4:
        s->trr = val;
        mcf_timer_recalibrate(s);
        break;
    case 8:
        s->tcr = val;
        break;
    case 0xc:
        ptimer_set_count(s->timer, val);
        break;
    case 0x11:
        s->ter &= ~val;
        break;
    default:
        break;
    }
    mcf_timer_update(s);
}

void *mcf_timer_init(qemu_irq irq, unsigned int sysclk)
{
    mcf_timer_state *s;
    QEMUBH *bh;

    s = (mcf_timer_state *)qemu_mallocz(sizeof(mcf_timer_state));
    bh = qemu_bh_new(mcf_timer_trigger, s);
    s->timer = ptimer_init(bh);
    s->irq = irq;
    s->sysclk = sysclk;
    mcf_timer_reset(s);
    return s;
}
