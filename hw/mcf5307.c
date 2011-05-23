/*
 * Motorola ColdFire MCF5307 SoC embedded peripheral emulation.
 *
 * Copyright (c) 2011 Paul Roukema.
 * Based on hw/mcf5206.c - Copyright (c) 2007 CodeSourcery.
 * This code is licenced under the GPL
 */
#include "hw.h"
#include "mcf.h"

#include "sysemu.h"



/* System Integration Module.  */

typedef struct {
    CPUState *env;
    void *timer[2];
    void *uart[2];
    uint8_t icr[10];
    uint8_t avec;
    uint32_t imr; /* 1 == interrupt is masked.  */
    uint32_t ipr;
    uint8_t rsr;
    uint8_t swivr;
    uint16_t par;
    /* Include the UART vector registers here.  */
    uint8_t uivr[2];
} m5307_mbar_state;

/* Interrupt controller.  */

static int m5307_find_pending_irq(m5307_mbar_state *s)
{
    int level;
    int curlevel;
    int vector;
    uint16_t active;
    int i;

    /* level format is 00LLLPPE
     * LLL is 3 bit level
     * PP  is 2 bit priority for ICR'd interrupts, 1 for externals
     * E   is one bit flag for external interrupts
     */
    level = 0;
    vector = 0;
    active = s->ipr & ~s->imr;
    if (!active) {
        return 0;
    }
    /* IRQ lines have an implicit level and priority */
    for (i = 1; i < 8; i++) {
        if (active & (1 << i)) {
            curlevel = i << 3 | 3;
            if (curlevel > level) {
                level = curlevel;
                vector = i;
            }
        }
    }
    /* internal interrupts get priority from ICR */
    for (; i < 18; i++) {
        if (active & (1 << i)) {
            curlevel = ((s->icr[i-7] & 0x1C) << 3) | ((s->icr[i-7] & 0x3) << 1);
            if (curlevel > level) {
                level = curlevel;
                vector = i;
            }
        }
    }

    /* All real interrupts have an LLL of at least 1 */
    if (level < 8) {
        vector = 0;
    }
    return vector;
}

static void m5307_mbar_update(m5307_mbar_state *s)
{
    int irq;
    int vector;
    int level;

    irq = m5307_find_pending_irq(s);
    if (irq) {
        if (irq < 8) {
            level = irq;
            if (s->avec & 1 << irq) {
                vector = 24 + irq;
            } else {
                fprintf(stderr,
                "Non-Autovectored Exteral interrupt %d\n", irq);
                vector = 0xf;
            }
        } else {
            int tmp;
            tmp = s->icr[irq-7];
            level = (tmp >> 2) & 7;
            if (tmp & 0x80) {
                /* Autovector.  */
                vector = 24 + level;
            } else {
                switch (irq) {
                case 8: /* SWT */
                    vector = s->swivr;
                    break;
                case 12: /* UART1 */
                    vector = s->uivr[0];
                    break;
                case 13: /* UART2 */
                    vector = s->uivr[1];
                    break;
                default:
                    /* Unknown vector.  */
                    fprintf(stderr, "Unhandled vector for IRQ %d\n", irq);
                    vector = 0xf;
                    break;
                }
            }
        }
    } else {
        level = 0;
        vector = 0;
    }
    m68k_set_irq_level(s->env, level, vector);
}

static void m5307_mbar_set_irq(void *opaque, int irq, int level)
{
    m5307_mbar_state *s = (m5307_mbar_state *)opaque;
    if (level) {
        s->ipr |= 1 << irq;
    } else {
        s->ipr &= ~(1 << irq);
    }
    m5307_mbar_update(s);
}

/* System Integration Module.  */

static void m5307_mbar_reset(m5307_mbar_state *s)
{
    s->icr[1] = 0x00;
    s->icr[2] = 0x00;
    s->icr[3] = 0x00;
    s->icr[4] = 0x00;
    s->icr[5] = 0x00;
    s->icr[6] = 0x00;
    s->icr[7] = 0x00;
    s->icr[8] = 0x00;
    s->icr[9] = 0x00;
    s->avec = 0x0;
    s->imr = 0x3fffe;
    s->rsr = 0x80;
    s->swivr = 0x0f;
    s->par = 0;
}

static uint32_t m5307_mbar_read(m5307_mbar_state *s, uint32_t offset)
{
    if (offset >= 0x140 && offset < 0x160) {
        return mcf_timer_read(s->timer[0], offset - 0x140);
    } else if (offset >= 0x180 && offset < 0x1A0) {
        return mcf_timer_read(s->timer[1], offset - 0x180);
    } else if (offset >= 0x1C0 && offset < 0x1E0) {
        return mcf_uart_read(s->uart[0], offset - 0x1C0);
    } else if (offset >= 0x200 && offset < 0x220) {
        return mcf_uart_read(s->uart[1], offset - 0x200);
    }
    switch (offset) {
    case 0x00: return s->rsr;
    case 0x02: return s->swivr;
    case 0x04: return s->par;
    case 0x44: return s->imr;
    case 0x4B: return s->avec;
    case 0x4C ... 0x55: return s->icr[offset - 0x4B];
    case 0x40: return s->ipr;
    case 0x1F0: return s->uivr[0];
    case 0x230: return s->uivr[1];
    }
    hw_error("Bad MBAR read offset 0x%x", (int)offset);
    return 0;
}

static void m5307_mbar_write(m5307_mbar_state *s, uint32_t offset,
                             uint32_t value)
{
    if (offset >= 0x140 && offset < 0x160) {
        mcf_timer_write(s->timer[0], offset - 0x140, value);
        return;
    } else if (offset >= 0x180 && offset < 0x1A0) {
        mcf_timer_write(s->timer[1], offset - 0x180, value);
        return;
    } else if (offset >= 0x1C0 && offset < 0x1E0) {
        mcf_uart_write(s->uart[0], offset - 0x1C0, value);
        return;
    } else if (offset >= 0x200 && offset < 0x220) {
        mcf_uart_write(s->uart[1], offset - 0x200, value);
        return;
    }
    switch (offset) {
    case 0x00:
        s->rsr &= ~value;
        break;
    case 0x02:
        s->swivr = value;
        break;
    case 0x04:
        s->par = value;
        break;
    case 0x44:
        s->imr = value;
        m5307_mbar_update(s);
        break;
    case 0x4B:
        s->avec = value;
        m5307_mbar_update(s);
        break;
    case 0x4C ... 0x55:
        s->icr[offset - 0x4B] = value;
        m5307_mbar_update(s);
        break;
    case 0x41:
        /* TODO: implement watchdog.  */
        break;
    case 0x1F0:
        s->uivr[0] = value;
        break;
    case 0x1F8: case 0x1FC: case 0x238: case 0x23c:
        /* Not implemented: UART Output port bits.  */
        break;
    case 0x230:
        s->uivr[1] = value;
        break;
    default:
        hw_error("Bad MBAR write offset 0x%x", (int)offset);
        break;
    }
}

/* Internal peripherals use a variety of register widths.
   This lookup table allows a single routine to handle all of them.  */
static const int m5307_mbar_width[] = {
  /* 000-040 */ 1, 2, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
  /* 040-080 */ 4, 4, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
  /* 080-0c0 */ 2, 4, 2, 2,  4, 2, 2, 4,  2, 2, 4, 2,  2, 4, 2, 2,
  /* 0c0-100 */ 1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
  /* 100-140 */ 2, 4, 4, 4,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
  /* 140-180 */ 2, 2, 2, 2,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
  /* 180-1c0 */ 2, 2, 2, 2,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
  /* 1c0-200 */ 1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
  /* 200-240 */ 1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,
};

static uint32_t m5307_mbar_readw(void *opaque, target_phys_addr_t offset);
static uint32_t m5307_mbar_readl(void *opaque, target_phys_addr_t offset);

static uint32_t m5307_mbar_readb(void *opaque, target_phys_addr_t offset)
{
    m5307_mbar_state *s = (m5307_mbar_state *)opaque;
    offset &= 0x3ff;
    if (offset > 0x240) {
        hw_error("Bad MBAR read offset 0x%x", (int)offset);
    }
    if (m5307_mbar_width[offset >> 2] > 1) {
        uint16_t val;
        val = m5307_mbar_readw(opaque, offset & ~1);
        if ((offset & 1) == 0) {
            val >>= 8;
        }
        return val & 0xff;
    }
    return m5307_mbar_read(s, offset);
}

static uint32_t m5307_mbar_readw(void *opaque, target_phys_addr_t offset)
{
    m5307_mbar_state *s = (m5307_mbar_state *)opaque;
    int width;
    offset &= 0x3ff;
    if (offset > 0x240) {
        hw_error("Bad MBAR read offset 0x%x", (int)offset);
    }
    width = m5307_mbar_width[offset >> 2];
    if (width > 2) {
        uint32_t val;
        val = m5307_mbar_readl(opaque, offset & ~3);
        if ((offset & 3) == 0) {
            val >>= 16;
        }
        return val & 0xffff;
    } else if (width < 2) {
        uint16_t val;
        val = m5307_mbar_readb(opaque, offset) << 8;
        val |= m5307_mbar_readb(opaque, offset + 1);
        return val;
    }
    return m5307_mbar_read(s, offset);
}

static uint32_t m5307_mbar_readl(void *opaque, target_phys_addr_t offset)
{
    m5307_mbar_state *s = (m5307_mbar_state *)opaque;
    int width;
    offset &= 0x3ff;
    if (offset > 0x240) {
        hw_error("Bad MBAR read offset 0x%x", (int)offset);
    }
    width = m5307_mbar_width[offset >> 2];
    if (width < 4) {
        uint32_t val;
        val = m5307_mbar_readw(opaque, offset) << 16;
        val |= m5307_mbar_readw(opaque, offset + 2);
        return val;
    }
    return m5307_mbar_read(s, offset);
}

static void m5307_mbar_writew(void *opaque, target_phys_addr_t offset,
                              uint32_t value);
static void m5307_mbar_writel(void *opaque, target_phys_addr_t offset,
                              uint32_t value);

static void m5307_mbar_writeb(void *opaque, target_phys_addr_t offset,
                              uint32_t value)
{
    m5307_mbar_state *s = (m5307_mbar_state *)opaque;
    int width;
    offset &= 0x3ff;
    if (offset > 0x240) {
        hw_error("Bad MBAR write offset 0x%x", (int)offset);
    }
    width = m5307_mbar_width[offset >> 2];
    if (width > 1) {
        uint32_t tmp;
        tmp = m5307_mbar_readw(opaque, offset & ~1);
        if (offset & 1) {
            tmp = (tmp & 0xff00) | value;
        } else {
            tmp = (tmp & 0x00ff) | (value << 8);
        }
        m5307_mbar_writew(opaque, offset & ~1, tmp);
        return;
    }
    m5307_mbar_write(s, offset, value);
}

static void m5307_mbar_writew(void *opaque, target_phys_addr_t offset,
                              uint32_t value)
{
    m5307_mbar_state *s = (m5307_mbar_state *)opaque;
    int width;
    offset &= 0x3ff;
    if (offset > 0x240) {
        hw_error("Bad MBAR write offset 0x%x", (int)offset);
    }
    width = m5307_mbar_width[offset >> 2];
    if (width > 2) {
        uint32_t tmp;
        tmp = m5307_mbar_readl(opaque, offset & ~3);
        if (offset & 3) {
            tmp = (tmp & 0xffff0000) | value;
        } else {
            tmp = (tmp & 0x0000ffff) | (value << 16);
        }
        m5307_mbar_writel(opaque, offset & ~3, tmp);
        return;
    } else if (width < 2) {
        m5307_mbar_writeb(opaque, offset, value >> 8);
        m5307_mbar_writeb(opaque, offset + 1, value & 0xff);
        return;
    }
    m5307_mbar_write(s, offset, value);
}

static void m5307_mbar_writel(void *opaque, target_phys_addr_t offset,
                              uint32_t value)
{
    m5307_mbar_state *s = (m5307_mbar_state *)opaque;
    int width;
    offset &= 0x3ff;
    if (offset > 0x240) {
        hw_error("Bad MBAR write offset 0x%x", (int)offset);
    }
    width = m5307_mbar_width[offset >> 2];
    if (width < 4) {
        m5307_mbar_writew(opaque, offset, value >> 16);
        m5307_mbar_writew(opaque, offset + 2, value & 0xffff);
        return;
    }
    m5307_mbar_write(s, offset, value);
}

static CPUReadMemoryFunc * const m5307_mbar_readfn[] = {
   m5307_mbar_readb,
   m5307_mbar_readw,
   m5307_mbar_readl
};

static CPUWriteMemoryFunc * const m5307_mbar_writefn[] = {
   m5307_mbar_writeb,
   m5307_mbar_writew,
   m5307_mbar_writel
};

qemu_irq *mcf5307_init(uint32_t base, CPUState *env)
{
    m5307_mbar_state *s;
    qemu_irq *pic;
    int iomemtype;

    s = (m5307_mbar_state *)qemu_mallocz(sizeof(m5307_mbar_state));
    iomemtype = cpu_register_io_memory(m5307_mbar_readfn,
                                       m5307_mbar_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    cpu_register_physical_memory(base, 0x00001000, iomemtype);

    pic = qemu_allocate_irqs(m5307_mbar_set_irq, s, 14);
    s->timer[0] = mcf_timer_init(pic[9], 45000000);
    s->timer[1] = mcf_timer_init(pic[10], 45000000);
    s->uart[0] = mcf_uart_init(pic[12], serial_hds[0]);
    s->uart[1] = mcf_uart_init(pic[13], serial_hds[1]);
    s->env = env;

    m5307_mbar_reset(s);
    return pic;
}
