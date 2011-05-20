#ifndef HW_MCF_H
#define HW_MCF_H
/* Motorola ColdFire device prototypes.  */

/* mcf_uart.c */
uint32_t mcf_uart_read(void *opaque, target_phys_addr_t addr);
void mcf_uart_write(void *opaque, target_phys_addr_t addr, uint32_t val);
void *mcf_uart_init(qemu_irq irq, CharDriverState *chr);
void mcf_uart_mm_init(target_phys_addr_t base, qemu_irq irq,
                      CharDriverState *chr);

/* mcf_intc.c */
qemu_irq *mcf_intc_init(target_phys_addr_t base, CPUState *env);

/* mcf_fec.c */
void mcf_fec_init(NICInfo *nd, target_phys_addr_t base, qemu_irq *irq);

/* mcf_timer.c */
void *mcf_timer_init(qemu_irq irq, unsigned int sysclk);
uint32_t mcf_timer_read(void *opaque, uint32_t addr);
void mcf_timer_write(void *opaque, uint32_t addr, uint32_t val);

/* mcf5206.c */
qemu_irq *mcf5206_init(uint32_t base, CPUState *env);

#endif
