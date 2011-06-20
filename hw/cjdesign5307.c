/*
 * CJ Design MCF5307 "Janus" ColdFire system emulation.
 *
 * Copyright (c) 2011 Paul Roukema.
 * Based on hw/an5206.c - Copyright (c) 2007 CodeSourcery.
 * This code is licenced under the GPL
 */

#include "hw.h"
#include "pc.h"
#include "mcf.h"
#include "boards.h"
#include "loader.h"
#include "elf.h"


#define CJ5307_RAM_ADDR  0x10000000
#define KERNEL_LOAD_ADDR (CJ5307_RAM_ADDR + 0x1000)
#define CJ5307_MBAR_ADDR 0xF0000000
#define CJ5307_RAMBAR_ADDR 0xFFFF8000


/* Board init.  */

static void cjdesign5307_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    CPUState *env;
    int kernel_size;
    uint64_t elf_entry;
    target_phys_addr_t entry;

    if (!cpu_model) {
        cpu_model = "m5307";
    }
    env = cpu_init(cpu_model);
    if (!env) {
        hw_error("Unable to find m68k CPU definition\n");
    }

    /* Initialize CPU registers.  */
    env->vbr = 0x0;
    /* TODO: allow changing MBAR and RAMBAR. */
    env->mbar = CJ5307_MBAR_ADDR | 1;
    env->rambar0 = CJ5307_RAMBAR_ADDR | 1;

    /* DRAM at address 0x10000000*/
    cpu_register_physical_memory(CJ5307_RAM_ADDR, ram_size,
        qemu_ram_alloc(NULL, "cjdesign5307.sdram", ram_size) | IO_MEM_RAM);

    /* Internal SRAM.  */
    cpu_register_physical_memory(CJ5307_RAMBAR_ADDR, 4096,
        qemu_ram_alloc(NULL, "m5307.sram", 4096) | IO_MEM_RAM);

    mcf5307_init(CJ5307_MBAR_ADDR, env);

    /* Load kernel.  */
    if (!kernel_filename) {
        fprintf(stderr, "Kernel image must be specified\n");
        exit(1);
    }

    kernel_size = load_elf(kernel_filename, NULL, NULL, &elf_entry,
                           NULL, NULL, 1, ELF_MACHINE, 0);
    entry = elf_entry;
    if (kernel_size < 0) {
        kernel_size = load_srec(kernel_filename, &entry);
    }
    if (kernel_size < 0) {
        kernel_size = load_uimage(kernel_filename, &entry, NULL, NULL);
    }
    if (kernel_size < 0) {
        kernel_size = load_image_targphys(kernel_filename, KERNEL_LOAD_ADDR,
                                          ram_size - KERNEL_LOAD_ADDR);
        entry = KERNEL_LOAD_ADDR;
    }
    if (kernel_size < 0) {
        fprintf(stderr, "qemu: could not load kernel '%s'\n", kernel_filename);
        exit(1);
    }

    env->pc = entry;
}

static QEMUMachine cjdesign5307_machine = {
    .name = "cj5307",
    .desc = "CJ Design 5307",
    .init = cjdesign5307_init,
};

static void cjdesign5307_machine_init(void)
{
    qemu_register_machine(&cjdesign5307_machine);
}

machine_init(cjdesign5307_machine_init);
