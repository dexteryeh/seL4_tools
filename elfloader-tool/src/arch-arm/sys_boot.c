/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright 2021, HENSOLDT Cyber
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <autoconf.h>
#include <elfloader/gen_config.h>

#include <drivers.h>
#include <drivers/uart.h>
#include <printf.h>
#include <types.h>
#include <abort.h>
#include <strops.h>
#include <cpuid.h>

#include <binaries/efi/efi.h>
#include <elfloader.h>

/* 0xd00dfeed in big endian */
#define DTB_MAGIC (0xedfe0dd0)

/* Maximum alignment we need to preserve when relocating (64K) */
#define MAX_ALIGN_BITS (14)

#ifdef CONFIG_IMAGE_EFI
ALIGN(BIT(PAGE_BITS)) VISIBLE
char core_stack_alloc[CONFIG_MAX_NUM_NODES][BIT(PAGE_BITS)];
#endif

struct image_info kernel_info;
struct image_info user_info;
void const *dtb;
size_t dtb_size;

extern void finish_relocation(int offset, void *_dynamic, unsigned int total_offset);
void continue_boot(int was_relocated);

#define V3S_CCU_AHB1_GATE0        0x01c20060u
#define V3S_CCU_BUS_GATE4         0x01c20070u
#define V3S_CCU_USB_CLK           0x01c200ccu
#define V3S_CCU_DRAM_GATE         0x01c20100u
#define V3S_CCU_MBUS_RESET        0x01c200fcu
#define V3S_CCU_MBUS_CLK          0x01c2015cu
#define V3S_CCU_AHB1_RESET0       0x01c202c0u
#define V3S_CCU_AHB1_RESET2       0x01c202c8u
#define V3S_SYSCON_BASE           0x01c00000u
#define V3S_EMAC_BASE             0x01c30000u
#define V3S_PIO_BASE              0x01c20800u
#define V3S_USB0_PHY_BASE         0x01c19400u
#define V3S_USB0_OTG_BASE         0x01c19000u
#define V3S_USB0_EHCI_BASE        0x01c1a000u
#define V3S_USB0_OHCI_BASE        0x01c1a400u
#define V3S_USB0_PMU_BASE         0x01c1a800u
#define V3S_PIO_PORT_STRIDE       0x24u
#define V3S_PIO_PORT_D            3u
#define V3S_PIO_CFG_BASE          0x00u
#define V3S_PIO_DRV_BASE          0x14u
#define V3S_PIO_PULL_BASE         0x1cu
#define V3S_PIO_FUNC_EMAC         2u
#define V3S_EMAC_SYSCON_EPHY_ADDR (1u << 20)
#define V3S_EMAC_SYSCON_CLK_24M   (1u << 18)
#define V3S_EMAC_SYSCON_LED_LOW   (1u << 17)
#define V3S_EMAC_SYSCON_SHUTDOWN  (1u << 16)
#define V3S_EMAC_SYSCON_SELECT    (1u << 15)
#define V3S_EMAC_SYSCON_RMII_EN   (1u << 13)
#define V3S_EMAC_SYSCON_RGMII     (1u << 2)
#define V3S_EMAC_SYSCON_ETCS_MASK 0x3u
#define V3S_EMAC_SYSCON_V3S_DEFAULT 0x00038000u
#define V3S_EMAC_SYSCON_VALUE \
    (((V3S_EMAC_SYSCON_V3S_DEFAULT | V3S_EMAC_SYSCON_EPHY_ADDR | \
       V3S_EMAC_SYSCON_CLK_24M | V3S_EMAC_SYSCON_LED_LOW | \
       V3S_EMAC_SYSCON_SELECT) & \
      ~(V3S_EMAC_SYSCON_SHUTDOWN | V3S_EMAC_SYSCON_RMII_EN | \
        V3S_EMAC_SYSCON_RGMII | V3S_EMAC_SYSCON_ETCS_MASK)))

static inline uint32_t slx_v3s_read32(uintptr_t addr)
{
    return *(volatile uint32_t *)addr;
}

static inline uint8_t slx_v3s_read8(uintptr_t addr)
{
    return *(volatile uint8_t *)addr;
}

static inline void slx_v3s_write32(uintptr_t addr, uint32_t value)
{
    *(volatile uint32_t *)addr = value;
}

static inline void slx_v3s_write8(uintptr_t addr, uint8_t value)
{
    *(volatile uint8_t *)addr = value;
}

static void slx_v3s_set32(uintptr_t addr, uint32_t mask)
{
    slx_v3s_write32(addr, slx_v3s_read32(addr) | mask);
}

static void slx_v3s_clear32(uintptr_t addr, uint32_t mask)
{
    slx_v3s_write32(addr, slx_v3s_read32(addr) & ~mask);
}

static void slx_v3s_delay_cycles(uint32_t cycles)
{
    for (volatile uint32_t i = 0; i < cycles; ++i) {
        __asm__ volatile("" ::: "memory");
    }
}

static void slx_v3s_enable_emac_pre_kernel(void)
{
    slx_v3s_set32(V3S_CCU_AHB1_GATE0, 1u << 17);
    slx_v3s_clear32(V3S_CCU_AHB1_RESET0, 1u << 17);
    slx_v3s_delay_cycles(120000u);
    slx_v3s_write32(V3S_SYSCON_BASE + 0x30u, V3S_EMAC_SYSCON_VALUE);

    slx_v3s_set32(V3S_CCU_BUS_GATE4, 1u << 0);
    slx_v3s_clear32(V3S_CCU_AHB1_RESET2, 1u << 2);
    slx_v3s_delay_cycles(240000u);
    slx_v3s_write32(V3S_SYSCON_BASE + 0x30u, V3S_EMAC_SYSCON_VALUE);
    slx_v3s_set32(V3S_CCU_AHB1_RESET0, 1u << 17);
    slx_v3s_delay_cycles(120000u);
    slx_v3s_set32(V3S_CCU_AHB1_RESET2, 1u << 2);
    slx_v3s_delay_cycles(240000u);
}

static void slx_v3s_log_emac_handoff(const char *stage)
{
    printf("ELF-loader V3S emac handoff %s: ccu_ahb1=0x%08x gate4=0x%08x "
           "reset0=0x%08x reset2=0x%08x syscon30=0x%08x emac00=0x%08x "
           "emac04=0x%08x emac48=0x%08x pd_cfg0=0x%08x pd_cfg1=0x%08x "
           "pd_cfg2=0x%08x\n",
           stage,
           slx_v3s_read32(V3S_CCU_AHB1_GATE0),
           slx_v3s_read32(V3S_CCU_BUS_GATE4),
           slx_v3s_read32(V3S_CCU_AHB1_RESET0),
           slx_v3s_read32(V3S_CCU_AHB1_RESET2),
           slx_v3s_read32(V3S_SYSCON_BASE + 0x30u),
           slx_v3s_read32(V3S_EMAC_BASE + 0x00u),
           slx_v3s_read32(V3S_EMAC_BASE + 0x04u),
           slx_v3s_read32(V3S_EMAC_BASE + 0x48u),
           slx_v3s_read32(V3S_PIO_BASE + V3S_PIO_PORT_D * V3S_PIO_PORT_STRIDE + V3S_PIO_CFG_BASE),
           slx_v3s_read32(V3S_PIO_BASE + V3S_PIO_PORT_D * V3S_PIO_PORT_STRIDE + V3S_PIO_CFG_BASE + 4u),
           slx_v3s_read32(V3S_PIO_BASE + V3S_PIO_PORT_D * V3S_PIO_PORT_STRIDE + V3S_PIO_CFG_BASE + 8u));
}

static void slx_v3s_usb_phy_write(uint32_t addr, uint32_t data, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        uint32_t temp;

        slx_v3s_write32(V3S_USB0_PHY_BASE + 0x10u, 0u);
        temp = slx_v3s_read32(V3S_USB0_PHY_BASE + 0x10u);
        temp &= ~(0xffu << 8);
        temp |= (addr + i) << 8;
        slx_v3s_write32(V3S_USB0_PHY_BASE + 0x10u, temp);

        temp = slx_v3s_read8(V3S_USB0_PHY_BASE + 0x10u);
        if (data & 1u) {
            temp |= 0x80u;
        } else {
            temp &= ~0x80u;
        }
        temp &= ~0x01u;
        slx_v3s_write8(V3S_USB0_PHY_BASE + 0x10u, (uint8_t)temp);
        slx_v3s_write8(V3S_USB0_PHY_BASE + 0x10u,
                       (uint8_t)(slx_v3s_read8(V3S_USB0_PHY_BASE + 0x10u) | 0x01u));
        slx_v3s_write8(V3S_USB0_PHY_BASE + 0x10u,
                       (uint8_t)(slx_v3s_read8(V3S_USB0_PHY_BASE + 0x10u) & ~0x01u));
        data >>= 1;
    }
}

static void slx_v3s_usb_force_host_id_vbus(void)
{
    uint32_t iscr = slx_v3s_read32(V3S_USB0_PHY_BASE + 0x00u);

    iscr &= ~((3u << 14) | (3u << 12));
    iscr |= 0x42000000u | (1u << 17) | (1u << 16) | (2u << 14) | (2u << 12);
    slx_v3s_write32(V3S_USB0_PHY_BASE + 0x00u, iscr);
    slx_v3s_delay_cycles(2400000u);

    iscr = slx_v3s_read32(V3S_USB0_PHY_BASE + 0x00u);
    iscr &= ~((3u << 14) | (3u << 12));
    iscr |= 0x42000000u | (1u << 17) | (1u << 16) | (2u << 14) | (3u << 12);
    slx_v3s_write32(V3S_USB0_PHY_BASE + 0x00u, iscr);
}

static void slx_v3s_enable_usb_hci_pre_kernel(void)
{
    const uint32_t bus_usb_v3s_mask = (1u << 24) | (1u << 26) | (1u << 29);
    const uint32_t linux_visible_bus_peer_mask = (1u << 17);
    const uint32_t bus_dram_mask = (1u << 14);
    const uint32_t linux_bus_gate_mask = bus_usb_v3s_mask | linux_visible_bus_peer_mask;
    const uint32_t linux_bus_reset_mask = linux_bus_gate_mask | bus_dram_mask;
    const uint32_t usb_clk_v3s_mask = (1u << 0) | (1u << 8) | (1u << 16);
    const uint32_t dram_hci_mask = (1u << 17) | (1u << 18);
    const uint32_t pmu_passby_mask = (1u << 10) | (1u << 9) | (1u << 8) | (1u << 0);
    const uint32_t pmu_siddq_isolation_mask = (1u << 3) | (1u << 1);
    const uint32_t pio_pf6_cfg_mask = 0x0fu << 24;
    const uint32_t pio_pf6_pull_mask = 0x03u << 12;
    const uint32_t pio_pf6_pull_down = 0x02u << 12;

    slx_v3s_write32(V3S_PIO_BASE + 0xb4u,
                    slx_v3s_read32(V3S_PIO_BASE + 0xb4u) & ~pio_pf6_cfg_mask);
    slx_v3s_write32(V3S_PIO_BASE + 0xd0u,
                    (slx_v3s_read32(V3S_PIO_BASE + 0xd0u) & ~pio_pf6_pull_mask) |
                    pio_pf6_pull_down);
    slx_v3s_write32(V3S_PIO_BASE + 0xd8u, 0x77777111u);
    slx_v3s_write32(V3S_PIO_BASE + 0xe8u, 0x00000006u);

    slx_v3s_set32(V3S_CCU_MBUS_RESET, 1u << 31);
    slx_v3s_write32(V3S_CCU_MBUS_CLK, 0x81000003u);
    slx_v3s_set32(V3S_CCU_AHB1_GATE0, linux_bus_gate_mask);
    slx_v3s_clear32(V3S_CCU_AHB1_GATE0, bus_dram_mask);
    slx_v3s_set32(V3S_CCU_USB_CLK, usb_clk_v3s_mask);
    slx_v3s_set32(V3S_CCU_AHB1_RESET0, linux_bus_reset_mask);
    slx_v3s_clear32(V3S_CCU_DRAM_GATE, dram_hci_mask);
    slx_v3s_delay_cycles(120000u);

    slx_v3s_clear32(V3S_USB0_PMU_BASE + 0x10u, pmu_siddq_isolation_mask);
    slx_v3s_usb_phy_write(0x0cu, 0x01u, 1u);
    slx_v3s_usb_phy_write(0x20u, 0x14u, 5u);
    slx_v3s_usb_phy_write(0x2au, 0x03u, 2u);
    slx_v3s_set32(V3S_USB0_PMU_BASE + 0x00u, pmu_passby_mask);
    slx_v3s_usb_force_host_id_vbus();
    slx_v3s_clear32(V3S_USB0_PHY_BASE + 0x20u, 1u);
    slx_v3s_write8(V3S_USB0_OTG_BASE + 0x43u, 0u);
    slx_v3s_write8(V3S_USB0_OTG_BASE + 0x40u, 0xe0u);
    slx_v3s_write8(V3S_USB0_OTG_BASE + 0x41u, 0x19u);
    slx_v3s_write8(V3S_USB0_OTG_BASE + 0x42u, 0x04u);
    slx_v3s_delay_cycles(360000u);
}

static void slx_v3s_log_usb_handoff(const char *stage)
{
    printf("ELF-loader V3S usb handoff %s: ccu_ahb1=0x%08x usb_clk=0x%08x "
           "dram_gate=0x%08x bus_reset=0x%08x ehci00=0x%08x ehci04=0x%08x "
           "ohci00=0x%08x ohci54=0x%08x phy_iscr=0x%08x phy_ctl=0x%08x "
           "phy_otgctl=0x%08x pmu00=0x%08x pmu10=0x%08x\n",
           stage,
           slx_v3s_read32(V3S_CCU_AHB1_GATE0),
           slx_v3s_read32(V3S_CCU_USB_CLK),
           slx_v3s_read32(V3S_CCU_DRAM_GATE),
           slx_v3s_read32(V3S_CCU_AHB1_RESET0),
           slx_v3s_read32(V3S_USB0_EHCI_BASE + 0x00u),
           slx_v3s_read32(V3S_USB0_EHCI_BASE + 0x04u),
           slx_v3s_read32(V3S_USB0_OHCI_BASE + 0x00u),
           slx_v3s_read32(V3S_USB0_OHCI_BASE + 0x54u),
           slx_v3s_read32(V3S_USB0_PHY_BASE + 0x00u),
           slx_v3s_read32(V3S_USB0_PHY_BASE + 0x10u),
           slx_v3s_read32(V3S_USB0_PHY_BASE + 0x20u),
           slx_v3s_read32(V3S_USB0_PMU_BASE + 0x00u),
           slx_v3s_read32(V3S_USB0_PMU_BASE + 0x10u));
}

/*
 * Make sure the ELF loader is below the kernel's first virtual address
 * so that when we enable the MMU we can keep executing.
 */
extern char _DYNAMIC[];
void relocate_below_kernel(void)
{
    /*
     * These are the ELF loader's physical addresses,
     * since we are either running with MMU off or
     * identity-mapped.
     */
    uintptr_t UNUSED start = (uintptr_t)_text;
    uintptr_t end = (uintptr_t)_end;

    if (end <= kernel_info.virt_region_start) {
        /*
         * If the ELF loader is already below the kernel,
         * skip relocation.
         */
        continue_boot(0);
        return;
    }

#ifdef CONFIG_IMAGE_EFI
    /*
     * Note: we make the (potentially incorrect) assumption
     * that there is enough physical RAM below the kernel's first vaddr
     * to fit the ELF loader.
     * FIXME: do we need to make sure we don't accidentally wipe out the DTB too?
     */
    uintptr_t size = end - start;

    /*
     * we ROUND_UP size in this calculation so that all aligned things
     * (interrupt vectors, stack, etc.) end up in similarly aligned locations.
     * The strictes alignment requirement we have is the 64K-aligned AArch32
     * page tables, so we use that to calculate the new base of the elfloader.
     */
    uintptr_t new_base = kernel_info.virt_region_start - (ROUND_UP(size, MAX_ALIGN_BITS));
    uint32_t offset = start - new_base;
    printf("relocating from %p-%p to %p-%p... size=0x%x (padded size = 0x%x)\n", start, end, new_base, new_base + size,
           size, ROUND_UP(size, MAX_ALIGN_BITS));

    memmove((void *)new_base, (void *)start, size);

    /* call into assembly to do the finishing touches */
    finish_relocation(offset, _DYNAMIC, new_base);
#else
    printf("ERROR: The ELF loader does not support relocating itself. You"
           " probably need to move the kernel window higher, or the load"
           " address lower.\n");
    abort();
#endif
}

/*
 * Entry point.
 *
 * Unpack images, setup the MMU, jump to the kernel.
 */
void main(UNUSED void *arg)
{
    void *bootloader_dtb = NULL;

    /* initialize platform to a state where we can print to a UART */
    if (initialise_devices()) {
        printf("ERROR: Did not successfully return from initialise_devices()\n");
        abort();
    }

    platform_init();

    /* Print welcome message. */
    printf("\nELF-loader started on ");
    print_cpuid();
    printf("  paddr=[%p..%p]\n", _text, (uintptr_t)_end - 1);

#if defined(CONFIG_IMAGE_UIMAGE)

    /* U-Boot passes a DTB. Ancient bootloaders may pass atags. When booting via
     * bootelf argc is NULL.
     */
    if (arg && (DTB_MAGIC == *(uint32_t *)arg)) {
        bootloader_dtb = arg;
    }

#elif defined(CONFIG_IMAGE_EFI)

    if (efi_exit_boot_services() != EFI_SUCCESS) {
        printf("ERROR: Unable to exit UEFI boot services!\n");
        abort();
    }

    bootloader_dtb = efi_get_fdt();

#endif

    if (bootloader_dtb) {
        printf("  dtb=%p\n", bootloader_dtb);
    } else {
        printf("No DTB passed in from boot loader.\n");
    }

    /* Unpack ELF images into memory. */
    unsigned int num_apps = 0;
    int ret = load_images(&kernel_info, &user_info, 1, &num_apps,
                          bootloader_dtb, &dtb, &dtb_size);
    if (0 != ret) {
        printf("ERROR: image loading failed\n");
        abort();
    }

    if (num_apps != 1) {
        printf("ERROR: expected to load just 1 app, actually loaded %u apps\n",
               num_apps);
        abort();
    }
    /*
     * We don't really know where we've been loaded.
     * It's possible that EFI loaded us in a place
     * that will become part of the 'kernel window'
     * once we switch to the boot page tables.
     * Make sure this is not the case.
     */
    relocate_below_kernel();
    printf("ERROR: Relocation failed, aborting!\n");
    abort();
}

void continue_boot(int was_relocated)
{
    if (was_relocated) {
        printf("ELF loader relocated, continuing boot...\n");
    }

    slx_v3s_log_usb_handoff("continue-entry");
    slx_v3s_enable_usb_hci_pre_kernel();
    slx_v3s_log_usb_handoff("after-prekernel-hci-enable");
    slx_v3s_log_emac_handoff("before-prekernel-enable");
    slx_v3s_enable_emac_pre_kernel();
    slx_v3s_log_emac_handoff("after-prekernel-enable");

    /*
     * If we were relocated, we need to re-initialise the
     * driver model so all its pointers are set up properly.
     */
    if (was_relocated) {
        if (initialise_devices()) {
            printf("ERROR: Did not successfully return from initialise_devices()\n");
            abort();
        }
    }

#if (defined(CONFIG_ARCH_ARM_V7A) || defined(CONFIG_ARCH_ARM_V8A)) && !defined(CONFIG_ARM_HYPERVISOR_SUPPORT)
    if (is_hyp_mode()) {
        extern void leave_hyp(void);
        leave_hyp();
    }
#endif
    /* Setup MMU. */
    if (is_hyp_mode()) {
#ifdef CONFIG_ARCH_AARCH64
        extern void disable_caches_hyp();
        disable_caches_hyp();
#endif
        init_hyp_boot_vspace(&kernel_info);
    } else {
        /* If we are not in HYP mode, we enable the SV MMU and paging
         * just in case the kernel does not support hyp mode. */
        init_boot_vspace(&kernel_info);
    }

#if CONFIG_MAX_NUM_NODES > 1
    smp_boot();
#endif /* CONFIG_MAX_NUM_NODES */

    slx_v3s_log_usb_handoff("pre-kernel-entry");
    slx_v3s_log_emac_handoff("pre-kernel-entry");

    if (is_hyp_mode()) {
        printf("Enabling hypervisor MMU and jumping to entry point...\n\n");
        arm_enable_hyp_mmu();
    } else {
        printf("Enabling MMU and jumping to entry point...\n\n");
        arm_enable_mmu();
    }

    /* Enter kernel. The UART is no longer accessible here. */
    ((init_arm_kernel_t)kernel_info.virt_entry)(user_info.phys_region_start,
                                                user_info.phys_region_end,
                                                user_info.phys_virt_offset,
                                                user_info.virt_entry,
                                                (word_t)dtb,
                                                dtb_size);

    /* We should never get here. */
    abort();
}
