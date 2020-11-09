/*
 * Copyright 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/**
 * @file
 * @brief codes required for AArch64 multicore and Zephyr smp support
 */

#include <cache.h>
#include <device.h>
#include <kernel.h>
#include <kernel_structs.h>
#include <ksched.h>
#include <soc.h>
#include <init.h>
#include <arch/arm/aarch64/arm_mmu.h>
#include <drivers/interrupt_controller/gic.h>
#include <drivers/psci.h>
#include <sys/arch_interface.h>

volatile struct {
	arch_cpustart_t fn;
	void *arg;
} arm64_cpu_init[CONFIG_MP_NUM_CPUS];

volatile char *arm64_cpu_sp;

volatile uint32_t arm64_cpu_up_flag;

/*
 * _curr_cpu is used to record the struct of _cpu_t of each cpu.
 * for efficient usage in assembly
 */
volatile _cpu_t *_curr_cpu[CONFIG_MP_NUM_CPUS];

extern void __start(void);
/* Called from Zephyr initialization */
void arch_start_cpu(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	const struct device *psci;
	int err;

	_curr_cpu[cpu_num] = &(_kernel.cpus[cpu_num]);
	arm64_cpu_init[cpu_num].fn = fn;
	arm64_cpu_init[cpu_num].arg = arg;

	arm64_cpu_sp = Z_THREAD_STACK_BUFFER(stack) + sz;
	arch_dcache_range((void *)arm64_cpu_sp, sizeof(char *),
			  K_CACHE_FLUSH);

	arm64_cpu_up_flag = cpu_num;
	arch_dcache_range((void *)&arm64_cpu_up_flag, sizeof(uint32_t),
			  K_CACHE_FLUSH);

	psci = device_get_binding("PSCI");
	/* TODO: get mpidr from device tree, using cpu_num */
	err = psci_cpu_on(psci, cpu_num, (uint64_t)&__start);
	if (err)
		printk("Failed to boot CPU%d (%d)\n", cpu_num, err);

	while (arm64_cpu_up_flag != 0) {
		;
	}
}

/* the C entry of slave cores */
void z_arm64_secondary_start(void)
{
	arch_cpustart_t fn;
	int cpu_num = MPIDR_TO_CORE(GET_MPIDR());

	arm_mmu_init(NULL);

	arm_gic_secondary_init();

	irq_enable(0);

	arm64_cpu_up_flag = 0;

	fn = arm64_cpu_init[cpu_num].fn;

	fn(arm64_cpu_init[cpu_num].arg);
}

void sched_ipi_handler(const void *unused)
{
	ARG_UNUSED(unused);

	z_sched_ipi();
}

/* arch implementation of sched_ipi */
void arch_sched_ipi(void)
{
	/* Send SGI to all cores expect itself */
	gic_raise_sgi(0, 0xff, 0xF & ~(1 << MPIDR_TO_CORE(GET_MPIDR())));
}

static int arm64_smp_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* necessary master core init */
	_curr_cpu[0] = &(_kernel.cpus[0]);

	IRQ_CONNECT(0, IRQ_DEFAULT_PRIORITY, sched_ipi_handler, NULL, 0);

	irq_enable(0);

	return 0;
}
SYS_INIT(arm64_smp_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
