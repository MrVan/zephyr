/*
 * Copyright 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_AARCH64_ARCH_INLINES_H
#define ZEPHYR_INCLUDE_ARCH_ARM_AARCH64_ARCH_INLINES_H

#ifndef _ASMLANGUAGE

#include <kernel_structs.h>
#include <arch/cpu.h>

static ALWAYS_INLINE _cpu_t *arch_curr_cpu(void)
{
#ifdef CONFIG_SMP
	uint64_t core;

	/* Note: Only support one Cluster */
	__asm__ volatile( "mrs %0, mpidr_el1" : "=r" (core));
	core = core & 0xFF;

	return &_kernel.cpus[core];
#else
	return &_kernel.cpus[0];
#endif /* CONFIG_SMP */
}

#endif /* !_ASMLANGUAGE */
#endif /* ZEPHYR_INCLUDE_ARCH_ARM_AARCH64_ARCH_INLINES_H */
