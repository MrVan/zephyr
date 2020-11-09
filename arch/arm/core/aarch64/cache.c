/* cache.c - d-cache support for AARCH64 CPUs */

/*
 * Copyright 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief d-cache manipulation
 *
 * This module contains functions for manipulation of the d-cache.
 */

#include <kernel.h>
#include <cache.h>

#define	CTR_EL0_DMINLINE_SHIFT	16
#define	CTR_EL0_DMINLINE_MASK	0xF
#define	CTR_EL0_CWG_SHIFT	24
#define	CTR_EL0_CWG_MASK	0xF

int arch_dcache_flush(void *addr, size_t size);
int arch_dcache_invd(void *addr, size_t size);

int arch_dcache_range(void *addr, size_t size, int op)
{
	if (op == K_CACHE_INVD) {
		arch_dcache_invd(addr, size);
	} else if (op == K_CACHE_FLUSH) {
		arch_dcache_flush(addr, size);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

size_t arch_cache_line_size_get(void)
{
	uint32_t ctr_el0;
	uint32_t cwg;
	uint32_t dminline;

	__asm__ volatile("mrs %0, ctr_el0" : "=r" (ctr_el0));

	cwg = (ctr_el0 >> CTR_EL0_CWG_SHIFT) & CTR_EL0_CWG_MASK;
	dminline = (ctr_el0 >> CTR_EL0_DMINLINE_SHIFT) &
		CTR_EL0_DMINLINE_MASK;

	return cwg ? 4 << cwg : 4 << dminline;
}
