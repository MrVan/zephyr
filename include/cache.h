/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_CACHE_H_
#define ZEPHYR_INCLUDE_CACHE_H_

#include <kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Common operations for the caches
 */
#define K_CACHE_INVD		BIT(0)
#define K_CACHE_FLUSH		BIT(1)
#define K_CACHE_CLEAN		BIT(2)

/**
 *
 * @brief Enable d-cache
 *
 * Enable the d-cache.
 *
 * @return N/A
 */
void arch_dcache_enable(void);

/**
 *
 * @brief Disable d-cache
 *
 * Disable the d-cache.
 *
 * @return N/A
 */
void arch_dcache_disable(void);

/**
 *
 * @brief Enable i-cache
 *
 * Enable the i-cache.
 *
 * @return N/A
 */
void arch_icache_enable(void);

/**
 *
 * @brief Disable i-cache
 *
 * Disable the i-cache.
 *
 * @return N/A
 */
void arch_icache_disable(void);

/**
 *
 * @brief Flush / Invalidate / Clean one level d-cache
 *
 * Flush, Invalidate or Clean one lever of the d-cache
 *
 * @param level	The cache level to act on
 * @param op	Operation to perform (one of the K_CACHE_* operations)
 *
 * @retval 0		On success
 * @retval -ENOTSUP	If the operation is not supported
 */
int arch_dcache_level(int level, int op);

/**
 *
 * @brief Flush / Invalidate / Clean all d-cache
 *
 * Flush, Invalidate or Clean the whole d-cache.
 *
 * @param op	Operation to perform (one of the K_CACHE_* operations)
 *
 * @retval 0		On success
 * @retval -ENOTSUP	If the operation is not supported
 */
int arch_dcache_all(int op);

/**
 *
 * @brief Flush / Invalidate / Clean d-cache lines
 *
 * No alignment is required for either addr or size, but since
 * arch_dcache_range() iterates on the d-cache lines, a d-cache line alignment
 * for both is optimal.
 *
 * The d-cache line size is specified either via the CONFIG_CACHE_LINE_SIZE
 * kconfig option or it is detected at runtime.
 *
 * @param addr	The pointer to start the multi-line flush
 * @param size	The number of bytes that are to be flushed
 * @param op	Operation to perform (one of the K_CACHE_* operations)
 *
 * @retval 0		On success
 * @retval -ENOTSUP	If the operation is not supported
 */
int arch_dcache_range(void *addr, size_t size, int op);

/**
 *
 * @brief Flush / Invalidate / Clean all i-cache
 *
 * Flush, Invalidate or Clean the whole i-cache.
 *
 * @param op	Operation to perform (one of the K_CACHE_* operations)
 *
 * @retval 0		On success
 * @retval -ENOTSUP	If the operation is not supported
 */
int arch_icache_all(int op);

/**
 *
 * @brief Flush / Invalidate / Clean i-cache lines
 *
 * No alignment is required for either addr or size, but since
 * arch_icache_range() iterates on the i-cache lines, an i-cache line alignment
 * for both is optimal.
 *
 * The i-cache line size is specified either via the CONFIG_CACHE_LINE_SIZE
 * kconfig option or it is detected at runtime.
 *
 * @param addr	The pointer to start the multi-line flush
 * @param size	The number of bytes that are to be flushed
 * @param op	Operation to perform (one of the K_CACHE_* operations)
 *
 * @retval 0		On success
 * @retval -ENOTSUP	If the operation is not supported
 */
int arch_icache_range(void *addr, size_t size, int op);

__syscall int sys_dcache_level(int level, int op);
static inline int z_impl_sys_dcache_level(int level, int op)
{
	if (IS_ENABLED(CONFIG_CACHE_MANAGEMENT)) {
		return arch_dcache_level(level, op);
	}
	return -ENOTSUP;
}

__syscall int sys_dcache_all(int op);
static inline int z_impl_sys_dcache_all(int op)
{
	if (IS_ENABLED(CONFIG_CACHE_MANAGEMENT)) {
		return arch_dcache_all(op);
	}
	return -ENOTSUP;
}

__syscall int sys_dcache_range(void *addr, size_t size, int op);
static inline int z_impl_sys_dcache_range(void *addr, size_t size, int op)
{
	if (IS_ENABLED(CONFIG_CACHE_MANAGEMENT)) {
		return arch_dcache_range(addr, size, op);
	}
	return -ENOTSUP;
}

__syscall int sys_icache_all(int op);
static inline int z_impl_sys_icache_all(int op)
{
	if (IS_ENABLED(CONFIG_CACHE_MANAGEMENT)) {
		return arch_icache_all(op);
	}
	return -ENOTSUP;
}

__syscall int sys_icache_range(void *addr, size_t size, int op);
static inline int z_impl_sys_icache_range(void *addr, size_t size, int op)
{
	if (IS_ENABLED(CONFIG_CACHE_MANAGEMENT)) {
		return arch_icache_range(addr, size, op);
	}
	return -ENOTSUP;
}

#ifdef CONFIG_LIBMETAL
static inline void sys_cache_flush(void *addr, size_t size)
{
	sys_dcache_range(addr, size,  K_CACHE_FLUSH);
}
#endif

/**
 *
 * @brief Get the d-cache line size.
 *
 * The API is provided to get the cache line size.
 *
 * @return size of the cache line or 0 if the cache is not enabled.
 */
static inline size_t sys_cache_line_size_get(void)
{
#ifdef CONFIG_CACHE_MANAGEMENT
#ifdef CONFIG_CACHE_LINE_SIZE
	return CONFIG_CACHE_LINE_SIZE;
#else
	return arch_cache_line_size_get();
#endif /* CONFIG_CACHE_LINE_SIZE */
#else
	return 0;
#endif /* CONFIG_CACHE_MANAGEMENT */
}

#include <syscalls/cache.h>
#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_CACHE_H_ */
