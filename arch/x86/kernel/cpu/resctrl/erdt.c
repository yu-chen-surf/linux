// SPDX-License-Identifier: GPL-2.0-only
/*
 * Resource Director Technology(RDT)
 * - Cache Allocation code.
 *
 * Copyright (C) 2025 Intel Corporation
 *
 */

#define pr_fmt(fmt)     "resctrl: " fmt

#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/xarray.h>
#include <linux/resctrl.h>
#include <asm/apic.h>
#include <asm/cpu_device_id.h>
#include "internal.h"
#include "erdt.h"

/* Global variable to hold ERDT ACPI table information for later processing */
static struct erdt_table_info erdt_info;
static DEFINE_XARRAY(erdt_domain_xa); /* Indexed by L3 cache ID */

/*
 * erdt_enabled - Check if the ERDT table is present and enabled
 */
bool erdt_enabled(void)
{
	return erdt_info.available;
}

#ifdef CONFIG_SMP
/*
 * lookup_logical_cpu_by_x2apicid - Map x2APIC ID to logical CPU number
 */
static __init int lookup_logical_cpu_by_x2apicid(u32 x2apicid)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		if (cpu_physical_id(cpu) == x2apicid)
			return cpu;
	}

	return -1;
}
#else
static __init int lookup_logical_cpu_by_x2apicid(u32 x2apicid)
{
	return -1;
}
#endif

/**
 * get_l3_cache_id_from_cacd - Resolve L3 cache ID from CACD subtable
 * @cacd: Pointer to the ACPI ERDT CACD structure
 *
 * Parses the X2APIC ID list in the given CACD subtable to
 * identify an online logical CPU and uses it to query the associated
 * L3 cache ID. The first valid CPU found is used for this lookup.
 *
 * The L3 cache ID is used as a unique domain key for ERDT domain
 * registration and lookup.
 *
 * Return:
 *   L3 cache ID for the first matching CPU, or
 *  -1 if no valid CPU or L3 cache ID could be determined.
 */
static __init int get_l3_cache_id_from_cacd(struct acpi_erdt_cacd *cacd)
{
	int num_ids = (cacd->header.length - sizeof(*cacd)) / sizeof(cacd->X2APICIDS[0]);
	int cpu;

	for (int i = 0; i < num_ids; i++) {
		cpu = lookup_logical_cpu_by_x2apicid(cacd->X2APICIDS[i]);
		if (cpu != -1)
			return get_cpu_cacheinfo_id(cpu, RESCTRL_L3_CACHE);
	}

	return -1;
}

/**
 * parse_rmdd_entry - Parse an ACPI ERDT RMDD entry and populate domain info
 * @rmdd_hdr: Pointer to the ACPI RMDD header structure (ACPI subtable)
 *
 * Parses a single RMDD (Resource Monitoring and Management Domain Descriptor)
 * entry from the ACPI ERDT table. It validates the domain type (only CPU-based
 * L3 cache domains are supported), allocates and populates a domain info
 * structure with pointers to all relevant ERDT substructures
 * (CACD, CMRC, MMRC, MARC), and stores it in a global xarray indexed by the
 * associated L3 cache ID.
 *
 * Calculates the end of the RMDD block based on its length,
 * then walks through all substructures contained within it. It resolves
 * the L3 cache ID from the CACD block to use as the key for domain registration.
 *
 * Return:
 *   0 on success,
 *  -EINVAL if the entry is invalid or unsupported,
 *  -ENOMEM if memory allocation fails.
 */
static __init int parse_rmdd_entry(struct acpi_subtbl_hdr_16 *rmdd_hdr)
{
	struct acpi_erdt_rmdd *rmdd = (struct acpi_erdt_rmdd *)rmdd_hdr;
	struct erdt_domain_info *domain_info __free(kfree) = NULL;
	struct acpi_subtbl_hdr_16 *subtbl;
	void *rmdd_end;
	int l3_cache_id = -1;
	void *ptr;

	/* Only process CPU-based L3 domains (bit 0 set) */
	if (!(rmdd->flags & 0x1))
		return -EINVAL;

	domain_info = kzalloc(sizeof(*domain_info), GFP_KERNEL);
	if (!domain_info)
		return -ENOMEM;

	domain_info->rmdd = rmdd;
	rmdd_end = (void *)rmdd + rmdd->header.length;

	/* Iterate through all sub-structures inside this RMDD block */
	for (subtbl = (void *)rmdd + sizeof(*rmdd);
	     (void *)subtbl < rmdd_end;
	     subtbl = (void *)subtbl + subtbl->length) {
		switch (subtbl->type) {
		case ACPI_ERDT_TYPE_CACD:
			domain_info->cacd = (struct acpi_erdt_cacd *)subtbl;
			l3_cache_id = get_l3_cache_id_from_cacd(domain_info->cacd);
			break;
		case ACPI_ERDT_TYPE_CMRC:
			domain_info->cmrc = (struct acpi_erdt_cmrc *)subtbl;
			break;
		case ACPI_ERDT_TYPE_MMRC:
			domain_info->mmrc = (struct acpi_erdt_mmrc *)subtbl;
			break;
		case ACPI_ERDT_TYPE_MARC:
			domain_info->marc = (struct acpi_erdt_marc *)subtbl;
			break;
		default:
			break;
		}
	}

	if (l3_cache_id == -1) {
		pr_info("ERDT: Failed to resolve L3 cache ID for RMDD domain %d\n",
			rmdd->domain_id);
		return -EINVAL;
	}

	ptr = xa_store(&erdt_domain_xa, l3_cache_id, domain_info, GFP_KERNEL);
	if (xa_is_err(ptr)) {
		pr_info("ERDT: Failed to store domain info for RMDD domain %d\n",
			rmdd->domain_id);
		return -EINVAL;
	}

	domain_info = NULL; /* ownership transferred to xarray */
	return 0;
}

#ifdef CONFIG_X86_64
/**
 * erdt_update_mmio_qword - Helper to safely update MMIO registers
 * @addr: iomem address where update needs to happen
 * @mask: bit position
 * @value: actual value that needs to be updated in the position
 */
static inline void erdt_update_mmio_qword(void __iomem *addr, u64 mask, u64 value)
{
	u64 orig, updated;

	/* Read 8 bytes (64 bits) */
	orig = readq(addr);

	/* Update: clear bits in mask, then set new value bits */
	updated = (orig & ~mask) | (value & mask);

	/* Write updated 8 bytes */
	writeq(updated, addr);
}
#else
static inline void erdt_update_mmio_qword(void __iomem *addr, u64 mask, u64 value)
{
}
#endif

/**
 * erdt_ioremap_checked - Helper to safely map physical MMIO region
 * @base: Physical address base to map
 * @size: Size in 4KB pages
 * @desc: Description for logging in case of error
 */
static inline void __iomem *erdt_ioremap_checked(phys_addr_t base, u32 size,
						 const char *desc)
{
	void __iomem *addr = ioremap(base, size << 12);

	if (!addr)
		pr_err("ERDT: Failed to map %s at phys addr %#llx (size: %u KB)\n",
		       desc, (unsigned long long)base, size << 12);
	return addr;
}

/**
 * erdt_iounmap_domain - Unmap all MMIO regions for a domain
 * @domain: Domain info structure containing MMIO mappings
 */
static inline void erdt_iounmap_domain(struct erdt_domain_info *domain)
{
	for (int i = 0; i < ERDT_MMIO_MAX; i++) {
		if (domain->base[i]) {
			iounmap(domain->base[i]);
			domain->base[i] = NULL;
		}
	}
}

/**
 * erdt_enable_mmio - Map and initialize MMIO register bases for all RMDD domains
 *
 * This function maps optional memory-mapped I/O regions for MARC, CMRC, MMRC,
 * and control registers as described in the ERDT sub-tables. It also sets the
 * opt-in bit for MMIO mode operation.
 */
static int __init erdt_enable_mmio(void)
{
	struct erdt_domain_info *d;
	unsigned long index;

	xa_for_each(&erdt_domain_xa, index, d) {
		struct {
		enum erdt_mmio_type type;
		phys_addr_t base;
		u32 size;
		const char *desc;
		} mmios[] = {
			{
				ERDT_MMIO_RMDD_CREG,
				d->rmdd->creg_base,
				d->rmdd->creg_size,
				"RMDD ctrl base"
			},
			{
				ERDT_MMIO_CMRC_BASE,
				d->cmrc->cmt_reg_base,
				d->cmrc->cmt_reg_size,
				"CMRC base"
			},
			{
				ERDT_MMIO_MMRC_BASE,
				d->mmrc->reg_base,
				d->mmrc->reg_size,
				"MMRC base"
			},
			{
				ERDT_MMIO_MARC_OPT,
				d->marc->reg_base_opt,
				d->marc->mba_reg_size,
				"MARC opt"
			},
		};

		/* Memory Map IO regions for all sub-tables */
		for (int i = 0; i < ARRAY_SIZE(mmios); i++) {
			d->base[mmios[i].type] = erdt_ioremap_checked(mmios[i].base,
								      mmios[i].size,
								      mmios[i].desc);
			if (!d->base[mmios[i].type])
				goto cleanup;
		}

		erdt_update_mmio_qword(d->base[ERDT_MMIO_RMDD_CREG],
				       RMDD_CREG_MASK, RMDD_CREG_ENABLE);
		continue;

cleanup:
		pr_err("MMIO mapping failed at rmdd index %lu, starting full cleanup\n",
		       index);
		erdt_iounmap_domain(d);

		while (index-- > 0) {
			struct erdt_domain_info *prev = xa_load(&erdt_domain_xa, index);

			if (!prev)
				continue;

			for (int i = 0; i < ERDT_MMIO_MAX; i++) {
				if (prev->base[i]) {
					if (i == ERDT_MMIO_RMDD_CREG)
						erdt_update_mmio_qword(prev->base[i],
								       RMDD_CREG_MASK,
								       RMDD_CREG_DISABLE);
					iounmap(prev->base[i]);
					prev->base[i] = NULL;
				}
			}
		}

		return -EIO;
	}

	return 0;
}

/**
 * walk_erdt_subtables - Iterate over ERDT subtables and invoke a handler
 * @table:   Pointer to the full ACPI ERDT table
 * @type:    Subtable type to match (e.g., ACPI_ERDT_TYPE_RMDD)
 * @handler: Callback function to invoke for each matching subtable
 *
 * Wwalks through all subtables contained in the ACPI ERDT table and
 * invokes the provided handler callback for each subtable that
 * matches the specified type.
 *
 * The ERDT table is assumed to follow the standard ACPI subtable layout,
 * with each subtable having a type and length. The traversal stops once
 * the end of the table is reached.
 *
 * Return: Always returns 0. Errors from the handler are logged but ignored.
 */
static __init int walk_erdt_subtables(struct acpi_table_erdt *table,
				      enum acpi_erdt_type type,
				      int (*handler)(struct acpi_subtbl_hdr_16 *))
{
	struct acpi_subtbl_hdr_16 *subtbl;
	void *table_end = (void *)table + table->header.length - 1;
	int ret;

	subtbl = (void *)table + sizeof(struct acpi_table_erdt);

	while ((void *)subtbl < table_end) {
		if (subtbl->type == type) {
			ret = handler(subtbl);
			if (ret)
				pr_warn("ERDT: handler for subtable type %d failed with %d\n",
					type, ret);
		}

		subtbl = (void *)subtbl + subtbl->length;
	}

	return 0;
}

/*
 * enumerate_erdt_table - Store pointer to ERDT and begin domain parsing
 */
static __init int enumerate_erdt_table(struct acpi_table_header *table_hdr)
{
	int ret;

	erdt_info.erdt = (struct acpi_table_erdt *)table_hdr;

	ret = walk_erdt_subtables(erdt_info.erdt,
				  ACPI_ERDT_TYPE_RMDD,
				  parse_rmdd_entry);

	return ret;
}

/*
 * erdt_init - ACPI ERDT table initialization entry point
 */
__init int erdt_init(void)
{
	int ret;

	ret = acpi_table_parse(ACPI_SIG_ERDT, enumerate_erdt_table);
	if (!ret)
		erdt_enable_mmio();

	return 0;
}
