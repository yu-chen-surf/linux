// SPDX-License-Identifier: GPL-2.0-only
/*
 * Enhanced Resource Director Technology (ERDT)
 *
 * Copyright (C) 2026 Intel Corporation
 *
 */

#define pr_fmt(fmt)     "resctrl: " fmt

#include <linux/acpi.h>
#include <linux/overflow.h>
#include <linux/resctrl.h>
#include <linux/sizes.h>

#include <asm/apic.h>

#include "internal.h"

static LIST_HEAD(domain_info_list);

/* True when the ERDT ACPI table describes at least one domain with at least one CPU. */
static bool erdt_enabled;

#define ERDT_VALID_VERSION		1
#define RMDD_FLAG_CPU_L3_DOMAIN		BIT(0)

/* Bitmask of valid sub-tables found in the first RMDD, used to ensure all RMDDs match. */
static u32 valid_subtbl_mask;

/* Domain ID of the first RMDD that established @valid_subtbl_mask, for diagnostics. */
static u16 first_rmdd_domain_id;

static int erdt_max_rmid;

int erdt_get_max_rmid(void)
{
	return erdt_max_rmid;
}

static void __iomem *erdt_ioremap(phys_addr_t base, u32 num_pages, const char *desc)
{
	void __iomem *addr;
	size_t size;

	if (check_mul_overflow(num_pages, SZ_4K, &size))
		return NULL;

	addr = ioremap(base, size);
	if (!addr)
		pr_warn(FW_BUG "ERDT: Failed to map %s at phys addr %pa (size: %u pages)\n",
		       desc, &base, num_pages);

	return addr;
}

static void erdt_iounmap_domain(struct erdt_domain_info *domain)
{
	for (int i = 0; i < ERDT_MMIO_NUM_TYPES; i++) {
		if (domain->base[i]) {
			iounmap(domain->base[i]);
			domain->base[i] = NULL;
		}
	}
}

static void cleanup_one_domain(struct erdt_domain_info *d)
{
	erdt_iounmap_domain(d);
	kfree(d);
}

/*
 * Save CACD information for this RMDD:
 * convert the X2APIC to CPU and save them in a mask.
 */
static __init int cacd_init(struct acpi_subtbl_hdr_16 *subtbl,
			    struct erdt_domain_info *domain_info)
{
	struct acpi_erdt_cacd *cacd = (struct acpi_erdt_cacd *)subtbl;
	int num_ids, cpu;

	if (cacd->header.length < struct_size(cacd, X2APICIDS, 1)) {
		pr_warn(FW_BUG "Invalid x2apicid CACD table\n");
		return -EIO;
	}

	num_ids = (cacd->header.length - sizeof(*cacd)) / sizeof(cacd->X2APICIDS[0]);

	for (int i = 0; i < num_ids; i++) {
		cpu = topo_lookup_cpuid(cacd->X2APICIDS[i]);
		if (cpu < 0) {
			pr_warn(FW_BUG "Unknown x2apicid 0x%x\n", cacd->X2APICIDS[i]);
			return -EIO;
		}

		cpumask_set_cpu(cpu, &domain_info->cpu_mask);
	}

	return 0;
}

static inline struct acpi_subtbl_hdr_16 *rmdd_subtbl(struct acpi_erdt_rmdd *rmdd)
{
	return (void *)rmdd + sizeof(*rmdd);
}

static inline struct acpi_subtbl_hdr_16 *next_subtbl(struct acpi_subtbl_hdr_16 *subtbl)
{
	return (void *)subtbl + subtbl->length;
}

static inline bool subtbl_valid(void *end, struct acpi_subtbl_hdr_16 *subtbl)
{
	/* Ensure the header is within bounds before dereferencing it. */
	if ((void *)subtbl + sizeof(*subtbl) > end)
		return false;

	/* A sub-table must be at least as large as its header. */
	if (subtbl->length < sizeof(*subtbl))
		return false;

	/* The entire sub-table (including body) must fit within the parent. */
	if ((void *)subtbl + subtbl->length > end)
		return false;

	return true;
}

static __init bool parse_rmdd_table(struct acpi_subtbl_hdr_16 *rmdd_hdr)
{
	struct erdt_domain_info *domain_info;
	struct acpi_subtbl_hdr_16 *subtbl;
	struct acpi_erdt_rmdd *rmdd;
	u32 subtbl_mask = 0;

	if (rmdd_hdr->length < sizeof(*rmdd)) {
		pr_warn(FW_BUG "Invalid RMDD length %u bytes\n", rmdd_hdr->length);
		return false;
	}

	rmdd = (struct acpi_erdt_rmdd *)rmdd_hdr;

	/* Quietly ignore non-CPU-based L3 domains */
	if (!(rmdd->flags & RMDD_FLAG_CPU_L3_DOMAIN))
		return true;

	domain_info = kzalloc_obj(*domain_info, GFP_KERNEL);
	if (!domain_info)
		return false;

	domain_info->dom_id = -1;

	domain_info->base[ERDT_MMIO_RMDD_CREG] =
		erdt_ioremap(rmdd->creg_base, rmdd->creg_size, "RMDD ctrl base");
	if (!domain_info->base[ERDT_MMIO_RMDD_CREG])
		goto cleanup;

	for (subtbl = rmdd_subtbl(rmdd);
	     subtbl_valid((void *)rmdd + rmdd->header.length, subtbl);
	     subtbl = next_subtbl(subtbl)) {
		switch (subtbl->type) {
		/* An RMDD table has one or more CACD sub-table(s) */
		case ACPI_ERDT_TYPE_CACD:
			if (cacd_init(subtbl, domain_info))
				goto cleanup;

			subtbl_mask |= BIT(ACPI_ERDT_TYPE_CACD);
			break;
		default:
			break;
		}
	}

	if (!subtbl_mask)
		goto cleanup;

	/*
	 * Require all RMDDs to support same set of sub-tables
	 */
	if (!valid_subtbl_mask) {
		valid_subtbl_mask = subtbl_mask;
		first_rmdd_domain_id = rmdd->domain_id;
	} else if (subtbl_mask != valid_subtbl_mask) {
		pr_warn(FW_BUG "RMDD %u sub-table set does not match the first RMDD %u\n",
			rmdd->domain_id, first_rmdd_domain_id);
		goto cleanup;
	}

	if (!rmdd->max_rmid) {
		pr_warn(FW_BUG "Unreasonable RMDD max_rmid %u\n", rmdd->max_rmid);
		goto cleanup;
	}
	domain_info->max_rmid = rmdd->max_rmid;

	if (!erdt_max_rmid)
		erdt_max_rmid = rmdd->max_rmid;
	else
		erdt_max_rmid = min_t(int, erdt_max_rmid, rmdd->max_rmid);

	list_add(&domain_info->entry, &domain_info_list);

	return true;

cleanup:
	cleanup_one_domain(domain_info);
	return false;
}

bool erdt_cpu_valid(int cpu)
{
	struct erdt_domain_info *d;
	int dom_id;

	if (!erdt_enabled)
		return true;

	dom_id = get_cpu_cacheinfo_id(cpu, RESCTRL_L3_CACHE);
	if (dom_id < 0)
		return true;

	/*
	 * Find the erdt_domain_info that contains this CPU,
	 * check if all CPUs in erdt_domain_info's cpumask
	 * have the same id(L3 id).
	 *
	 * For example, erdt_domain_info reports:
	 * domain0: CPU0, CPU2, domain1: CPU1, CPU3
	 * rdt_domain_hdr reports:
	 * domain0: CPU0, CPU1, domain1: CPU2, CPU3
	 * As a result, CPU1, CPU2 should not be covered by resctrl.
	 */
	list_for_each_entry(d, &domain_info_list, entry) {

		if (cpumask_test_cpu(cpu, &d->cpu_mask)) {
			if (d->dom_id == -1) {
				d->dom_id = dom_id;
			} else if (d->dom_id != dom_id) {
				pr_warn(FW_BUG "CPU%d's id=%d not equal to CACD domain(%*pbl) id=%d, skip this CPU\n",
					cpu, dom_id, cpumask_pr_args(&d->cpu_mask), d->dom_id);

				return false;
			}

			return true;
		}
	}

	pr_warn(FW_BUG "Cannot find CACD domain for CPU%d\n", cpu);
	return false;
}

/*
 * Associate ERDT table information with this domain.
 */
void erdt_l3_mon_domain_setup(int cpu, struct rdt_domain_hdr *hdr)
{
	struct rdt_hw_l3_mon_domain *hw_dom;
	struct erdt_domain_info *d;

	if (!erdt_enabled)
		return;

	hw_dom = resctrl_to_arch_mon_dom(container_of(hdr, struct rdt_l3_mon_domain, hdr));

	list_for_each_entry(d, &domain_info_list, entry) {
		if (cpumask_test_cpu(cpu, &d->cpu_mask)) {
			/* Assign the ERDT information to hw_dom */
			if (!hw_dom->d_info)
				hw_dom->d_info = d;
			return;
		}
	}
}

void erdt_exit(void)
{
	struct erdt_domain_info *d, *tmp;

	list_for_each_entry_safe(d, tmp, &domain_info_list, entry) {
		list_del(&d->entry);
		cleanup_one_domain(d);
	}
	erdt_enabled = false;
	valid_subtbl_mask = 0;
	first_rmdd_domain_id = 0;
}

static __init int enumerate_erdt_table(struct acpi_table_header *table_hdr)
{
	struct acpi_table_erdt *erdt = (struct acpi_table_erdt *)table_hdr;
	struct acpi_subtbl_hdr_16 *subtbl;
	void *table_end;

	if (erdt->header.revision != ERDT_VALID_VERSION) {
		pr_info("Unsupported ERDT table revision %u (expected %u)\n",
			erdt->header.revision, ERDT_VALID_VERSION);
		return -EINVAL;
	}

	if (erdt->header.length < sizeof(*erdt)) {
		pr_warn(FW_BUG "ERDT: Invalid table length %u bytes\n", erdt->header.length);
		return -EINVAL;
	}

	subtbl = (void *)erdt + sizeof(struct acpi_table_erdt);
	table_end = (void *)erdt + erdt->header.length;

	while (subtbl_valid(table_end, subtbl)) {
		if (subtbl->type == ACPI_ERDT_TYPE_RMDD &&
		    !parse_rmdd_table(subtbl))
			goto cleanup;

		subtbl = next_subtbl(subtbl);
	}

	if (list_empty(&domain_info_list))
		goto cleanup;

	erdt_enabled = true;

	return 0;

cleanup:
	erdt_exit();
	return -EINVAL;
}

int __init erdt_init(void)
{
	return acpi_table_parse(ACPI_SIG_ERDT, enumerate_erdt_table);
}
