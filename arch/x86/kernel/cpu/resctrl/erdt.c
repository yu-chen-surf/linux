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
#include <asm/cpu.h>
#include <asm/apic.h>
#include <asm/cpu_device_id.h>
#include "internal.h"
#include "erdt.h"

/* Global variable to hold ERDT ACPI table information for later processing */
static struct erdt_table_info erdt_info;
static DEFINE_XARRAY(erdt_domain_xa); /* Indexed by L3 cache ID */

#define RDT_CTRL_LEGACY_MODE   BIT_ULL(2)
#define VALID_VERSION 1
#define UNAVAILABLE_COUNTER	BIT_ULL(63)
static u32 valid_subtbl_mask;

#define FIXPOINT_LOW_BITS	16
/*
 * erdt_enabled - Check if the ERDT table is present and enabled
 */
bool erdt_enabled(void)
{
	return erdt_info.available;
}

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

/*
 * erdt_read_l3_occupancy - Read L3 occupancy count for a given RMID
 * @d:    Pointer to the ERDT domain info
 * @rmid: Resource Monitoring ID to read occupancy for
 *
 * Calculates the MMIO address using clump and stride information
 * from the CMRC ACPI structure and reads the L3 cache occupancy
 * count for the given RMID. The raw value is scaled using the
 * up_scale factor provided by firmware.
 *
 * Return: 0 for success, error code for other cases.
 */
static int erdt_read_l3_occupancy(struct erdt_domain_info *d, int rmid,
				  u64 *val)
{
	struct acpi_erdt_cmrc *cmrc;
	void __iomem *vaddr;
	u16 clump_size, stride_size;
	u64 l3_cmt_count;

	cmrc = d->cmrc;
	if (!cmrc)
		return -EIO;

	clump_size = cmrc->clump_size;
	stride_size = cmrc->clump_stride;

	/*
	 * MMIO_ADDRESS_for_RMID# = CMRC Base +
	 *   (RMID / ClumpSize) * Stride +
	 *   (RMID % ClumpSize) * 8
	 */
	vaddr = d->base[ERDT_MMIO_CMRC_BASE] +
		(rmid / clump_size) * stride_size +
		(rmid % clump_size) * 8;

	l3_cmt_count = readq(vaddr);
	if (l3_cmt_count & UNAVAILABLE_COUNTER)
		return -EINVAL;

	*val = l3_cmt_count * cmrc->up_scale;

	return 0;
}

static u64 apply_correction_factor(u64 val, u32 factor)
{
	if (!factor)
		return val;

	return ((val * factor) >> FIXPOINT_LOW_BITS);
}

static int erdt_read_region_mbm(struct rdt_domain_hdr *hdr,
				struct erdt_domain_info *d, int rmid,
				int eventid, u64 *val)
{
	u64 blk_rmid, blk_offset, mbm_rmid_count = 0, chunks = 0;
	int region_idx = eventid - QOS_L3_MBM_R0_EVENT_ID;
	int corr_factor_len, corr_factor = 0;
	struct rdt_hw_l3_mon_domain *hw_dom;
	struct acpi_erdt_mmrc *mmrc = NULL;
	struct rdt_l3_mon_domain *mon_dom;
	struct arch_mbm_state *am;
	void __iomem *vaddr;

	mmrc = d->mmrc;
	if (!mmrc)
		return -EIO;

	if (!domain_header_is_valid(hdr, RESCTRL_MON_DOMAIN, RDT_RESOURCE_L3))
		return -EIO;

	mon_dom = container_of(hdr, struct rdt_l3_mon_domain, hdr);
	hw_dom = resctrl_to_arch_mon_dom(mon_dom);

	/*
	 * Block_to_locate_RMID# = floor((RMID# % 32) / 8) x 4 x 4096B;
	 * Offset_within_this_Block = (floor(((RMID#/32)x8)+RMID#%8) x
	 * 8B)+(Region# x 2048B);
	 * MMIO_ADDRESS_for_RMID#_Region# =
	 * MBM Register Block Base Address + Block_to_locate_RMID# +
	 * Offset_within_this_Block;
	 */
	blk_rmid = ((rmid % 32) / 8) * 4 * 4096;
	blk_offset = ((rmid / 32) * 8 + (rmid % 8)) * 8 + region_idx * 2048;
	vaddr = d->base[ERDT_MMIO_MMRC_BASE] + blk_rmid + blk_offset;

	mbm_rmid_count = readq(vaddr);

	/* verify if the data is valid */
	if (mbm_rmid_count & UNAVAILABLE_COUNTER)
		return -EINVAL;

	corr_factor_len = mmrc->corr_factor_list_len;
	if (corr_factor_len) {
		if (corr_factor_len == 1)
			corr_factor = mmrc->corr_factor_list[0];
		else
			corr_factor = mmrc->corr_factor_list[rmid];
	}

	am = get_arch_mbm_state(hw_dom, rmid, eventid);
	if (am) {
		am->chunks += mbm_overflow_count(am->prev_mon_val, mbm_rmid_count,
						mmrc->counter_width);

		chunks = apply_correction_factor(am->chunks, corr_factor);
		am->prev_mon_val = mbm_rmid_count;
	}

	*val = chunks * mmrc->up_scale;

	return 0;
}

/*
 * erdt_mon_read - Read monitoring data for a given domain and RMID
 * @hdr:    Domain header
 * @ev_id:  Monitoring event ID (e.g. QOS_L3_OCCUP_EVENT_ID)
 * @rmid:   Resource Monitoring ID for which to read the data
 * @val:    Store the read data
 *
 * Looks up the domain by domid and dispatches the read request
 * to the appropriate helper based on the event type.
 * Currently supports only L3 occupancy monitoring.
 *
 * Return 0 on succeed, error code otherwise.
 */
int erdt_mon_read(struct rdt_domain_hdr *hdr, int ev_id, int rmid,
		  u64 *val)
{
	struct erdt_domain_info *d;

	d = xa_load(&erdt_domain_xa, hdr->id);
	if (!d)
		return -EIO;

	if (ev_id == QOS_L3_OCCUP_EVENT_ID)
		return erdt_read_l3_occupancy(d, rmid, val);

	if (rmbm_event(ev_id))
		return erdt_read_region_mbm(hdr, d, rmid, ev_id, val);

	return 0;
}

/*
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
	int cpu, cache_id = -1, tmp;
	struct cacheinfo *ci;

	for (int i = 0; i < num_ids; i++) {
		cpu = lookup_logical_cpu_by_x2apicid(cacd->X2APICIDS[i]);
		if (cpu == -1) {
			pr_warn(FW_BUG "Unknown x2apicid 0x%x\n", cacd->X2APICIDS[i]);

			return -1;
		}

		if (!cpu_online(cpu))
			continue;

		tmp = get_cpu_cacheinfo_id(cpu, RESCTRL_L3_CACHE);
		if (tmp == -1) {
			pr_warn(FW_BUG "Can not find L3 cache id for CPU%d\n", cpu);
			return -1;
		}

		if (cache_id == -1)
			cache_id = tmp;

		if (tmp != cache_id) {
			pr_warn(FW_BUG "CACD references multiple L3 cache instances\n");
			return -1;
		}
	}

	/*
	 * Check if CACD lists all CPUs in the LLC domain.
	 */
	ci = get_cpu_cacheinfo_level(cpu, RESCTRL_L3_CACHE);
	if (!ci || num_ids != cpumask_weight(&ci->shared_cpu_map)) {
		pr_warn(FW_BUG "CACD does not list all the CPUs in L3 domain\n");
		return -1;
	}

	return cache_id;
}


static void region_aware_enable(void __iomem *addr, bool enable)
{
	u64 rdt_ctrl = readq(addr);

	if (enable)
		rdt_ctrl &= ~RDT_CTRL_LEGACY_MODE;
	else
		rdt_ctrl |= RDT_CTRL_LEGACY_MODE;

	writeq(rdt_ctrl, addr);
}

static void __iomem *erdt_ioremap_checked(phys_addr_t base, u32 size,
					  const char *desc)
{
	void __iomem *addr = ioremap(base, size << 12);

	if (!addr)
		pr_err("ERDT: Failed to map %s at phys addr %#llx (size: %u KB)\n",
		       desc, (unsigned long long)base, size);
	return addr;
}

static void erdt_iounmap_domain(struct erdt_domain_info *domain)
{
	for (int i = 0; i < ERDT_MMIO_MAX; i++) {
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

static bool cacd_init(struct erdt_domain_info *d, struct acpi_subtbl_hdr_16 *subtbl,
		      int *l3_cache_id)
{
	*l3_cache_id = get_l3_cache_id_from_cacd((struct acpi_erdt_cacd *)subtbl);

	return *l3_cache_id != -1;
}

static bool cmrc_init(struct erdt_domain_info *d, struct acpi_subtbl_hdr_16 *subtbl)
{
	struct acpi_erdt_cmrc *cmrc = (struct acpi_erdt_cmrc *)subtbl;

	if (cmrc->index_fn != VALID_VERSION) {
		pr_info("Unknown CMRC index function %d\n", cmrc->index_fn);
		return false;
	}

	d->cmrc = cmrc;
	d->base[ERDT_MMIO_CMRC_BASE] = erdt_ioremap_checked(cmrc->cmt_reg_base,
							    cmrc->cmt_reg_size, "CMRC base");

	return !!d->base[ERDT_MMIO_CMRC_BASE];
}

static bool mmrc_init(struct erdt_domain_info *d, struct acpi_subtbl_hdr_16 *subtbl)
{
	struct acpi_erdt_mmrc *mmrc = (struct acpi_erdt_mmrc *)subtbl;

	if (mmrc->index_fn != VALID_VERSION) {
		pr_info("Unknown MMRC index function %d\n", mmrc->index_fn);
		return false;
	}

	d->mmrc = mmrc;
	d->base[ERDT_MMIO_MMRC_BASE] = erdt_ioremap_checked(mmrc->reg_base,
							    mmrc->reg_size, "CMRC base");

	return !!d->base[ERDT_MMIO_MMRC_BASE];
}

static bool marc_init(struct erdt_domain_info *d, struct acpi_subtbl_hdr_16 *subtbl)
{
	struct acpi_erdt_marc *marc = (struct acpi_erdt_marc *)subtbl;

	if (marc->index_fn != VALID_VERSION) {
		pr_info("Unknown MARC index function %d\n", marc->index_fn);
		return false;
	}

	d->marc = marc;

	d->base[ERDT_MMIO_MARC_OPT] = erdt_ioremap_checked(marc->reg_base_opt,
							   marc->mba_reg_size, "MARC OPT base");
	if (!d->base[ERDT_MMIO_MARC_OPT])
		return false;

	d->base[ERDT_MMIO_MARC_MIN] = erdt_ioremap_checked(marc->reg_base_min,
							   marc->mba_reg_size, "MARC MIN base");
	if (!d->base[ERDT_MMIO_MARC_MIN])
		return false;

	d->base[ERDT_MMIO_MARC_MAX] = erdt_ioremap_checked(marc->reg_base_max,
							   marc->mba_reg_size, "MARC MAX base");
	if (!d->base[ERDT_MMIO_MARC_MAX])
		return false;

	return true;
}

static __init bool parse_rmdd_entry(struct acpi_subtbl_hdr_16 *rmdd_hdr)
{
	struct acpi_erdt_rmdd *rmdd = (struct acpi_erdt_rmdd *)rmdd_hdr;
	struct erdt_domain_info *domain_info;
	struct acpi_subtbl_hdr_16 *subtbl;
	int l3_cache_id = -1;
	u32 subtbl_mask = 0;
	void *rmdd_end;
	void *ptr;

	/* Quietly ignore non-CPU-based L3 domains (bit 0 set) */
	if (!(rmdd->flags & 0x1))
		return true;

	domain_info = kzalloc(sizeof(*domain_info), GFP_KERNEL);
	if (!domain_info)
		return false;

	domain_info->rmdd = rmdd;
	domain_info->base[ERDT_MMIO_RMDD_CREG] = erdt_ioremap_checked(rmdd->creg_base, rmdd->creg_size,
								      "RMDD ctrl base");
	if (!domain_info->base[ERDT_MMIO_RMDD_CREG])
		goto cleanup;

	rmdd_end = (void *)rmdd + rmdd->header.length;

	/* Iterate through all sub-structures inside this RMDD block */
	for (subtbl = (void *)rmdd + sizeof(*rmdd);
	     (void *)subtbl < rmdd_end;
	     subtbl = (void *)subtbl + subtbl->length) {
		switch (subtbl->type) {
		case ACPI_ERDT_TYPE_CACD:
			if (cacd_init(domain_info, subtbl, &l3_cache_id))
				subtbl_mask |= BIT(ACPI_ERDT_TYPE_CACD);
			break;
		case ACPI_ERDT_TYPE_CMRC:
			if (cmrc_init(domain_info, subtbl))
				subtbl_mask |= BIT(ACPI_ERDT_TYPE_CMRC);
			break;
		case ACPI_ERDT_TYPE_MMRC:
			if (mmrc_init(domain_info, subtbl))
				subtbl_mask |= BIT(ACPI_ERDT_TYPE_MMRC);
			break;
		case ACPI_ERDT_TYPE_MARC:
			if (marc_init(domain_info, subtbl))
				subtbl_mask |= BIT(ACPI_ERDT_TYPE_MARC);
			break;
		default:
			break;
		}
	}

	if (l3_cache_id == -1) {
		pr_info("ERDT: Failed to resolve L3 cache ID for RMDD domain %d\n",
			rmdd->domain_id);

		goto cleanup;
	}

	if (!valid_subtbl_mask) {
		valid_subtbl_mask = subtbl_mask;
	} else if (subtbl_mask != valid_subtbl_mask) {
		pr_info(FW_BUG "Mismatch domain\n");
		goto cleanup;
	}

	ptr = xa_store(&erdt_domain_xa, l3_cache_id, domain_info, GFP_KERNEL);
	if (xa_is_err(ptr)) {
		pr_info("ERDT: Failed to store domain info for RMDD domain %d\n",
			rmdd->domain_id);
		goto cleanup;
	}

	region_aware_enable(domain_info->base[ERDT_MMIO_RMDD_CREG], true);
	return true;

cleanup:
	cleanup_one_domain(domain_info);
	return false;
}

static void erdt_cleanup(void)
{
	struct erdt_domain_info *d;
	unsigned long index;

	xa_for_each(&erdt_domain_xa, index, d)
		cleanup_one_domain(d);
	xa_destroy(&erdt_domain_xa);
}

/*
 * enumerate_erdt_table - Store pointer to ERDT and begin domain parsing
 */
static __init int enumerate_erdt_table(struct acpi_table_header *table_hdr)
{
	struct acpi_table_erdt *erdt = (struct acpi_table_erdt *)table_hdr;
	struct acpi_subtbl_hdr_16 *subtbl;
	void *table_end;

	if (erdt->header.revision != VALID_VERSION) {
		pr_info("Unknown ERDT table revision %d\n", erdt->header.revision);
		return -EINVAL;
	}

	subtbl = (void *)erdt + sizeof(struct acpi_table_erdt);
	table_end = (void *)erdt + erdt->header.length - 1;

	while ((void *)subtbl < table_end) {
		if (subtbl->type == ACPI_ERDT_TYPE_RMDD)
			if (!parse_rmdd_entry(subtbl))
				goto cleanup;

		subtbl = (void *)subtbl + subtbl->length;
	}

	return 0;

cleanup:
	erdt_cleanup();
	return -EINVAL;
}

/*
 * erdt_init - ACPI ERDT table initialization entry point
 */
int __init erdt_init(void)
{
	return acpi_table_parse(ACPI_SIG_ERDT, enumerate_erdt_table);
}

void __exit erdt_exit(void)
{
	erdt_cleanup();
}
