// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025, Intel Corporation.
 *
 * Memory Range and Region Mapping (MRRM) structure
 *
 * Parse and report the platform's MRRM table in /sys.
 */

#define pr_fmt(fmt) "acpi/mrrm: " fmt

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/memory-tiers.h>

/* Default assume one memory region covering all system memory, per the spec */
static int max_mem_region = 1;

/* Access for use by resctrl file system */
int acpi_mrrm_max_mem_region(void)
{
	return max_mem_region;
}

struct mrrm_mem_range_entry {
	u64 base;
	u64 length;
	int node;
	u8  local_region_id;
	u8  remote_region_id;
};

struct mrrm_tier_info {
	char *name; /*lowercase name*/
	char *cname;/*uppercase name*/
	int region;
};

static struct mrrm_mem_range_entry *mrrm_mem_range_entry;
static u32 mrrm_mem_entry_num;

static int get_node_num(struct mrrm_mem_range_entry *e)
{
	unsigned int nid;

	for_each_online_node(nid) {
		for (int z = 0; z < MAX_NR_ZONES; z++) {
			struct zone *zone = NODE_DATA(nid)->node_zones + z;

			if (!populated_zone(zone))
				continue;
			if (zone_intersects(zone, PHYS_PFN(e->base), PHYS_PFN(e->length)))
				return zone_to_nid(zone);
		}
	}

	return -ENOENT;
}

#define MRRM_TIER_ENTRY(n, cn)	\
	{.name = n, .cname = cn}

static struct mrrm_tier_info mt[] = {
	MRRM_TIER_ENTRY("local", "LOCAL"),
	MRRM_TIER_ENTRY("tier2_local", "TIER2_LOCAL"),
	MRRM_TIER_ENTRY("tier3_local", "TIER3_LOCAL"),
	MRRM_TIER_ENTRY("tier4_local", "TIER4_LOCAL"),
	MRRM_TIER_ENTRY("remote", "REMOTE"),
	MRRM_TIER_ENTRY("tier2_remote", "TIER2_REMOTE"),
	MRRM_TIER_ENTRY("tier3_remote", "TIER3_REMOTE"),
	MRRM_TIER_ENTRY("tier4_remote", "TIER4_REMOTE"),
};

/*
 * Given the region id, return the display name
 * for this region, meanwhile save the corresponding
 * region id in global mt array for future query.
 * @region:	the region ID
 * @cap:	return capital format name
 */
char *get_mrrm_region_name(int region, bool cap)
{
	struct mrrm_mem_range_entry *mre = NULL;
	int loc = -1, nid, offset, level;
	/*
	 * 1. Figure out if the region id is local or
	 *    remote by iterating the mrrm entries to
	 *    to find the mrrm entry whose local_id/remote_id
	 *    matches the region id.
	 * 2. Find the corresponding node of this mrrm entry.
	 * 3. Find the tier level based on the node's abstract
	 *    distance from HMAT(via memory tier subsystem).
	 */
	for (int i = 0; i < mrrm_mem_entry_num; i++) {
		mre = mrrm_mem_range_entry + i;
		if (region == mre->local_region_id) {
			loc = 0;
			break;
		} else if (region == mre->remote_region_id) {
			loc = 1;
			break;
		}
	}

	if (!mre || loc == -1)
		return NULL;

	nid = get_node_num(mre);
	if (nid < 0)
		return NULL;

	level = get_numa_tier_level(nid);
	/* currently 4 regions are supported */
	offset = loc * 4 + level;

	/* save the region id */
	mt[offset].region = region;

	/* return the user-friendly name */
	if (cap)
		return mt[offset].cname;
	else
		return mt[offset].name;
}

/* get the region id for the region's user friendly name */
int get_region_id_from_name(char *name, bool cap)
{
	int len = ARRAY_SIZE(mt), i;

	for (i = 0; i < len; i++) {
		char *p;

		if (!cap)
			p = mt[i].name;
		else
			p = mt[i].cname;
		if (!strcmp(p, name))
			return mt[i].region;
	}

	return -1;
}

static __init int acpi_parse_mrrm(struct acpi_table_header *table)
{
	struct acpi_mrrm_mem_range_entry *mre_entry;
	struct acpi_table_mrrm *mrrm;
	void *mre, *mrrm_end;
	int mre_count = 0;

	mrrm = (struct acpi_table_mrrm *)table;
	if (!mrrm)
		return -ENODEV;

	if (mrrm->flags & ACPI_MRRM_FLAGS_REGION_ASSIGNMENT_OS)
		return -EOPNOTSUPP;

	mrrm_end = (void *)mrrm + mrrm->header.length - 1;
	mre = (void *)mrrm + sizeof(struct acpi_table_mrrm);
	while (mre < mrrm_end) {
		mre_entry = mre;
		mre_count++;
		mre += mre_entry->header.length;
	}
	if (!mre_count) {
		pr_info(FW_BUG "No ranges listed in MRRM table\n");
		return -EINVAL;
	}

	mrrm_mem_range_entry = kmalloc_array(mre_count, sizeof(*mrrm_mem_range_entry),
					     GFP_KERNEL | __GFP_ZERO);
	if (!mrrm_mem_range_entry)
		return -ENOMEM;

	mre = (void *)mrrm + sizeof(struct acpi_table_mrrm);
	while (mre < mrrm_end) {
		struct mrrm_mem_range_entry *e;

		mre_entry = mre;
		e = mrrm_mem_range_entry + mrrm_mem_entry_num;

		e->base = mre_entry->addr_base;
		e->length = mre_entry->addr_len;
		e->node = get_node_num(e);

		if (mre_entry->region_id_flags & ACPI_MRRM_VALID_REGION_ID_FLAGS_LOCAL)
			e->local_region_id = mre_entry->local_region_id;
		else
			e->local_region_id = -1;
		if (mre_entry->region_id_flags & ACPI_MRRM_VALID_REGION_ID_FLAGS_REMOTE)
			e->remote_region_id = mre_entry->remote_region_id;
		else
			e->remote_region_id = -1;

		mrrm_mem_entry_num++;
		mre += mre_entry->header.length;
	}

	max_mem_region = mrrm->max_mem_region;

	return 0;
}

#define RANGE_ATTR(name, fmt)						\
static ssize_t name##_show(struct kobject *kobj,			\
			  struct kobj_attribute *attr, char *buf)	\
{									\
	struct mrrm_mem_range_entry *mre;				\
	const char *kname = kobject_name(kobj);				\
	int n, ret;							\
									\
	ret = kstrtoint(kname + 5, 10, &n);				\
	if (ret)							\
		return ret;						\
									\
	mre = mrrm_mem_range_entry + n;					\
									\
	return sysfs_emit(buf, fmt, mre->name);				\
}									\
static struct kobj_attribute name##_attr = __ATTR_RO(name)

RANGE_ATTR(base, "0x%llx\n");
RANGE_ATTR(length, "0x%llx\n");
RANGE_ATTR(node, "%d\n");
RANGE_ATTR(local_region_id, "%d\n");
RANGE_ATTR(remote_region_id, "%d\n");

static struct attribute *memory_range_attrs[] = {
	&base_attr.attr,
	&length_attr.attr,
	&node_attr.attr,
	&local_region_id_attr.attr,
	&remote_region_id_attr.attr,
	NULL
};

ATTRIBUTE_GROUPS(memory_range);

static __init int add_boot_memory_ranges(void)
{
	struct kobject *pkobj, *kobj;
	int ret = -EINVAL;
	char *name;

	pkobj = kobject_create_and_add("memory_ranges", acpi_kobj);

	for (int i = 0; i < mrrm_mem_entry_num; i++) {
		name = kasprintf(GFP_KERNEL, "range%d", i);
		if (!name) {
			ret = -ENOMEM;
			break;
		}

		kobj = kobject_create_and_add(name, pkobj);

		ret = sysfs_create_groups(kobj, memory_range_groups);
		if (ret)
			return ret;
	}

	return ret;
}

static __init int mrrm_init(void)
{
	int ret;

	ret = acpi_table_parse(ACPI_SIG_MRRM, acpi_parse_mrrm);
	if (ret < 0)
		return ret;

	return add_boot_memory_ranges();
}
device_initcall(mrrm_init);
