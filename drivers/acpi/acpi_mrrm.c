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

/*
 * Figure what does region mean on this node,
 * append the information to str.
 * return the bytes written to the str.
 */
int acpi_mrrm_fill_info(char *str, int region, int node, int domid)
{
	struct mrrm_mem_range_entry *e;
	int i, len = 0;
	bool local = false, remote = false;

	for (i = 0; i < mrrm_mem_entry_num; i++) {

		e = mrrm_mem_range_entry + i;
		if (e->node != node)
			continue;

		if (e->local_region_id == region) {
			len += sprintf(str + len, "[dom%d][mem %#010Lx-%#010Lx] ",
				       domid, e->base, (e->base + e->length));

			if (!local)
				local = true;
		} else if (e->remote_region_id == region) {
			len += sprintf(str + len, "[dom%d][mem %#010Lx-%#010Lx] ",
				       domid, e->base, (e->base + e->length));

			if (!remote)
				remote = true;
		}
	}

	if (local)
		len += sprintf(str + len, " [local socket access]");
	else if (remote)
		len += sprintf(str + len, " [remote socket access]");

	return len;
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

#if 1
unsigned char mrrm_table[] = {
0x4d, 0x52, 0x52, 0x4d, 0xe0, 0x0, 0x0, 0x0, 0x1, 0x8e, 0x49, 0x4e, 0x54, 0x45, 0x4c, 0x0,  // MRRM......INTEL.
0x49, 0x4e, 0x54, 0x45, 0x4c, 0x20, 0x49, 0x44, 0x2, 0x0, 0x0, 0x0, 0x49, 0x4e, 0x54, 0x4c,  // INTEL ID....INTL
0x28, 0x6, 0x23, 0x20, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // (.# ............
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // ................
0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // .. .............
0x0, 0x0, 0x0, 0xd0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // ................
0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0,  // .. .............
0x0, 0x0, 0x0, 0x0, 0x7f, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,  // ................
0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x0,  // .. .............
0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x1, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,  // ................
0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x2, 0x0, 0x0,  // .. .............
0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x0, 0x0, 0x1, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0,  // ................
0x0, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0x4, 0x0, 0x0,  // .. .............
0x0, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x0, 0x1, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0 // ................
};
#endif

static __init int mrrm_init(void)
{
	int ret;

	ret = acpi_parse_mrrm((struct acpi_table_header *)mrrm_table);
	if (ret < 0)
		return ret;

	return add_boot_memory_ranges();
}
device_initcall(mrrm_init);
