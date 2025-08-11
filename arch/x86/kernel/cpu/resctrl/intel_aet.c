// SPDX-License-Identifier: GPL-2.0-only
/*
 * Resource Director Technology(RDT)
 * - Intel Application Energy Telemetry
 *
 * Copyright (C) 2025 Intel Corporation
 *
 * Author:
 *    Tony Luck <tony.luck@intel.com>
 */

#define pr_fmt(fmt)   "resctrl: " fmt

#include <linux/cleanup.h>
#include <linux/cpu.h>
#include <linux/intel_vsec.h>
#include <linux/resctrl.h>
#include <linux/slab.h>

#include "internal.h"

/**
 * struct event_group - All information about a group of telemetry events.
 * @pfg:		Points to the aggregated telemetry space information
 *			within the OOBMSM driver that contains data for all
 *			telemetry regions.
 * @guid:		Unique number per XML description file.
 * @mmio_size:		Number of bytes of MMIO registers for this group.
 */
struct event_group {
	/* Data fields for additional structures to manage this group. */
	struct pmt_feature_group	*pfg;

	/* Remaining fields initialized from XML file. */
	u32				guid;
	size_t				mmio_size;
};

#define XML_MMIO_SIZE(num_rmids, num_events, num_extra_status) \
		      (((num_rmids) * (num_events) + (num_extra_status)) * sizeof(u64))

/*
 * Link: https://github.com/intel/Intel-PMT
 * File: xml/CWF/OOBMSM/RMID-ENERGY/cwf_aggregator.xml
 */
static struct event_group energy_0x26696143 = {
	.guid		= 0x26696143,
	.mmio_size	= XML_MMIO_SIZE(576, 2, 3),
};

/*
 * Link: https://github.com/intel/Intel-PMT
 * File: xml/CWF/OOBMSM/RMID-PERF/cwf_aggregator.xml
 */
static struct event_group perf_0x26557651 = {
	.guid		= 0x26557651,
	.mmio_size	= XML_MMIO_SIZE(576, 7, 3),
};

static struct event_group *known_energy_event_groups[] = {
	&energy_0x26696143,
};

static struct event_group *known_perf_event_groups[] = {
	&perf_0x26557651,
};

static bool skip_this_region(struct telemetry_region *tr, struct event_group *e)
{
	if (tr->guid != e->guid)
		return true;
	if (tr->plat_info.package_id >= topology_max_packages()) {
		pr_warn_once("Bad package %d in guid 0x%x\n", tr->plat_info.package_id,
			     tr->guid);
		return true;
	}
	if (tr->size != e->mmio_size) {
		pr_warn_once("MMIO space %zu wrong size for guid 0x%x\n", tr->size, e->guid);
		return true;
	}

	return false;
}

/*
 * Discover events from one pmt_feature_group.
 * 1) Count how many usable telemetry regions per package.
 * 2...) To be continued.
 */
static int discover_events(struct event_group *e, struct pmt_feature_group *p)
{
	int *pkgcounts __free(kfree) = NULL;
	struct telemetry_region *tr;
	int num_pkgs;

	num_pkgs = topology_max_packages();

	/* Get per-package counts of telemetry regions for this event group */
	for (int i = 0; i < p->count; i++) {
		tr = &p->regions[i];
		if (skip_this_region(tr, e))
			continue;

		if (!pkgcounts) {
			pkgcounts = kcalloc(num_pkgs, sizeof(*pkgcounts), GFP_KERNEL);
			if (!pkgcounts)
				return -ENOMEM;
		}
		pkgcounts[tr->plat_info.package_id]++;
	}

	if (!pkgcounts)
		return -ENODEV;

	return 0;
}

DEFINE_FREE(intel_pmt_put_feature_group, struct pmt_feature_group *,
		if (!IS_ERR_OR_NULL(_T))
			intel_pmt_put_feature_group(_T))

/*
 * Make a request to the INTEL_PMT_DISCOVERY driver for the
 * pmt_feature_group for a specific feature. If there is
 * one the returned structure has an array of telemetry_region
 * structures. Each describes one telemetry aggregator.
 * Try to use every telemetry aggregator with a known guid.
 */
static bool get_pmt_feature(enum pmt_feature_id feature, struct event_group **evgs,
			    unsigned int num_evg)
{
	struct pmt_feature_group *p __free(intel_pmt_put_feature_group) = NULL;
	struct event_group **peg;
	int ret;

	p = intel_pmt_get_regions_by_feature(feature);

	if (IS_ERR_OR_NULL(p))
		return false;

	for (peg = evgs; peg < &evgs[num_evg]; peg++) {
		ret = discover_events(*peg, p);
		if (!ret) {
			(*peg)->pfg = no_free_ptr(p);
			return true;
		}
	}

	return false;
}

/*
 * Ask OOBMSM discovery driver for all the RMID based telemetry groups
 * that it supports.
 */
bool intel_aet_get_events(void)
{
	bool ret1, ret2;

	ret1 = get_pmt_feature(FEATURE_PER_RMID_ENERGY_TELEM,
			       known_energy_event_groups,
			       ARRAY_SIZE(known_energy_event_groups));
	ret2 = get_pmt_feature(FEATURE_PER_RMID_PERF_TELEM,
			       known_perf_event_groups,
			       ARRAY_SIZE(known_perf_event_groups));

	return ret1 || ret2;
}

void __exit intel_aet_exit(void)
{
}
