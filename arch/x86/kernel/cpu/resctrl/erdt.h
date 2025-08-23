/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_RESCTRL_ERDT_H
#define _ASM_X86_RESCTRL_ERDT_H

#include <linux/resctrl.h>
#include <linux/acpi.h>

#define RMDD_CREG_MASK 0b100
#define RMDD_CREG_ENABLE 0
#define RMDD_CREG_DISABLE 1

enum erdt_mmio_type {
	ERDT_MMIO_RMDD_CREG,
	ERDT_MMIO_CMRC_BASE,
	ERDT_MMIO_MMRC_BASE,
	ERDT_MMIO_MARC_OPT,
	ERDT_MMIO_MARC_MIN,
	ERDT_MMIO_MARC_MAX,
	ERDT_MMIO_MAX
};

struct erdt_domain_info {
	struct acpi_erdt_rmdd *rmdd;
	struct acpi_erdt_cacd *cacd;
	struct acpi_erdt_cmrc *cmrc;
	struct acpi_erdt_mmrc *mmrc;
	struct acpi_erdt_marc *marc;
	/* MMIO  address */
	void __iomem *base[ERDT_MMIO_MAX];
};

struct erdt_table_info {
	/* true if ERDT table is present and valid */
	bool available;
	struct acpi_table_erdt *erdt;
};

#endif /* _ASM_X86_RESCTRL_ERDT_H */
