/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_RESCTRL_ERDT_H
#define _ASM_X86_RESCTRL_ERDT_H

#include <linux/resctrl.h>
#include <linux/acpi.h>

struct erdt_domain_info {
	struct acpi_erdt_rmdd *rmdd;
	struct acpi_erdt_cacd *cacd;
	struct acpi_erdt_cmrc *cmrc;
	struct acpi_erdt_mmrc *mmrc;
	struct acpi_erdt_marc *marc;
};

struct erdt_table_info {
	/* true if ERDT table is present and valid */
	bool available;
	struct acpi_table_erdt *erdt;
};

#endif /* _ASM_X86_RESCTRL_ERDT_H */
