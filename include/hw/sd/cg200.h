/*
 * D-Broad CG200-V2 dual SD/SDIO host controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SD_CG200_H
#define HW_SD_CG200_H

#include "hw/sd/sdhci.h"
#include "qom/object.h"

#define TYPE_CG200 "cg200"
OBJECT_DECLARE_SIMPLE_TYPE(CG200State, CG200)

SDBus *cg200_get_bus(CG200State *s, unsigned int slot);

#endif
