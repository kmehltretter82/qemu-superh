/*
 * Renesas R8A66597 USB host controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_USB_HCD_R8A66597_H
#define HW_USB_HCD_R8A66597_H

#include "hw/usb/usb.h"
#include "qom/object.h"

#define TYPE_R8A66597_USB_HOST "r8a66597-usb-host"
OBJECT_DECLARE_SIMPLE_TYPE(R8A66597State, R8A66597_USB_HOST)

USBBus *r8a66597_usb_bus(R8A66597State *s);

#endif
