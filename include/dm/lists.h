/*
 * Copyright (c) 2013 Google, Inc
 *
 * (C) Copyright 2012
 * Pavel Herrmann <morpheus.ibis@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _DM_LISTS_H_
#define _DM_LISTS_H_

#include <dm/uclass-id.h>

/**
 * lists_driver_lookup_name() - Return u_boot_driver corresponding to name
 *
 * This function returns a pointer to a driver given its name. This is used
 * for binding a driver given its name and platdata.
 *
 * @name: Name of driver to look up
 * @return pointer to driver, or NULL if not found
 */
struct driver *lists_driver_lookup_name(const char *name);

/**
 * lists_uclass_lookup() - Return uclass_driver based on ID of the class
 * id:		ID of the class
 *
 * This function returns the pointer to uclass_driver, which is the class's
 * base structure based on the ID of the class. Returns NULL on error.
 */
struct uclass_driver *lists_uclass_lookup(enum uclass_id id);

/**
 * lists_bind_drivers() - search for and bind all drivers to parent
 *
 * This searches the U_BOOT_DEVICE() structures and creates new devices for
 * each one. The devices will have @parent as their parent.
 *
 * @parent: parent driver (root)
 * @early_only: If true, bind only drivers with the DM_INIT_F flag. If false
 * bind all drivers.
 */
int lists_bind_drivers(struct udevice *parent);

/**
 * lists_bind_fdt() - bind a device tree node
 *
 * This creates a new device bound to the given device tree node, with
 * @parent as its parent.
 *
 * @parent: parent driver (root)
 * @blob: device tree blob
 * @offset: offset of this device tree node
 */
int lists_bind_fdt(struct udevice *parent, const void *blob, int offset);

#endif
