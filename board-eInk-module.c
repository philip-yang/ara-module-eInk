/**
 * Copyright (c) 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Philip Yang <philipy@bsquare.com>
 */

#include <syslog.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/device.h>
#include <nuttx/device_table.h>
#include <nuttx/device_hid.h>

#include <arch/tsb/gpio.h>

#ifdef CONFIG_DEVICE_CORE
static struct device devices[] = {
    {
        .type           = DEVICE_TYPE_HID_HW,
        .name           = "hid_Button",
        .desc           = "Button HID Driver",
        .id             = 0,
    },
};

static struct device_table bdb_device_table = {
    .device = devices,
    .device_count = ARRAY_SIZE(devices),
};

static void bdb_driver_register(void)
{
    extern struct device_driver hid_button_driver;
    device_register_driver(&hid_button_driver);
};
#endif

void ara_module_early_init(void)
{
    tsb_gpio_register(NULL);
}

void ara_module_init(void)
{
    lowsyslog("E-Ink Module init\n");
#ifdef CONFIG_DEVICE_CORE
    device_table_register(&bdb_device_table);
    bdb_driver_register();
#endif
}
