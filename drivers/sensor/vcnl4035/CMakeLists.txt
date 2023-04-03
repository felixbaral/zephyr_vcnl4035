#
# Copyright (c) 2022 Circuit Dojo LLC
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_VCNL4035 vcnl4035.c)
zephyr_library_sources_ifdef(CONFIG_VCNL4035_TRIGGER vcnl4035_trigger.c)
