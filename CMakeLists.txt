# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_include_directories(include)
zephyr_library_sources_ifdef(CONFIG_VCNL4035_TRIGGER drivers/vcnl4035)
