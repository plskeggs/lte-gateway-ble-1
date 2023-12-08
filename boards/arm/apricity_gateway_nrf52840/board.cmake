# Copyright (c) 2020 Nordic Semiconductor ASA.
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic

board_runner_args(nrfjprog "--softreset")
board_runner_args(jlink "--device=nRF52832_xxAA" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
