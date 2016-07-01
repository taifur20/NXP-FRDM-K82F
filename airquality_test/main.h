/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_adc16.h"
#include "fsl_clock.h"
#include "fsl_cmp.h"
#include "fsl_cmt.h"
#include "fsl_common.h"
#include "fsl_crc.h"
#include "fsl_dac.h"
#include "fsl_debug_console.h"
#include "fsl_dmamux.h"
#include "fsl_dspi.h"
#include "fsl_dspi_edma.h"
#include "fsl_edma.h"
#include "fsl_ewm.h"
#include "fsl_flash.h"
#include "fsl_flexbus.h"
#include "fsl_flexio.h"
#include "fsl_flexio_camera.h"
#include "fsl_flexio_camera_edma.h"
#include "fsl_flexio_i2c_master.h"
#include "fsl_flexio_i2s.h"
#include "fsl_flexio_i2s_edma.h"
#include "fsl_flexio_spi.h"
#include "fsl_flexio_spi_edma.h"
#include "fsl_flexio_uart.h"
#include "fsl_flexio_uart_edma.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "fsl_i2c_edma.h"
#include "fsl_llwu.h"
#include "fsl_lmem_cache.h"
#include "fsl_lptmr.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_ltc.h"
#include "fsl_ltc_edma.h"
#include "fsl_mpu.h"
#include "fsl_notifier.h"
#include "fsl_pdb.h"
#include "fsl_pit.h"
#include "fsl_pmc.h"
#include "fsl_port.h"
#include "fsl_qspi.h"
#include "fsl_qspi_edma.h"
#include "fsl_rcm.h"
#include "fsl_rtc.h"
#include "fsl_sai.h"
#include "fsl_sai_edma.h"
#include "fsl_sdhc.h"
#include "fsl_sdramc.h"
#include "fsl_sim.h"
#include "fsl_smartcard.h"
#include "fsl_smartcard_emvsim.h"
#include "fsl_smartcard_phy_emvsim.h"
#include "fsl_smartcard_phy_ncn8025.h"
#include "fsl_smc.h"
#include "fsl_tpm.h"
#include "fsl_trng.h"
#include "fsl_tsi_v4.h"
#include "fsl_vref.h"
#include "fsl_wdog.h"
 
#include "fsl_device_registers.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
