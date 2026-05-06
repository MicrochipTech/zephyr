/*
 * Copyright (c) 2026 Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/ccc.h>
#include <zephyr/logging/log.h>
#include "crc8.h"

LOG_MODULE_REGISTER(i3c_host, CONFIG_I3C_TEST_HOST_LOG_LEVEL);


/*
 * i3c_xec_debug.c
 *
 * Debug / diagnostic helpers for the XEC I3C controller.
 *
 * Provides:
 *   - Device Address Table (DAT) pointer decoding helpers
 *   - DAT entry read helper
 *   - Full host-side register dump routine (sec_host block excluded)
 *
 * All output is emitted via LOG_DBG().
 */


/* ====================================================================== */
/*  Register Block Definition                                              */
/* ====================================================================== */

struct i3c_xec_regs {
    volatile uint32_t dev_ctrl;                  /* 0x00 */
    volatile uint32_t dev_addr;                  /* 0x04 */
    volatile uint32_t hw_capability;             /* 0x08 */
    volatile uint32_t cmd_queue_port;            /* 0x0C */
    volatile uint32_t resp_queue_port;           /* 0x10 */
    volatile uint32_t rx_tx_port;                /* 0x14 */
    volatile uint32_t ibi_queue_sts_data;        /* 0x18 */
    volatile uint32_t queue_thld_ctrl;           /* 0x1C */
    volatile uint32_t data_buf_thld_ctrl;        /* 0x20 */
    volatile uint32_t ibi_queue_ctrl;            /* 0x24 */
    volatile uint8_t  _rsvd0[(0x2CU - 0x28U)];
    volatile uint32_t ibi_MR_req_rej;            /* 0x2C */
    volatile uint32_t ibi_SIR_req_rej;           /* 0x30 */
    volatile uint32_t reset_ctrl;                /* 0x34 */
    volatile uint32_t slave_evt_sts;             /* 0x38 */
    volatile uint32_t intr_sts;                  /* 0x3C */
    volatile uint32_t intr_sts_en;               /* 0x40 */
    volatile uint32_t intr_sig_en;               /* 0x44 */
    volatile uint32_t intr_force;                /* 0x48 */
    volatile uint32_t queue_sts_lvl;             /* 0x4C */
    volatile uint32_t data_buf_sts_lvl;          /* 0x50 */
    volatile uint32_t present_state;             /* 0x54 */
    volatile uint32_t ccc_dev_sts;               /* 0x58 */
    volatile uint32_t dev_addr_tbl_ptr;          /* 0x5C */
    volatile uint32_t dev_char_tbl_ptr;          /* 0x60 */
    volatile uint8_t  _rsvd1[(0x6CU - 0x64U)];
    volatile uint32_t vendor_reg_ptr;            /* 0x6C */
    volatile uint32_t slv_mipi_id_val;           /* 0x70 */
    volatile uint32_t slv_pid_val;               /* 0x74 */
    volatile uint32_t slv_char_ctrl;             /* 0x78 */
    volatile uint32_t slv_max_len;               /* 0x7C */
    volatile uint32_t max_rd_turnaround;         /* 0x80 */
    volatile uint32_t max_data_speed;            /* 0x84 */
    volatile uint32_t slv_dbg_sts;               /* 0x88 */
    volatile uint32_t slv_intr_req;              /* 0x8C */
    volatile uint32_t slv_tsx_symbl_timing;      /* 0x90 */
    volatile uint32_t slv_intr_req_data;         /* 0x94 */
    volatile uint32_t slv_ibi_resp;              /* 0x98 */
    volatile uint32_t slv_nack_req;              /* 0x9C */
    volatile uint32_t slv_nack_conf;             /* 0xA0 */
    volatile uint32_t slv_inst_sts;              /* 0xA4 */
    volatile uint8_t  _rsvd3[(0xB0U - 0xA8U)];
    volatile uint32_t dev_ctrl_ext;              /* 0xB0 */
    volatile uint32_t scl_i3c_od_timing;         /* 0xB4 */
    volatile uint32_t scl_i3c_pp_timing;         /* 0xB8 */
    volatile uint32_t scl_i2c_fm_timing;         /* 0xBC */
    volatile uint32_t scl_i2c_fmp_timing;        /* 0xC0 */
    volatile uint8_t  _rsvd4[(0xC8U - 0xC4U)];
    volatile uint32_t scl_ext_lcnt_timing;       /* 0xC8 */
    volatile uint32_t scl_ext_term_lcnt_timing;  /* 0xCC */
    volatile uint32_t sda_hld_switch_dly_timing; /* 0xD0 */
    volatile uint32_t bus_free_avail_timing;     /* 0xD4 */
    volatile uint32_t bus_idle_timing;           /* 0xD8 */
    volatile uint32_t scl_low_mst_ext_timeout;   /* 0xDC */
    volatile uint32_t i3c_ver_id;                /* 0xE0 */
    volatile uint32_t i3c_ver_type;              /* 0xE4 */
    volatile uint32_t queue_size_capability;     /* 0xE8 */
    volatile uint8_t  _rsvd7[(0x200U - 0xECU)];
    volatile uint32_t dev_char_tbl1_loc1;        /* 0x200 */
    volatile uint32_t dev_char_tbl1_loc2;        /* 0x204 */
    volatile uint32_t dev_char_tbl1_loc3;        /* 0x208 */
    volatile uint32_t dev_char_tbl1_loc4;        /* 0x20C */
    volatile uint8_t  _rsvd8[(0x220U - 0x210U)];
    volatile uint32_t dev_addr_tbl1_loc1;        /* 0x220 */
    volatile uint8_t  _rsvd9[(0x300U - 0x224U)];
    volatile uint32_t host_config;               /* 0x300 */
    volatile uint32_t host_reset_ctrl;           /* 0x304 */
    volatile uint32_t host_dma_tx_timeout;       /* 0x308 */
    volatile uint32_t host_dma_rx_timeout;       /* 0x30C */
    volatile uint32_t host_stuck_sda_timeout;    /* 0x310 */
    volatile uint32_t host_status;               /* 0x314 */
    volatile uint32_t host_interrupt_en;         /* 0x318 */
    volatile uint8_t  _rsvd10[(0x3D0U - 0x31CU)];
    volatile uint32_t host_pad_test;             /* 0x3D0 */
    volatile uint32_t host_debug_0;              /* 0x3D4 */
    volatile uint32_t host_debug_1;              /* 0x3D8 */
    volatile uint8_t  _rsvd11[(0x400U - 0x3DCU)];
    volatile uint32_t sec_host_config0;          /* 0x400 */
    volatile uint32_t sec_host_reset_ctrl;       /* 0x404 */
    volatile uint32_t sec_host_dma_tx_timeout;   /* 0x408 */
    volatile uint32_t sec_host_dma_rx_timeout;   /* 0x40C */
    volatile uint32_t sec_host_stuck_sda_timeout;/* 0x410 */
    volatile uint32_t sec_host_status;           /* 0x414 */
    volatile uint32_t sec_host_interrupt_en;     /* 0x418 */
    volatile uint32_t sec_host_tgt_config0;      /* 0x420 */
    volatile uint32_t sec_host_tgt_config1;      /* 0x424 */
    volatile uint32_t sec_host_tgt_config2;      /* 0x428 */
    volatile uint32_t sec_host_tgt_config3;      /* 0x42C */
    volatile uint32_t sec_host_pad_test;         /* 0x4D0 */
    volatile uint32_t sec_host_debug_0;          /* 0x4D4 */
    volatile uint32_t sec_host_debug_1;          /* 0x4D8 */
    volatile uint32_t sec_host_debug_2;          /* 0x4DC */
};

/* ====================================================================== */
/*  Helper functions                                                       */
/* ====================================================================== */

/**
 * @brief Read the Device Address Table Pointer Register (0x5C)
 *
 * @param regs       Pointer to controller registers
 * @param start_addr Out: start offset of the device address table
 * @param depth      Out: depth (number of entries) of the device address table
 */
static void _i3c_dev_addr_table_ptr_get(struct i3c_xec_regs *regs,
                                        uint16_t *start_addr,
                                        uint16_t *depth)
{
    uint32_t val = regs->dev_addr_tbl_ptr;

    *start_addr = (uint16_t)(val & 0xFFFFU);
    *depth      = (uint16_t)((val >> 16) & 0xFFFFU);
}

/**
 * @brief Retrieve Device Address Table information
 *
 * @param regs       Pointer to controller registers
 * @param start_addr Out: start offset of the DAT
 * @param depth      Out: depth of the DAT
 */
void I3C_DAT_info_get(struct i3c_xec_regs *regs,
                      uint16_t *start_addr,
                      uint16_t *depth)
{
    _i3c_dev_addr_table_ptr_get(regs, start_addr, depth);
}

/**
 * @brief Read a single DAT entry
 *
 * @param regs      Pointer to controller registers
 * @param DAT_start Start offset of the DAT (from dev_addr_tbl_ptr)
 * @param DAT_idx   Index of the entry within the DAT
 * @return uint32_t Raw 32-bit DAT value
 */
uint32_t _i3c_DAT_read(struct i3c_xec_regs *regs,
                       uint16_t DAT_start,
                       uint8_t DAT_idx)
{
    volatile uint32_t *entry_addr =
        (volatile uint32_t *)((uintptr_t)regs + (uintptr_t)(DAT_start + (DAT_idx * 4U)));

    return *entry_addr;
}

/* ====================================================================== */
/*  Register dump routine                                                  */
/* ====================================================================== */

/**
 * @brief Dump all I3C host-side registers and first 4 DAT entries via LOG_DBG
 *
 * Does NOT dump the sec_host_* register block.
 *
 * @param regs Pointer to the base of the i3c_xec_regs register block
 */
void i3c_xec_dump_regs(struct i3c_xec_regs *regs)
{
    if (regs == NULL) {
        LOG_INF("i3c_xec_dump_regs: NULL register pointer");
        return;
    }

    LOG_INF("=== I3C XEC Register Dump ===");

    /* -------------------------------------------------------------- */
    /*  Core / Common I3C Registers                                    */
    /* -------------------------------------------------------------- */
    LOG_INF("[I3C] dev_ctrl                  (0x000) = 0x%08X", regs->dev_ctrl);
    LOG_INF("[I3C] dev_addr                  (0x004) = 0x%08X", regs->dev_addr);
    LOG_INF("[I3C] hw_capability             (0x008) = 0x%08X", regs->hw_capability);
    LOG_INF("[I3C] cmd_queue_port            (0x00C) = 0x%08X", regs->cmd_queue_port);
    LOG_INF("[I3C] resp_queue_port           (0x010) = 0x%08X", regs->resp_queue_port);
    LOG_INF("[I3C] rx_tx_port                (0x014) = 0x%08X", regs->rx_tx_port);
    LOG_INF("[I3C] ibi_queue_sts_data        (0x018) = 0x%08X", regs->ibi_queue_sts_data);
    LOG_INF("[I3C] queue_thld_ctrl           (0x01C) = 0x%08X", regs->queue_thld_ctrl);
    LOG_INF("[I3C] data_buf_thld_ctrl        (0x020) = 0x%08X", regs->data_buf_thld_ctrl);
    LOG_INF("[I3C] ibi_queue_ctrl            (0x024) = 0x%08X", regs->ibi_queue_ctrl);
    //LOG_INF("[I3C] ibi_MR_req_rej            (0x02C) = 0x%08X", regs->ibi_MR_req_rej);
    //LOG_INF("[I3C] ibi_SIR_req_rej           (0x030) = 0x%08X", regs->ibi_SIR_req_rej);
    LOG_INF("[I3C] reset_ctrl                (0x034) = 0x%08X", regs->reset_ctrl);
    //LOG_INF("[I3C] slave_evt_sts             (0x038) = 0x%08X", regs->slave_evt_sts);
    LOG_INF("[I3C] intr_sts                  (0x03C) = 0x%08X", regs->intr_sts);
    LOG_INF("[I3C] intr_sts_en               (0x040) = 0x%08X", regs->intr_sts_en);
    LOG_INF("[I3C] intr_sig_en               (0x044) = 0x%08X", regs->intr_sig_en);
    LOG_INF("[I3C] intr_force                (0x048) = 0x%08X", regs->intr_force);
    LOG_INF("[I3C] queue_sts_lvl             (0x04C) = 0x%08X", regs->queue_sts_lvl);
    LOG_INF("[I3C] data_buf_sts_lvl          (0x050) = 0x%08X", regs->data_buf_sts_lvl);
    LOG_INF("[I3C] present_state             (0x054) = 0x%08X", regs->present_state);
    //LOG_INF("[I3C] ccc_dev_sts               (0x058) = 0x%08X", regs->ccc_dev_sts);
    LOG_INF("[I3C] dev_addr_tbl_ptr          (0x05C) = 0x%08X", regs->dev_addr_tbl_ptr);
    LOG_INF("[I3C] dev_char_tbl_ptr          (0x060) = 0x%08X", regs->dev_char_tbl_ptr);
    LOG_INF("[I3C] vendor_reg_ptr            (0x06C) = 0x%08X", regs->vendor_reg_ptr);
    //LOG_INF("[I3C] slv_mipi_id_val           (0x070) = 0x%08X", regs->slv_mipi_id_val);
    //LOG_INF("[I3C] slv_pid_val               (0x074) = 0x%08X", regs->slv_pid_val);
    //LOG_INF("[I3C] slv_char_ctrl             (0x078) = 0x%08X", regs->slv_char_ctrl);
    //LOG_INF("[I3C] slv_max_len               (0x07C) = 0x%08X", regs->slv_max_len);
    //LOG_INF("[I3C] max_rd_turnaround         (0x080) = 0x%08X", regs->max_rd_turnaround);
    //LOG_INF("[I3C] max_data_speed            (0x084) = 0x%08X", regs->max_data_speed);
    //LOG_INF("[I3C] slv_dbg_sts               (0x088) = 0x%08X", regs->slv_dbg_sts);
    //LOG_INF("[I3C] slv_intr_req              (0x08C) = 0x%08X", regs->slv_intr_req);
    //LOG_INF("[I3C] slv_tsx_symbl_timing      (0x090) = 0x%08X", regs->slv_tsx_symbl_timing);
    //LOG_INF("[I3C] slv_intr_req_data         (0x094) = 0x%08X", regs->slv_intr_req_data);
    //LOG_INF("[I3C] slv_ibi_resp              (0x098) = 0x%08X", regs->slv_ibi_resp);
    //LOG_INF("[I3C] slv_nack_req              (0x09C) = 0x%08X", regs->slv_nack_req);
    //LOG_INF("[I3C] slv_nack_conf             (0x0A0) = 0x%08X", regs->slv_nack_conf);
    //LOG_INF("[I3C] slv_inst_sts              (0x0A4) = 0x%08X", regs->slv_inst_sts);
    LOG_INF("[I3C] dev_ctrl_ext              (0x0B0) = 0x%08X", regs->dev_ctrl_ext);
    LOG_INF("[I3C] scl_i3c_od_timing         (0x0B4) = 0x%08X", regs->scl_i3c_od_timing);
    LOG_INF("[I3C] scl_i3c_pp_timing         (0x0B8) = 0x%08X", regs->scl_i3c_pp_timing);
    LOG_INF("[I3C] scl_i2c_fm_timing         (0x0BC) = 0x%08X", regs->scl_i2c_fm_timing);
    LOG_INF("[I3C] scl_i2c_fmp_timing        (0x0C0) = 0x%08X", regs->scl_i2c_fmp_timing);
    LOG_INF("[I3C] scl_ext_lcnt_timing       (0x0C8) = 0x%08X", regs->scl_ext_lcnt_timing);
    LOG_INF("[I3C] scl_ext_term_lcnt_timing  (0x0CC) = 0x%08X", regs->scl_ext_term_lcnt_timing);
    LOG_INF("[I3C] sda_hld_switch_dly_timing (0x0D0) = 0x%08X", regs->sda_hld_switch_dly_timing);
    LOG_INF("[I3C] bus_free_avail_timing     (0x0D4) = 0x%08X", regs->bus_free_avail_timing);
    //LOG_INF("[I3C] bus_idle_timing           (0x0D8) = 0x%08X", regs->bus_idle_timing);
    LOG_INF("[I3C] scl_low_mst_ext_timeout   (0x0DC) = 0x%08X", regs->scl_low_mst_ext_timeout);
    LOG_INF("[I3C] i3c_ver_id                (0x0E0) = 0x%08X", regs->i3c_ver_id);
    LOG_INF("[I3C] i3c_ver_type              (0x0E4) = 0x%08X", regs->i3c_ver_type);
    LOG_INF("[I3C] queue_size_capability     (0x0E8) = 0x%08X", regs->queue_size_capability);

    /* -------------------------------------------------------------- */
    /*  Device Characteristic Table                                    */
    /* -------------------------------------------------------------- */
    LOG_INF("--- Device Char Table ---");
    LOG_INF("[I3C] dev_char_tbl1_loc1        (0x200) = 0x%08X", regs->dev_char_tbl1_loc1);
    LOG_INF("[I3C] dev_char_tbl1_loc2        (0x204) = 0x%08X", regs->dev_char_tbl1_loc2);
    LOG_INF("[I3C] dev_char_tbl1_loc3        (0x208) = 0x%08X", regs->dev_char_tbl1_loc3);
    LOG_INF("[I3C] dev_char_tbl1_loc4        (0x20C) = 0x%08X", regs->dev_char_tbl1_loc4);

    /* -------------------------------------------------------------- */
    /*  Device Address Table (raw struct location)                     */
    /* -------------------------------------------------------------- */
    LOG_INF("--- Device Address Table (raw) ---");
    LOG_INF("[I3C] dev_addr_tbl1_loc1        (0x220) = 0x%08X", regs->dev_addr_tbl1_loc1);

    /* -------------------------------------------------------------- */
    /*  Host Registers (0x300 - 0x3D8)                                 */
    /* -------------------------------------------------------------- */
    LOG_INF("--- Host Registers ---");
    LOG_INF("[HOST] host_config              (0x300) = 0x%08X", regs->host_config);
    LOG_INF("[HOST] host_reset_ctrl          (0x304) = 0x%08X", regs->host_reset_ctrl);
    LOG_INF("[HOST] host_dma_tx_timeout      (0x308) = 0x%08X", regs->host_dma_tx_timeout);
    LOG_INF("[HOST] host_dma_rx_timeout      (0x30C) = 0x%08X", regs->host_dma_rx_timeout);
    LOG_INF("[HOST] host_stuck_sda_timeout   (0x310) = 0x%08X", regs->host_stuck_sda_timeout);
    LOG_INF("[HOST] host_status              (0x314) = 0x%08X", regs->host_status);
    LOG_INF("[HOST] host_interrupt_en        (0x318) = 0x%08X", regs->host_interrupt_en);
    LOG_INF("[HOST] host_pad_test            (0x3D0) = 0x%08X", regs->host_pad_test);
    LOG_INF("[HOST] host_debug_0             (0x3D4) = 0x%08X", regs->host_debug_0);
    LOG_INF("[HOST] host_debug_1             (0x3D8) = 0x%08X", regs->host_debug_1);

    /* -------------------------------------------------------------- */
    /*  Device Address Table (DAT) - First 4 device entries            */
    /*                                                                 */
    /*  Fields per entry (DEV_ADDR_TABLE1_LOC1 spec):                  */
    /*    [31]    DEVICE           - 0=I3C, 1=I2C                      */
    /*    [30:29] DEV_NACK_RETRY   - NACK retry count                  */
    /*    [28:24] Reserved                                              */
    /*    [23:16] DEV_DYNAMIC_ADDR - Dynamic addr[22:16] + parity[23]  */
    /*    [15]    Reserved                                              */
    /*    [14]    MR_REJECT        - 0=Accept, 1=Reject MR             */
    /*    [13]    SIR_REJECT       - 0=Accept, 1=Reject SIR            */
    /*    [12]    IBI_WITH_DATA    - 0=No mandatory byte, 1=Has byte   */
    /*    [11]    IBI_PEC_EN       - 0=PEC disabled, 1=PEC enabled     */
    /*    [10:7]  Reserved                                              */
    /*    [6:0]   STATIC_ADDRESS   - Device Static Address             */
    /* -------------------------------------------------------------- */
    LOG_INF("--- DAT Entries (first 4 devices) ---");

    uint16_t DAT_start = 0U;
    uint16_t DAT_depth = 0U;
    uint8_t  num_entries;

    /* Retrieve DAT start offset and depth from dev_addr_tbl_ptr (0x5C) */
    I3C_DAT_info_get(regs, &DAT_start, &DAT_depth);

    LOG_INF("[DAT] DAT start_addr=0x%04X  depth=%u", DAT_start, DAT_depth);

    /* Clamp to the lesser of 4 or the actual DAT depth */
    num_entries = (DAT_depth < 4U) ? (uint8_t)DAT_depth : 4U;

    for (uint8_t i = 0U; i < num_entries; i++) {

        uint32_t dat_val = _i3c_DAT_read(regs, DAT_start, i);

        /* Parse individual bit fields */
        uint8_t device_type    = (uint8_t)((dat_val >> 31) & 0x1U);
        uint8_t nack_retry_cnt = (uint8_t)((dat_val >> 29) & 0x3U);
        uint8_t dyn_addr_raw   = (uint8_t)((dat_val >> 16) & 0xFFU); /* [23:16] */
        uint8_t dyn_addr_7bit  = (uint8_t)((dat_val >> 16) & 0x7FU); /* [22:16] */
        uint8_t dyn_addr_par   = (uint8_t)((dat_val >> 23) & 0x1U);  /* [23]    */
        uint8_t mr_reject      = (uint8_t)((dat_val >> 14) & 0x1U);
        uint8_t sir_reject     = (uint8_t)((dat_val >> 13) & 0x1U);
        uint8_t ibi_with_data  = (uint8_t)((dat_val >> 12) & 0x1U);
        uint8_t ibi_pec_en     = (uint8_t)((dat_val >> 11) & 0x1U);
        uint8_t static_addr    = (uint8_t)( dat_val        & 0x7FU);

        LOG_INF("[DAT] Entry[%u] raw=0x%08X", i, dat_val);
        LOG_INF("[DAT]   DEVICE           = %u (%s)",
                device_type, device_type ? "I2C" : "I3C");
        LOG_INF("[DAT]   DEV_NACK_RETRY   = %u", nack_retry_cnt);
        LOG_INF("[DAT]   DEV_DYNAMIC_ADDR = 0x%02X (7-bit=0x%02X parity=%u)",
                dyn_addr_raw, dyn_addr_7bit, dyn_addr_par);
        LOG_INF("[DAT]   MR_REJECT        = %u (%s)",
                mr_reject, mr_reject ? "Reject" : "Accept");
        LOG_INF("[DAT]   SIR_REJECT       = %u (%s)",
                sir_reject, sir_reject ? "Reject" : "Accept");
        LOG_INF("[DAT]   IBI_WITH_DATA    = %u (%s)",
                ibi_with_data ? 1U : 0U,
                ibi_with_data ? "Mandatory byte" : "No mandatory byte");
        LOG_INF("[DAT]   IBI_PEC_EN       = %u (%s)",
                ibi_pec_en, ibi_pec_en ? "PEC enabled" : "PEC disabled");
        LOG_INF("[DAT]   STATIC_ADDRESS   = 0x%02X", static_addr);
    }

    LOG_INF("=== I3C XEC Register Dump Complete ===");
}

#define I3C_BASE_ADDR	(0x40010800)



/* --------------------------------------------------------------------
 * Constants
 * -------------------------------------------------------------------- */
#define I3C_TEST_BUF_SIZE         1024U
#define I3C_TEST_MDB_TIR          0xAEU
#define I3C_TEST_TIMEOUT_MS       10000U
#define I3C_TEST_IBI_POLL_MS      1000U
#define I3C_TEST_HJ_TIMEOUT_MS    5000U
#define I3C_TEST_ECHO_DELAY_MS      10U
#define I3C_SCL_TYP_HZ            12500000U   /* I3C typical 12.5 MHz */

/* Event bits — must match target.c and loopback/main.c */
#define EVT_IBI_TIR_MDB_AE   BIT(0)   /* TIR IBI with MDB=0xAE */
#define EVT_IBI_ANY          BIT(1)   /* any IBI received */
#define EVT_IBI_HOTJOIN      BIT(2)   /* hot-join IBI received */
#define EVT_DATA_RECEIVED    BIT(3)   /* target data received (target side) */
#define EVT_TARGET_READY     BIT(4)   /* target registered (loopback sync) */

/* --------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------- */
__aligned(4) static uint8_t host_tx_buf[I3C_TEST_BUF_SIZE] = {
	0x58, 0x12, 0x59, 0x84, 0x5F, 0x7B, 0xC4, 0xBC, 0xD0, 0x8D, 0xF3, 0x46, 0x79, 0x18, 0x75,
	0xEC, 0xB6, 0x44, 0x43, 0x1C, 0xF4, 0xF7, 0xDA, 0xC7, 0xE8, 0x63, 0x64, 0x37, 0x62, 0xB4,
	0x13, 0xDA, 0x5F, 0x1A, 0x34, 0x98, 0x9D, 0xB4, 0x63, 0x90, 0x1E, 0xFD, 0x35, 0xD3, 0xDF,
	0x0E, 0x14, 0xD8, 0x6C, 0x49, 0xDB, 0x32, 0xF2, 0x7E, 0x7B, 0x89, 0x3C, 0xE8, 0x16, 0x1B,
	0xA0, 0x12, 0xD0, 0x53, 0xFF, 0xDC, 0xCC, 0xE7, 0xC1, 0x99, 0xDE, 0xEC, 0xD1, 0xFC, 0xFC,
	0x6D, 0x21, 0x9C, 0x98, 0x03, 0x98, 0x1D, 0xF6, 0xDB, 0xFD, 0xC2, 0x61, 0x99, 0xDB, 0x7F,
	0xA0, 0x9D, 0x75, 0xD1, 0x09, 0xE7, 0xF5, 0x8C, 0x5B, 0xE8, 0x4B, 0x91, 0x9B, 0xBD, 0x0A,
	0x43, 0x0E, 0xB3, 0x30, 0x05, 0x77, 0x18, 0xF4, 0xD0, 0x4C, 0x4E, 0xCE, 0x28, 0x3C, 0x05,
	0x6E, 0xDF, 0xE0, 0xF2, 0xBE, 0xC0, 0x50, 0x15, 0xE0, 0xCB, 0xE5, 0x5F, 0x01, 0x5F, 0x7C,
	0xA5, 0x0E, 0xB0, 0x1A, 0x17, 0x2C, 0x12, 0x6E, 0xB0, 0xED, 0xC6, 0xFF, 0x14, 0x47, 0x85,
	0x9D, 0x23, 0x7F, 0x99, 0x0D, 0x15, 0x99, 0xB5, 0xA9, 0xD4, 0x59, 0x98, 0xB1, 0xD6, 0xEE,
	0xD3, 0x4E, 0x48, 0x75, 0x63, 0xFF, 0x1B, 0x3A, 0x95, 0xE7, 0x83, 0x69, 0x74, 0xBA, 0x07,
	0x53, 0x07, 0xF5, 0x47, 0x4D, 0xE7, 0x4E, 0xFE, 0x35, 0xF9, 0x21, 0x95, 0x93, 0x8E, 0xB8,
	0xE8, 0xDD, 0xDE, 0x21, 0xE1, 0xA2, 0x6D, 0xC0, 0x55, 0xFE, 0x1F, 0xF4, 0x3B, 0xE8, 0x50,
	0x4D, 0x13, 0x8F, 0x02, 0x6B, 0xAA, 0x0C, 0x04, 0x43, 0xCD, 0xFC, 0x81, 0x8A, 0x6F, 0x75,
	0xD6, 0x97, 0xE8, 0x82, 0x6D, 0x39, 0x54, 0x84, 0x84, 0x83, 0x32, 0xA1, 0xD1, 0x22, 0x9E,
	0x6F, 0x32, 0xA8, 0xC8, 0xAB, 0x9A, 0x73, 0x1B, 0x2B, 0x85, 0xA8, 0x0B, 0x55, 0x1D, 0x17,
	0xD9, 0x2D, 0xDB, 0x9C, 0x77, 0x1E, 0x42, 0xDE, 0x94, 0x66, 0xBD, 0xF8, 0xD4, 0x0D, 0x2C,
	0x27, 0x3A, 0x1B, 0x47, 0x7E, 0x52, 0xBB, 0x2A, 0x13, 0x59, 0xC0, 0xDE, 0x1F, 0xBA, 0xFD,
	0xBC, 0x5A, 0x2F, 0x2C, 0x3D, 0xBD, 0x8D, 0x8A, 0xDD, 0x9E, 0x8B, 0x20, 0xB8, 0x80, 0xA2,
	0x1C, 0xB6, 0x55, 0xA8, 0x63, 0x6F, 0xD2, 0x39, 0x73, 0xC9, 0xD9, 0xA3, 0x27, 0x3D, 0x39,
	0x25, 0x38, 0x11, 0x94, 0xE6, 0x1D, 0xEE, 0xD2, 0x71, 0x8D, 0x90, 0x48, 0xEA, 0x96, 0x51,
	0x48, 0x3B, 0x1D, 0x58, 0xD2, 0x69, 0x28, 0x22, 0xE6, 0x87, 0x69, 0x45, 0x54, 0x16, 0x4E,
	0x56, 0xF4, 0xFC, 0x9B, 0x95, 0xFC, 0xFE, 0xA2, 0xD3, 0xC7, 0xF0, 0x91, 0x2E, 0x43, 0xE2,
	0x8D, 0xBE, 0x7F, 0x78, 0x02, 0x25, 0xF8, 0x36, 0x0B, 0x82, 0xB9, 0x74, 0xDE, 0x00, 0x99,
	0xC9, 0x03, 0x93, 0x06, 0xE1, 0x67, 0x95, 0xB6, 0x2B, 0xE3, 0x49, 0xE6, 0xC3, 0xD3, 0xCA,
	0xC0, 0x8A, 0x03, 0x50, 0x68, 0x59, 0xAF, 0x6C, 0x54, 0x0C, 0x6E, 0x73, 0xF2, 0x5D, 0x66,
	0x18, 0x35, 0xA2, 0x7D, 0xE9, 0xD4, 0xD7, 0xF7, 0xDE, 0x50, 0x06, 0x8F, 0xE3, 0x4F, 0x67,
	0xCA, 0x8F, 0xE7, 0xFD, 0x00, 0x22, 0xD5, 0xD1, 0x2D, 0x7C, 0x62, 0xC1, 0x49, 0xF2, 0x6B,
	0xE2, 0xF1, 0x1B, 0xD0, 0xF4, 0xEE, 0x3D, 0x7E, 0x25, 0x73, 0xCB, 0x35, 0xBA, 0x69, 0x2C,
	0x13, 0x71, 0x60, 0x01, 0xAA, 0xC0, 0x02, 0xDC, 0x17, 0xD1, 0xED, 0x71, 0xE9, 0x6D, 0x23,
	0x9D, 0x30, 0xD7, 0xED, 0xD5, 0xEE, 0xD7, 0xC5, 0xB7, 0x52, 0x95, 0x72, 0x8E, 0xB6, 0x5E,
	0xE3, 0x62, 0x5E, 0xA4, 0xC2, 0x54, 0xA3, 0xEC, 0x3B, 0xB9, 0x5D, 0xB9, 0x50, 0x9B, 0x7B,
	0x01, 0xB3, 0xA1, 0x7D, 0xAB, 0x36, 0xA5, 0x6F, 0xC4, 0x9E, 0xBE, 0x7B, 0xBA, 0xF1, 0x1C,
	0xAA, 0x9E, 0x20, 0x4D, 0x88, 0xD0, 0x19, 0xF2, 0xA0, 0xE9, 0xDC, 0xEF, 0x1A, 0x07, 0xDD,
	0x02, 0x80, 0x9A, 0xB9, 0x58, 0x06, 0x34, 0x9B, 0x34, 0xA1, 0x73, 0x20, 0x5B, 0x57, 0x8C,
	0x6A, 0xF5, 0x3C, 0x39, 0x45, 0x63, 0x40, 0x46, 0x42, 0x7B, 0x89, 0x05, 0x42, 0xB7, 0xAF,
	0x69, 0xF6, 0x78, 0x87, 0x29, 0xD7, 0x84, 0x90, 0x67, 0x82, 0x4B, 0xC5, 0x21, 0x3D, 0x36,
	0xBA, 0xA2, 0xEC, 0x2B, 0x4A, 0x6B, 0x43, 0x1D, 0x43, 0x49, 0xF6, 0x5E, 0x53, 0x8B, 0x96,
	0x97, 0xBE, 0xE7, 0x6C, 0x98, 0xDF, 0x2A, 0x55, 0x57, 0xE8, 0x2B, 0x23, 0x52, 0x94, 0xC5,
	0x5C, 0xE0, 0xF5, 0x5D, 0xA0, 0xE9, 0x87, 0x39, 0xBF, 0xF2, 0xC5, 0xD4, 0xB7, 0xC5, 0x9F,
	0x49, 0x86, 0x12, 0x05, 0x48, 0x9D, 0x00, 0x84, 0x68, 0xA8, 0x65, 0xE3, 0xAC, 0xE3, 0x07,
	0xAC, 0x59, 0x1F, 0x46, 0xF8, 0x8A, 0x48, 0xF4, 0xB3, 0x1F, 0x52, 0x1F, 0xD5, 0x17, 0x7A,
	0x3E, 0xAD, 0x1B, 0xF2, 0x07, 0xE3, 0x22, 0x19, 0x63, 0xEA, 0xFC, 0x62, 0xF8, 0x26, 0x86,
	0xE3, 0x5F, 0x82, 0x37, 0x73, 0x67, 0x22, 0xA5, 0x4E, 0xB4, 0xE8, 0x2D, 0xCC, 0x9B, 0xB9,
	0xAB, 0x2E, 0x71, 0x45, 0x01, 0x37, 0x4D, 0x7B, 0xC8, 0xBF, 0xD6, 0x00, 0x1C, 0x8C, 0x6A,
	0xF7, 0x0C, 0x5F, 0xC1, 0x05, 0xB9, 0x73, 0xC7, 0x33, 0x36, 0x23, 0x1E, 0x27, 0x73, 0xF7,
	0x93, 0x21, 0x27, 0x97, 0xD2, 0xD4, 0x7F, 0xCB, 0xAD, 0x27, 0x48, 0xCE, 0xCD, 0x88, 0x1E,
	0x6D, 0x6F, 0x93, 0xD8, 0x53, 0x14, 0x0E, 0x54, 0x17, 0xDA, 0xE6, 0xE5, 0x72, 0x08, 0xDD,
	0x9A, 0x70, 0x41, 0x0C, 0xC4, 0x80, 0x69, 0x17, 0xEA, 0xBC, 0xB8, 0x49, 0x2C, 0xD1, 0x3E,
	0xB2, 0xF3, 0xB2, 0x3D, 0xA8, 0x75, 0x75, 0x5F, 0x38, 0xD7, 0xC7, 0x70, 0xAD, 0x05, 0x3E,
	0xE7, 0x63, 0x47, 0x34, 0x66, 0xCD, 0xA9, 0x30, 0x05, 0x84, 0xD8, 0xA5, 0x9C, 0xF1, 0x37,
	0x97, 0x0F, 0xFA, 0xB2, 0x42, 0x52, 0x8F, 0x40, 0xF8, 0x33, 0x5D, 0xCA, 0x3B, 0x69, 0x25,
	0x94, 0x52, 0x64, 0xCE, 0x57, 0xAF, 0xEC, 0x72, 0x3E, 0xAF, 0x57, 0x14, 0xEA, 0xAD, 0xB8,
	0xB6, 0x12, 0x62, 0x6F, 0xE5, 0x5E, 0x35, 0x45, 0x17, 0xA1, 0xD0, 0x93, 0x36, 0xD9, 0x90,
	0x08, 0x0E, 0x4B, 0xE3, 0xF6, 0x0A, 0x38, 0xB2, 0x18, 0xE5, 0x2B, 0xC8, 0x74, 0xAA, 0xDE,
	0x19, 0xD7, 0x40, 0x61, 0x35, 0xAA, 0x4A, 0x6D, 0x51, 0x13, 0xE4, 0x5E, 0x72, 0x58, 0x56,
	0x1A, 0xEE, 0x9A, 0xAF, 0xE6, 0xB7, 0xBD, 0x26, 0xB9, 0x5B, 0x0A, 0x05, 0xC2, 0xDE, 0xBB,
	0xC0, 0xCD, 0x21, 0x2C, 0x13, 0x02, 0xAD, 0xF6, 0x54, 0x26, 0x12, 0xFD, 0x5F, 0x85, 0x9E,
	0x59, 0xFE, 0x37, 0x67, 0x3A, 0x10, 0x39, 0xE3, 0xE0, 0xED, 0xE2, 0xB4, 0xAD, 0x67, 0x38,
	0x05, 0x91, 0xD0, 0x6B, 0x4E, 0xFB, 0xF8, 0xB1, 0x5E, 0x25, 0xA4, 0x3C, 0xC0, 0x8B, 0x12,
	0xA1, 0x1D, 0xC5, 0xA1, 0x4F, 0x13, 0x6E, 0xF8, 0xFA, 0xF1, 0xD3, 0x95, 0x29, 0xD7, 0x44,
	0x58, 0x05, 0xE0, 0x8A, 0xE0, 0xAB, 0x81, 0x0B, 0xF5, 0x74, 0x72, 0x31, 0x9B, 0x1B, 0x34,
	0x60, 0xD3, 0xD1, 0xED, 0xA7, 0xCF, 0x34, 0x11, 0x80, 0x94, 0x19, 0x3C, 0x57, 0x95, 0xCC,
	0x07, 0x54, 0x3E, 0x0E, 0x61, 0x21, 0xB3, 0x0A, 0xD1, 0xF0, 0x23, 0x52, 0xBB, 0x37, 0xBC,
	0xBA, 0xD2, 0x5D, 0x5C, 0xA2, 0x6F, 0x8C, 0x95, 0x8F, 0x06, 0xD0, 0x71, 0x9B, 0x14, 0xC8,
	0xBC, 0x68, 0x7A, 0x2B, 0x25, 0xCE, 0xE8, 0x9E, 0xDD, 0x08, 0xE6, 0x9D, 0xB7, 0x38, 0x33,
	0x25, 0x6F, 0xE8, 0x5B, 0x3A, 0x13, 0xA1, 0xF4, 0xA1, 0x05, 0x45, 0x26, 0x68, 0x35, 0xDD,
	0xCB, 0x03, 0xC6, 0x34
};
__aligned(4) static uint8_t host_rx_buf[I3C_TEST_BUF_SIZE];

static struct k_event *host_events;   /* set from p2 in thread entry */

/* --------------------------------------------------------------------
 * PASS/FAIL reporting helpers
 * -------------------------------------------------------------------- */
#define TEST_REPORT_INIT()          \
    bool _group_pass = true;        \
    const char *_fail_reason = "(unknown)"

#define TEST_SUB_PASS(name)         \
    LOG_DBG("[step] %s PASS", (name))

#define TEST_SUB_FAIL(name, reason_str)         \
    do {                                        \
        LOG_DBG("[step] %s FAIL: %s",           \
                (name), (reason_str));          \
        if (_group_pass) {                      \
            _fail_reason = (reason_str);        \
        }                                       \
        _group_pass = false;                    \
    } while (0)

#define TEST_GROUP_RESULT(name)                         \
    do {                                                \
        if (_group_pass) {                              \
            LOG_INF("[PASS] %s", (name));               \
        } else {                                        \
            LOG_ERR("[FAIL] %s: %s",                    \
                    (name), _fail_reason);              \
        }                                               \
    } while (0)

/* --------------------------------------------------------------------
 * IBI callbacks
 * -------------------------------------------------------------------- */
static int i3c_host_ibi_cb(struct i3c_device_desc *target,
			    struct i3c_ibi_payload *payload)
{
	k_event_post(host_events, EVT_IBI_ANY);
	if (payload && payload->payload_len > 0 &&
	    payload->payload[0] == I3C_TEST_MDB_TIR) {
		k_event_post(host_events, EVT_IBI_TIR_MDB_AE);
	}
	return 0;
}

/* --------------------------------------------------------------------
 * CRC loopback verification helper
 * -------------------------------------------------------------------- */
static bool verify_loopback(const uint8_t *tx, const uint8_t *rx,
			     uint32_t len, const char *test_name)
{
#if CONFIG_I3C_TEST_CRC_CHECK
	crc8_t tx_crc = crc8_finalize(crc8_update(crc8_init(), tx, len));
	crc8_t rx_crc = crc8_finalize(crc8_update(crc8_init(), rx, len));

	if (tx_crc != rx_crc) {
		LOG_DBG("[step] %s FAIL: CRC mismatch tx=0x%02x rx=0x%02x",
			test_name, tx_crc, rx_crc);
		return false;
	}
#else
	for (uint32_t i = 0; i < len; i++) {
		if (tx[i] != rx[i]) {
			LOG_DBG("[step] %s FAIL: byte[%u] tx=0x%02x rx=0x%02x",
				test_name, i, tx[i], rx[i]);
			return false;
		}
	}
#endif
	return true;
}

/* ====================================================================
 * TC0: data_transfer
 * ==================================================================== */
static void run_tc0_data_transfer(const struct device *dev,
				  struct i3c_device_desc *target)
{
	TEST_REPORT_INIT();
	int ret;

	/* Step 1: write_1B — fire-and-forget single byte */
	host_tx_buf[0] = 0xA5U;
	ret = i3c_write(target, host_tx_buf, 1);
	if (ret != 0) {
		TEST_SUB_FAIL("write_1B", "i3c_write failed");
	} else {
		TEST_SUB_PASS("write_1B");
	}

	/* Steps 2–5: loopback at increasing lengths */
	static const struct {
		uint32_t    len;
		const char *name;
	} lb_steps[] = {
		{ 1U,    "loopback_1B"    },
		{ 32U,   "loopback_32B"   },
		{ 64U,  "loopback_64B"  },
		{ 128U,  "loopback_128B"  },
		//{ 1024U, "loopback_1024B" },
	};

	for (size_t i = 0; i < ARRAY_SIZE(lb_steps); i++) {
		uint32_t    len  = lb_steps[i].len;
		const char *name = lb_steps[i].name;

		for (uint32_t b = 0; b < len; b++) {
			host_tx_buf[b] = (uint8_t)(b & 0xFFU);
		}

		ret = i3c_write(target, host_tx_buf, len);
		if (ret != 0) {
			TEST_SUB_FAIL(name, "write failed");
			continue;
		}

		k_sleep(K_MSEC(I3C_TEST_ECHO_DELAY_MS));
		memset(host_rx_buf, 0, len);

		ret = i3c_read(target, host_rx_buf, len);
		if (ret != 0) {
			TEST_SUB_FAIL(name, "read failed");
			continue;
		}

		if (!verify_loopback(host_tx_buf, host_rx_buf, len, name)) {
			TEST_SUB_FAIL(name, "data mismatch");
		} else {
			TEST_SUB_PASS(name);
		}
	}

	/* Step 6: combined_xfer (write + repeated-START read) */
	for (uint32_t b = 0; b < 64U; b++) {
		host_tx_buf[b] = (uint8_t)(b & 0xFFU);
	}
	memset(host_rx_buf, 0, 64U);

	ret = i3c_write_read(target, host_tx_buf, 64U, host_rx_buf, 64U);
	if (ret != 0) {
		TEST_SUB_FAIL("combined_xfer", "i3c_write_read failed");
	} else if (!verify_loopback(host_tx_buf, host_rx_buf, 64U, "combined_xfer")) {
		TEST_SUB_FAIL("combined_xfer", "data mismatch");
	} else {
		TEST_SUB_PASS("combined_xfer");
	}

	TEST_GROUP_RESULT("data_transfer");
}

/* ====================================================================
 * TC1: ccc_commands
 * ==================================================================== */
static void run_tc1_ccc_commands(const struct device *dev,
				 struct i3c_device_desc *target)
{
	TEST_REPORT_INIT();
	int ret;

	/* Step 1: SETMWL / GETMWL */
	const struct i3c_ccc_mwl set_mwl = { .len = 128U };
	struct i3c_ccc_mwl get_mwl = { 0 };

	ret = i3c_ccc_do_setmwl(target, &set_mwl);
	if (ret != 0) {
		TEST_SUB_FAIL("setmwl_getmwl", "SETMWL failed");
	} else {
		ret = i3c_ccc_do_getmwl(target, &get_mwl);
		if (ret != 0) {
			TEST_SUB_FAIL("setmwl_getmwl", "GETMWL failed");
		} else if (get_mwl.len != 128U) {
			TEST_SUB_FAIL("setmwl_getmwl", "MWL not accepted");
		} else {
			TEST_SUB_PASS("setmwl_getmwl");
		}
	}

	/* Step 2: SETMRL / GETMRL */
	const struct i3c_ccc_mrl set_mrl = { .len = 128U, .ibi_len = 1U };
	struct i3c_ccc_mrl get_mrl = { 0 };

	ret = i3c_ccc_do_setmrl(target, &set_mrl);
	if (ret != 0) {
		TEST_SUB_FAIL("setmrl_getmrl", "SETMRL failed");
	} else {
		ret = i3c_ccc_do_getmrl(target, &get_mrl);
		if (ret != 0) {
			TEST_SUB_FAIL("setmrl_getmrl", "GETMRL failed");
		} else if (get_mrl.len != 128U) {
			TEST_SUB_FAIL("setmrl_getmrl", "MRL not accepted");
		} else {
			TEST_SUB_PASS("setmrl_getmrl");
		}
	}

	/* Step 3: GETPID */
	struct i3c_ccc_getpid pid_resp = { 0 };

	ret = i3c_ccc_do_getpid(target, &pid_resp);
	if (ret != 0) {
		TEST_SUB_FAIL("getpid", "GETPID failed");
	} else {
		/* Reconstruct 48-bit PID from big-endian 6 bytes */
		uint64_t received_pid = 0;

		for (int i = 0; i < 6; i++) {
			received_pid = (received_pid << 8) | pid_resp.pid[i];
		}
		uint64_t expected_pid =
			((uint64_t)CONFIG_I3C_HOST_TARGET_PID_HI << 32) |
			(uint64_t)CONFIG_I3C_HOST_TARGET_PID_LO;

		if (received_pid != expected_pid) {
			LOG_DBG("GETPID: got 0x%012llx expected 0x%012llx",
				received_pid, expected_pid);
			TEST_SUB_FAIL("getpid", "PID mismatch");
		} else {
			TEST_SUB_PASS("getpid");
		}
	}

	/* Step 4: GETBCR / GETDCR */
	struct i3c_ccc_getbcr bcr = { 0 };
	struct i3c_ccc_getdcr dcr = { 0 };
	int ret_b = i3c_ccc_do_getbcr(target, &bcr);
	int ret_d = i3c_ccc_do_getdcr(target, &dcr);

	if (ret_b != 0) {
		TEST_SUB_FAIL("getbcr_getdcr", "GETBCR failed");
	} else if (ret_d != 0) {
		TEST_SUB_FAIL("getbcr_getdcr", "GETDCR failed");
	} else {
		LOG_DBG("[step] getbcr_getdcr: BCR=0x%02x DCR=0x%02x",
			bcr.bcr, dcr.dcr);
		TEST_SUB_PASS("getbcr_getdcr");
	}

	TEST_GROUP_RESULT("ccc_commands");
}

/* ====================================================================
 * TC2: ibi
 * ==================================================================== */
static void run_tc2_ibi(const struct device *dev, struct i3c_device_desc *target)
{
	TEST_REPORT_INIT();
	int ret;
	uint32_t events;
	struct i3c_xec_regs *regs = (struct i3c_xec_regs *)I3C_BASE_ADDR;

	k_event_clear(host_events, 0xFFFFFFFFU);

#if 1
	/* Step 1: ibi_tir — write 128 bytes, expect IBI with MDB=0xAE, read back */
	target->ibi_cb = i3c_host_ibi_cb;
	ret = i3c_ibi_enable(target);
	if (ret != 0) {
		TEST_SUB_FAIL("ibi_tir", "ibi_enable failed");
		goto step2_ibi_enable_disable;
	}
#endif

	for (uint32_t b = 0; b < 128U; b++) {
		host_tx_buf[b] = (uint8_t)(b & 0xFFU);
	}
	ret = i3c_write(target, host_tx_buf, 128U);
	if (ret != 0) {
		TEST_SUB_FAIL("ibi_tir", "write failed");
		goto step2_ibi_enable_disable;
	}

#if 0
	/* Step 1: ibi_tir — write 128 bytes, expect IBI with MDB=0xAE, read back */
	target->ibi_cb = i3c_host_ibi_cb;
	ret = i3c_ibi_enable(target);
	if (ret != 0) {
		TEST_SUB_FAIL("ibi_tir", "ibi_enable failed");
		goto step2_ibi_enable_disable;
	}
	//i3c_xec_dump_regs(regs);
#endif

	events = k_event_wait(host_events, EVT_IBI_TIR_MDB_AE, true,
			      K_MSEC(I3C_TEST_TIMEOUT_MS));
	if (!(events & EVT_IBI_TIR_MDB_AE)) {
		TEST_SUB_FAIL("ibi_tir", "IBI timeout");
	} else {
		memset(host_rx_buf, 0, 128U);
		ret = i3c_read(target, host_rx_buf, 128U);
		if (ret != 0) {
			TEST_SUB_FAIL("ibi_tir", "read failed");
		} else if (!verify_loopback(host_tx_buf, host_rx_buf, 128U, "ibi_tir")) {
			TEST_SUB_FAIL("ibi_tir", "data mismatch");
		} else {
			TEST_SUB_PASS("ibi_tir");
		}
	}

	memset(host_rx_buf, 0, 128U);
	ret = i3c_read(target, host_rx_buf, 128U);
	if (ret != 0) {
		TEST_SUB_FAIL("ibi_tir", "read failed");
	} else if (!verify_loopback(host_tx_buf, host_rx_buf, 128U, "ibi_tir")) {
		TEST_SUB_FAIL("ibi_tir", "data mismatch");
	} else {
		TEST_SUB_PASS("ibi_tir");
	}

step2_ibi_enable_disable:
#if 0
	/* Step 2a: ibi_enable — verify IBI fires when enabled */
	k_event_clear(host_events, 0xFFFFFFFFU);
	(void)i3c_ibi_enable(target);

	host_tx_buf[0] = 0x01U;
	(void)i3c_write(target, host_tx_buf, 1U);

	events = k_event_wait(host_events, EVT_IBI_ANY, true,
			      K_MSEC(I3C_TEST_IBI_POLL_MS));
	if (events & EVT_IBI_ANY) {
		TEST_SUB_PASS("ibi_enable");
	} else {
		TEST_SUB_FAIL("ibi_enable", "IBI did not fire after enable");
	}

	/* Step 2b: ibi_disable — verify no IBI after disable */
	(void)i3c_ibi_disable(target);
	k_event_clear(host_events, 0xFFFFFFFFU);

	host_tx_buf[0] = 0x02U;
	(void)i3c_write(target, host_tx_buf, 1U);

	events = k_event_wait(host_events, EVT_IBI_ANY, true,
			      K_MSEC(I3C_TEST_IBI_POLL_MS));
	if (events & EVT_IBI_ANY) {
		TEST_SUB_FAIL("ibi_disable", "IBI fired after disable");
	} else {
		TEST_SUB_PASS("ibi_disable");
	}

	/* Step 3: hot_join — configure controller to ACK HJ, wait for HJ IBI */
	k_event_clear(host_events, 0xFFFFFFFFU);
	/*
	 * Enable hot-join ACK on the controller.  This tells the DW IP to
	 * acknowledge (not NACK) an incoming hot-join IBI.
	 */
	ret = i3c_ibi_hj_response(dev, true);
	if (ret != 0 && ret != -ENOSYS) {
		LOG_WRN("hot_join: i3c_ibi_hj_response not supported (%d)", ret);
	}

	events = k_event_wait(host_events, EVT_IBI_HOTJOIN, true,
			      K_MSEC(I3C_TEST_HJ_TIMEOUT_MS));
	if (events & EVT_IBI_HOTJOIN) {
		TEST_SUB_PASS("hot_join");
	} else {
		TEST_SUB_FAIL("hot_join", "HJ IBI timeout");
	}

	/* Step 4: ibi_reject — disable IBI, verify none fired */
	(void)i3c_ibi_disable(target);
	k_event_clear(host_events, 0xFFFFFFFFU);

	for (uint32_t b = 0; b < 64U; b++) {
		host_tx_buf[b] = (uint8_t)(b & 0xFFU);
	}
	(void)i3c_write(target, host_tx_buf, 64U);

	events = k_event_wait(host_events, EVT_IBI_ANY, true,
			      K_MSEC(I3C_TEST_IBI_POLL_MS));
	if (events & EVT_IBI_ANY) {
		TEST_SUB_FAIL("ibi_reject", "IBI received after disable");
	} else {
		TEST_SUB_PASS("ibi_reject");
	}
#endif
	TEST_GROUP_RESULT("ibi");
}

/* ====================================================================
 * TC3: advanced
 * ==================================================================== */
static void run_tc3_advanced(const struct device *dev,
			     struct i3c_device_desc *target)
{
	TEST_REPORT_INIT();
	int ret;

	/* Steps 1–3: frequency sweep at 1 / 6.25 / 12.5 MHz */
	static const struct {
		uint32_t    hz;
		const char *name;
	} freqs[] = {
		{ 1000000U,  "freq_1MHz"    },
		{ 6250000U,  "freq_6_25MHz" },
		{ 12500000U, "freq_12_5MHz" },
	};

	struct i3c_config_controller scl_cfg;

	for (size_t f = 0; f < ARRAY_SIZE(freqs); f++) {
		memset(&scl_cfg, 0, sizeof(scl_cfg));
		scl_cfg.scl.i3c = freqs[f].hz;

		ret = i3c_configure(dev, I3C_CONFIG_CONTROLLER, &scl_cfg);
		if (ret != 0) {
			TEST_SUB_FAIL(freqs[f].name, "i3c_configure failed");
			continue;
		}

		for (uint32_t b = 0; b < 64U; b++) {
			host_tx_buf[b] = (uint8_t)(b & 0xFFU);
		}
		memset(host_rx_buf, 0, 64U);

		int ret_w = i3c_write(target, host_tx_buf, 64U);

		k_sleep(K_MSEC(I3C_TEST_ECHO_DELAY_MS));

		int ret_r = i3c_read(target, host_rx_buf, 64U);

		if (ret_w != 0 || ret_r != 0) {
			TEST_SUB_FAIL(freqs[f].name, "transfer failed");
		} else if (!verify_loopback(host_tx_buf, host_rx_buf, 64U, freqs[f].name)) {
			TEST_SUB_FAIL(freqs[f].name, "data mismatch");
		} else {
			TEST_SUB_PASS(freqs[f].name);
		}
	}

	/* Restore SCL to 12.5 MHz regardless of sweep results */
	memset(&scl_cfg, 0, sizeof(scl_cfg));
	scl_cfg.scl.i3c = I3C_SCL_TYP_HZ;
	(void)i3c_configure(dev, I3C_CONFIG_CONTROLLER, &scl_cfg);

	/* Step 4: err_invalid_addr — write to unassigned address 0x7F */
	struct i3c_device_desc bad_desc = {
		.bus          = dev,
		.dynamic_addr = 0x7FU,
	};
	ret = i3c_write(&bad_desc, host_tx_buf, 1U);
	if (ret == 0) {
		TEST_SUB_FAIL("err_invalid_addr", "expected error, got 0");
	} else {
		TEST_SUB_PASS("err_invalid_addr");
	}

	/* Step 5: err_exceed_mwl — SETMWL(32), then try to write 128 bytes */
	const struct i3c_ccc_mwl small_mwl = { .len = 32U };

	ret = i3c_ccc_do_setmwl(target, &small_mwl);
	if (ret != 0) {
		TEST_SUB_FAIL("err_exceed_mwl", "SETMWL(32) failed");
	} else {
		ret = i3c_write(target, host_tx_buf, 128U);
		if (ret == 0) {
			TEST_SUB_FAIL("err_exceed_mwl", "expected error, got 0");
		} else {
			TEST_SUB_PASS("err_exceed_mwl");
		}
	}

	/* Always restore MWL to 128 to leave bus in a clean state */
	const struct i3c_ccc_mwl restore_mwl = { .len = 128U };

	(void)i3c_ccc_do_setmwl(target, &restore_mwl);

	TEST_GROUP_RESULT("advanced");
}

/* ====================================================================
 * setup_i3c_host — called from main() before thread start
 * ==================================================================== */
int setup_i3c_host(const struct device *dev)
{
	if (!device_is_ready(dev)) {
		int ret = device_init(dev);

		if (ret != 0) {
			LOG_ERR("Host[%s] init failed: %d", dev->name, ret);
			return ret;
		}
	}
	LOG_INF("Host[%s] ready", dev->name);
	return 0;
}

/* ====================================================================
 * i3c_host_xfer_task — thread entry point
 * ==================================================================== */
void i3c_host_xfer_task(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p3);

	const struct device *dev = (const struct device *)p1;

	host_events = (struct k_event *)p2;

	/*
	 * Wait for the target to complete i3c_target_register().
	 * In the standalone host app, main() posts this bit before starting
	 * the thread so the wait completes immediately.
	 * In the loopback app, the target thread posts it after register.
	 */
	(void)k_event_wait(host_events, EVT_TARGET_READY, false, K_FOREVER);

	const struct i3c_device_id devid = {
		.pid = ((uint64_t)CONFIG_I3C_HOST_TARGET_PID_HI << 32) |
		       (uint64_t)CONFIG_I3C_HOST_TARGET_PID_LO
	};

	struct i3c_device_desc *target = i3c_device_find(dev, &devid);

	if (!target || !target->dynamic_addr) {
		uint64_t pid_val = devid.pid;
		LOG_ERR("[FAIL] target 0x%012llx not found on bus (dynamic_addr=0)",
			pid_val);
		k_sleep(K_FOREVER);
		return;
	}

	LOG_INF("Target found: dynamic_addr=0x%02x", target->dynamic_addr);
	target->ibi_cb = i3c_host_ibi_cb;

	switch (CONFIG_I3C_TEST_CASE) {
	case 0:
		run_tc0_data_transfer(dev, target);
		break;
	case 1:
		run_tc1_ccc_commands(dev, target);
		break;
	case 2:
		run_tc2_ibi(dev, target);
		break;
	case 3:
		run_tc3_advanced(dev, target);
		break;
	default:
		LOG_ERR("[FAIL] unknown test case %d", CONFIG_I3C_TEST_CASE);
		break;
	}

	k_sleep(K_FOREVER);
}
