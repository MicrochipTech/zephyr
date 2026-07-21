/*
 * Copyright (c) 2026, Microchip Technology Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Microchip XEC I2Cv3 byte mode I2C driver.
 */

#include "soc_ecia.h"
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/mchp_xec_i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(i2c_mchp_xec_v3_bm, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mchp_xec_regs.h"

#if 0
The Recommended Architecture: ISR-Driven State MachineFor a byte-by-byte, non-DMA I2C controller, 
do not use the work queue to pump the bytes. Instead, use an ISR-driven state machine for the 
transmission, and use the shared work queue only to process transaction completion or errors.
[I2C Hardware Interrupt]
       │
       ▼
[Driver ISR (Static C Function)]
       │───► Reads status (ACK, NAK, Bus Error).
       │───► If more bytes: Writes next byte to data register, exits ISR immediately.
       │───► If Done or Error:
       │         │
       │         ▼
         [k_work_submit_to_queue()] 
                 │
                 ▼
       [Shared Work Queue Thread]
                 │───► Processes the I2C callback to upper layer.
                 │───► Signals threads waiting on k_sem.
                 │───► Handles error recovery (bus reset).


With 8 distinct I2C controllers, all operating asynchronously, your architecture must be completely
non-blocking.Zephyr’s standard i2c_transfer() API is strictly synchronous (blocking), but Zephyr 
does provide a standard asynchronous mechanism via the RTIO (Request-Response I/O) framework, 
or you can implement a standard user-callback registration pattern.
Because you are targeting an asynchronous API, you can completely eliminate the use of 
k_sem (semaphores) in your driver data structure. 
Instead, the shared work queue will be responsible for executing the user-registered callback 
functions safely out of interrupt context.

[Application]
       │  (Calls async_i2c_transfer with callback pointer)
       ▼
[Driver Instance X] ───► Starts first byte ───► Exits immediately to caller
                                                       │
   ┌───────────────────────────────────────────────────┘
   ▼
[Hardware ISR] ───► Pumps bytes 1-by-1 (Interrupt per byte)
   │
   └─► (Last Byte Complete / Error) ───► k_work_submit_to_queue()
                                                 │
                                                 ▼
                                     [Shared Work Queue Thread]
                                                 │
                                                 ▼
                                      Executes User Callback 
                                  (e.g., rx_callback(dev, error))

// User callback signature for async completion
typedef void (*my_i2c_async_cb_t)(const struct device *dev, int error, void *user_data);

struct my_i2c_async_data {
    struct k_work completion_work; // Embedded work item for the shared queue
    
    // Buffer state tracking
    uint8_t *msg_buf;
    uint32_t msg_len;
    uint32_t msg_idx;
    int status_err;
    
    // Async callback tracking
    my_i2c_async_cb_t callback;
    void *callback_user_data;
};

// Pre-allocated single shared work queue for all 8 instances
static K_THREAD_STACK_DEFINE(i2c_async_workq_stack, 1536); // Slightly larger stack to handle user callback depth
static struct k_work_q i2c_async_workq;

static void i2c_async_work_handler(struct k_work *work)
{
    // CONTAINER_OF identifies which of the 8 instances completed
    struct my_i2c_async_data *data = CONTAINER_OF(work, struct my_i2c_async_data, completion_work);
    
    // Safely deduce the device pointer if needed by the callback
    // (Assuming you map data back to device struct or pass it via user_data)
    
    if (data->callback) {
        // Execute the user's callback out of interrupt context
        data->callback(NULL, data->status_err, data->callback_user_data);
    }
}

void my_i2c_controller_isr(const struct device *dev)
{
    struct my_i2c_async_data *data = dev->data;
    
    // Read your specific hardware status registers
    uint32_t status = sys_read32(DEVICE_GET_STATUS_REG(dev)); 

    // Handle Errors (NAK, Bus Error, Lost Arbitration)
    if (status & (STATUS_BUS_ERROR | STATUS_LOST_ARB | STATUS_NAK)) {
        data->status_err = (status & STATUS_NAK) ? -ENXIO : -EIO;
        DISABLE_I2C_INTERRUPTS(dev);
        k_work_submit_to_queue(&i2c_async_workq, &data->completion_work);
        return;
    }

    // Handle Active Byte Transfers (ACK)
    if (status & STATUS_ACK) {
        if (data->msg_idx < data->msg_len) {
            // Put next byte on the bus; clears interrupt, waits for next 9 clocks
            sys_write32(data->msg_buf[data->msg_idx++], DEVICE_GET_DATA_REG(dev));
        } else {
            // All bytes transferred successfully!
            data->status_err = 0;
            DISABLE_I2C_INTERRUPTS(dev);
            // Offload user callback invocation to the work queue
            k_work_submit_to_queue(&i2c_async_workq, &data->completion_work);
        }
    }
}

int my_i2c_transfer_async(const struct device *dev, uint8_t *buf, uint32_t len, 
                          my_i2c_async_cb_t cb, void *user_data)
{
    struct my_i2c_async_data *data = dev->data;

    // Basic state checking to prevent re-entrancy on a single instance
    if (k_work_is_pending(&data->completion_work)) {
        return -EBUSY; 
    }

    // Set up transfer context
    data->msg_buf = buf;
    data->msg_len = len;
    data->msg_idx = 0;
    data->status_err = 0;
    data->callback = cb;
    data->callback_user_data = user_data;

    // Initialize the work item bound to the shared queue handler
    k_work_init(&data->completion_work, i2c_async_work_handler);

    // Trigger the I2C hardware to send the first byte / address frame
    sys_write32(data->msg_buf[data->msg_idx++], DEVICE_GET_DATA_REG(dev));
    ENABLE_I2C_INTERRUPTS(dev);

    return 0; // Request accepted, processing asynchronously
}

// Must support multi-messages such as I2C write-read, etc.
// Gemini recommended custom CONFIG_I2C_CALLBACK instead of RTIO.
// RTIO was designed for DMA based or peripherals with large FIFOs.
// We will go with CONFIG_I2C_CALLBACK
struct my_i2c_async_data {
    struct k_work completion_work;
    
    // Combined Transaction Tracking
    struct i2c_msg *msgs;        // Array of messages (e.g. Write then Read)
    uint32_t num_msgs;           // Total number of messages in this transfer
    uint32_t current_msg_idx;    // Which message we are currently processing
    
    // Byte Tracking for the ACTIVE message
    uint32_t byte_idx;
    int status_err;

    my_i2c_async_cb_t callback;
    void *callback_user_data;
};

void my_i2c_controller_isr(const struct device *dev)
{
    struct my_i2c_async_data *data = dev->data;
    uint32_t status = sys_read32(DEVICE_GET_STATUS_REG(dev)); 
    struct i2c_msg *current_msg = &data->msgs[data->current_msg_idx];

    // 1. Handle Errors
    if (status & (STATUS_BUS_ERROR | STATUS_LOST_ARB | STATUS_NAK)) {
        data->status_err = (status & STATUS_NAK) ? -ENXIO : -EIO;
        sys_write32(CMD_GENERATE_STOP, DEVICE_GET_CTRL_REG(dev));
        DISABLE_I2C_INTERRUPTS(dev);
        k_work_submit_to_queue(&i2c_async_workq, &data->completion_work);
        return;
    }

    // 2. Handle Byte Pumping
    if (status & STATUS_ACK) {
        if (data->byte_idx < current_msg->len) {
            // Process the current active message bytes
            if ((current_msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
                current_msg->buf[data->byte_idx++] = sys_read32(DEVICE_GET_DATA_REG(dev));
            } else {
                sys_write32(current_msg->buf[data->byte_idx++], DEVICE_GET_DATA_REG(dev));
            }
        } else {
            // CURRENT MESSAGE COMPLETE! Check if there's another combined message
            data->current_msg_idx++;
            if (data->current_msg_idx < data->num_msgs) {
                // Set up the next combined sequence message
                struct i2c_msg *next_msg = &data->msgs[data->current_msg_idx];
                data->byte_idx = 0;

                // Trigger a REPEATED START condition instead of a STOP
                sys_write32(CMD_GENERATE_REPEATED_START | (next_msg->flags & I2C_MSG_RW_MASK), 
                            DEVICE_GET_CTRL_REG(dev));
                
                // Send out target peripheral address frame
                sys_write32(next_msg->addr, DEVICE_GET_DATA_REG(dev));
         } else {
                // ALL COMBINED TRANSACTIONS COMPLETE
                data->status_err = 0;
                if (current_msg->flags & I2C_MSG_STOP) {
                    sys_write32(CMD_GENERATE_STOP, DEVICE_GET_CTRL_REG(dev));
                }
                DISABLE_I2C_INTERRUPTS(dev);
                k_work_submit_to_queue(&i2c_async_workq, &data->completion_work);
            }
        }
    }
}

To implement Solution 1 while safely managing up to 4 combined messages and preventing race
conditions from multiple threads, we will integrate a thread-safety mechanism.
Since your entry function must remain asynchronous and non-blocking, you cannot use a 
standard k_mutex to protect the instance (as mutexes force a thread to sleep/block when waiting). 
Instead, we will use an atomic flag (atomic_t) or a dedicated state tracker. 
This allows the API to fail immediately with a -EBUSY error if another thread is already 
utilizing that specific I2C instance.

#define MAX_COMBINED_MSGS 4

// Explicit status flags for state tracking
enum my_driver_state {
    MY_DRIVER_STATE_IDLE = 0,
    MY_DRIVER_STATE_BUSY = 1
};

struct my_i2c_async_data {
    struct k_work completion_work;
    
    // Thread safety state
    atomic_t driver_state; 
    
    // Internal copy array to hold up to 4 combined transactions safely
    struct i2c_msg msgs_copy[MAX_COMBINED_MSGS];
    uint32_t num_msgs;
    uint32_t current_msg_idx;
    
    // Active byte pointer tracking
    uint32_t byte_idx;
    int status_err;

    // Upstream Async Callbacks
    void (*callback)(const struct device *dev, int error, void *user_data);
    void *callback_user_data;
};

int my_i2c_transfer_async(const struct device *dev, 
                          const struct i2c_msg *msgs, 
                          uint8_t num_msgs, 
                          void (*cb)(const struct device *, int, void *), 
                          void *user_data)
{
    struct my_i2c_async_data *data = dev->data;

    // 1. Validate transaction boundaries
    if (num_msgs == 0 || num_msgs > MAX_COMBINED_MSGS) {
        return -EINVAL;
    }

    // 2. Atomic Lock: Attempt to claim this specific I2C instance
    if (!atomic_cas(&data->driver_state, MY_DRIVER_STATE_IDLE, MY_DRIVER_STATE_BUSY)) {
        return -EBUSY; // Thread safety breach prevented; instance is actively running
    }

    // 3. Deep-copy message tracking parameters securely
    data->num_msgs = num_msgs;
    data->current_msg_idx = 0;
    data->byte_idx = 0;
    data->status_err = 0;
    data->callback = cb;
    data->callback_user_data = user_data;

    for (int i = 0; i < num_msgs; i++) {
        data->msgs_copy[i] = msgs[i];
    }

    // Initialize work bound to the single shared work queue
    k_work_init(&data->completion_work, i2c_async_work_handler);

    // 4. Kickstart physical hardware sequence
    struct i2c_msg *first_msg = &data->msgs_copy[0];
    
    // Generate START and transmit Target Peripheral Address Frame
    sys_write32(CMD_GENERATE_START | (first_msg->flags & I2C_MSG_RW_MASK), DEVICE_GET_CTRL_REG(dev));
    
    if ((first_msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
        // If writing, put the very first data payload byte on the bus line
        sys_write32(first_msg->buf[data->byte_idx++], DEVICE_GET_DATA_REG(dev));
    }
    
    ENABLE_I2C_INTERRUPTS(dev);

    return 0; // Instance claimed and transaction processing completely in background
}

void my_i2c_controller_isr(const struct device *dev)
{
    struct my_i2c_async_data *data = dev->data;
    uint32_t status = sys_read32(DEVICE_GET_STATUS_REG(dev)); 
    struct i2c_msg *current_msg = &data->msgs_copy[data->current_msg_idx];

    // Handle Bus Level Errors / NAKs
    if (status & (STATUS_BUS_ERROR | STATUS_LOST_ARB | STATUS_NAK)) {
        data->status_err = (status & STATUS_NAK) ? -ENXIO : -EIO;
        sys_write32(CMD_GENERATE_STOP, DEVICE_GET_CTRL_REG(dev));
        DISABLE_I2C_INTERRUPTS(dev);
        k_work_submit_to_queue(&i2c_async_workq, &data->completion_work);
        return;
    }
    
    // Process Byte Sequencing (ACK received)
    if (status & STATUS_ACK) {
        if (data->byte_idx < current_msg->len) {
            if ((current_msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
                current_msg->buf[data->byte_idx++] = sys_read32(DEVICE_GET_DATA_REG(dev));
            } else {
                sys_write32(current_msg->buf[data->byte_idx++], DEVICE_GET_DATA_REG(dev));
            }
        } else {
            // End of active message boundary reached. Check for next chained packet.
            data->current_msg_idx++;
            if (data->current_msg_idx < data->num_msgs) {
                struct i2c_msg *next_msg = &data->msgs_copy[data->current_msg_idx];
                data->byte_idx = 0;

                // Fire a REPEATED START condition (No STOP command in between)
                sys_write32(CMD_GENERATE_REPEATED_START | (next_msg->flags & I2C_MSG_RW_MASK), 
                            DEVICE_GET_CTRL_REG(dev));
                
                if ((next_msg->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
                    sys_write32(next_msg->buf[data->byte_idx++], DEVICE_GET_DATA_REG(dev));
                }
            } else {
                // ALL COMBINED TRANSACTIONS COMPLETED SUCCESSFULLY
                data->status_err = 0;
                if (current_msg->flags & I2C_MSG_STOP) {
                    sys_write32(CMD_GENERATE_STOP, DEVICE_GET_CTRL_REG(dev));
                }
                DISABLE_I2C_INTERRUPTS(dev);
                k_work_submit_to_queue(&i2c_async_workq, &data->completion_work);
            }
        }
    }
}

static void i2c_async_work_handler(struct k_work *work)
{
    struct my_i2c_async_data *data = CONTAINER_OF(work, struct my_i2c_async_data, completion_work);
    
    // Extract the original callback context pointers before clearing state
    void (*user_cb)(const struct device *, int, void *) = data->callback;
    void *user_cb_data = data->callback_user_data;

    // Release the driver lock completely out of atomic/interrupt paths
    atomic_set(&data->driver_state, MY_DRIVER_STATE_IDLE);

    // Securely fire the user callback inside our safe cooperative thread context
    if (user_cb) {
        user_cb(NULL, data->status_err, user_cb_data);
    }
}

Architectural Analysis of this Strategy
Zero Thread Starvation: 
Using an atomic Compare-And-Swap instead of a mutex ensures that high-priority threads 
attempting to submit to a busy controller are rejected instantly with an error code 
rather than forcing priority inversion or blocking paths.

Predictable RAM Consumption: 
Copying 4 struct i2c_msg records consumes exactly 48 bytes of static RAM per instance 
(assuming a 12-byte standard Zephyr i2c_msg memory profile). 
Across 8 instances, this maps to a total overhead of less than 400 bytes, which is much lower 
than the multi-kilobyte baseline required by the RTIO engine.

If you want to review the error behavior under high-stress conditions, let me know:

How you intend to handle bus recovery (e.g., standard clock-clearing loops) if a transaction 
fails with a timeout or lockup?

If you need to enforce a maximum transaction timeout duration using Zephyr's 
k_work_delayable subsystem?


#endif

struct xec_i2c_v3_bm_xcfg {
	uintptr_t base;
	uint32_t dflt_freq;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	void (*irq_connect)(void);

};

struct xec_i2c_v3_bm_xdat {
	const struct device *ctrl_dev;
	uint32_t i2c_cmpl;
	uint32_t i2c_cfg;
	uint8_t i2c_sr;
	uint8_t i2c_cr_cache;
	struct i2c_msg *msgs;
	uint16_t nmsgs;
	struct k_mutex lock;
	struct k_sem sync;
	struct k_work kw;
	int instance_id;
	uint32_t active_freq;
	uint8_t active_port;
	struct i2c_msg *msgs;
	uint8_t num_msgs;
	uint8_t msg_idx;
	uint8_t msg_len;
	uint8_t *msg_buf;
};

struct xec_i2c_v3_bm_port_xcfg {
	const struct device *parent;
	const struct pinctrl_dev_config *pcfg;
	uint32_t bitrate;
	uint8_t port_id;
	bool is_default;
};

K_THREAD_STACK_DEFINE(xec_i2c_v3_bm_q_stack, CONFIG_I2C_MCHP_XEC_V3_BM_KWQ_STACK_SIZE);
static struct k_work_q xec_i2c_v3_bm_work_q;

static void xec_i2c_v3_bm_work_handler(struct k_work *work)
{
	struct xec_i2c_v3_bm_xdat *const xdat = CONTAINER_OF(work, struct xec_i2c_v3_bm_xdat, kw);

	LOG_INF("XEC I2Cv3 kwq id = %d", xdat->instance_id);
}

#if 0
static void my_i2c_work_handler(struct k_work *work)
{
    // Extract which of the 8 instances completed
    struct my_i2c_instance_data *data = CONTAINER_OF(work, struct my_i2c_instance_data, transfer_work);

    if (data->status_err != 0) {
        // Perform heavy/slow error recovery (e.g., toggling GPIOs to clear a stuck bus)
        recover_stuck_i2c_bus();
    }

    // Unblock the thread that initiated the I2C transfer
    k_sem_give(&data->transfer_sync);
}

void my_i2c_isr(const struct device *dev)
{
    struct my_i2c_instance_data *data = dev->data;
    uint32_t status = read_i2c_status_reg();

    // 1. Handle Errors Immediately
    if (status & (STATUS_BUS_ERROR | STATUS_LOST_ARB)) {
        data->status_err = -EIO;
        disable_i2c_interrupts();
        k_work_submit_to_queue(&shared_driver_workq, &data->transfer_work);
        return;
    }
    if (status & STATUS_NAK) {
        data->status_err = -ENXIO; // No device responded
        disable_i2c_interrupts();
        k_work_submit_to_queue(&shared_driver_workq, &data->transfer_work);
        return;
    }

    // 2. Handle Byte Progression (ACK received)
    if (status & STATUS_ACK) {
        if (data->msg_idx < data->msg_len) {
            // Write next byte out to the hardware
            write_i2c_data_reg(data->msg_buf[data->msg_idx++]);
        } else {
            // Transaction finished successfully!
            data->status_err = 0;
            disable_i2c_interrupts();
            k_work_submit_to_queue(&shared_driver_workq, &data->transfer_work);
        }
    }
}
#endif

static void xec_i2c_v3_bm_isr(const struct device *ctrl_dev)
{
	const struct xec_i2c_v3_bm_xcfg *xcfg = ctrl_dev->config;
	struct xec_i2c_v3_bm_xdat *const xdat = ctrl_dev->data;
	uintptr_t base = xcfg->base;

	xdat->i2c_cmpl = sys_read32(base + XEC_I2C_CMPL_OFS);
	xdat->i2c_sr = sys_read8(base + XEC_I2C_SR_OFS);

	soc_ecia_girq_status_clear(xcfg->girq, xcfg->girq_pos);
}

/* ---- API ---- */
static int xec_i2c_v3_bm_vport_configure(const struct device *port_dev, uint32_t i2c_config)
{
	return 0;
}

static int xec_i2c_v3_bm_vport_get_config(const struct device *port_dev, uint32_t *i2c_config)
{
	return 0;
}

static int xec_i2c_v3_bm_vport_transfer(const struct device *port_dev, struct i2c_msg *msgs,
                                        uint8_t num_msgs, uint16_t address)
{
	/* k_work_sumbit_to_queue(&xec_i2c_v3_bm_work_q, &xdat->kw); */
	return 0;
}

static int xec_i2c_v3_bm_vport_recover_bus(const struct device *port_dev)
{
	return 0;
}

static int xec_i2c_v3_bm_cr_init(const struct device *cr_dev, uint32_t freq, uint8_t port)
{
	const struct xec_i2c_v3_bm_xcfg *xcfg = cr_dev->config;
	struct xec_i2c_v3_bm_xdat *const xdat = cr_dev->data;
	static bool wq_started = false;

	xdat->ctrl_dev = cr_dev;
	xdat->active_freq = freq;
	xdat->active_port = port;

	soc_xec_pcr_sleep_en_clear(xcfg->enc_pcr);

	/* TODO controller configuration. Note, port init has configured port pins before
	 * calling this function.
	 */

	k_work_init(&xdat->kw, xec_i2c_v3_bm_work_handler);

	if (!wq_started) {
		/* start custom work queue at low priority (high positive number) */
		k_work_queue_start(&xec_i2c_v3_bm_work_q, xec_i2c_v3_bm_q_stack,
				   K_THREAD_STACK_SIZEOF(xec_i2c_v3_bm_q_stack),
				   K_PRIO_PREEMPT(CONFIG_I2C_MCHP_XEC_V3_BM_KWQ_PRIORITY), NULL);
		/* Use K_PRIO_COOP(2) or K_PRIO_COOP(3) */
		wq_started = true;
	}

	if (xcfg->irq_connect != NULL) {
		xcfg->irq_connect();
		soc_ecia_girq_ctrl(xcfg->girq, xcfg->girq_pos, MCHP_MEC_ECIA_GIRQ_EN);
	}

	return 0;
}

static int xec_i2c_v3_bm_port_init(const struct device *port_dev)
{
	const struct xec_i2c_v3_bm_port_xcfg *pc = port_dev->config;
	int rc = 0;

	rc = pinctrl_apply_state(pc->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("pinctrl_apply_state(%s)=%d", port_dev->name, rc);
		return rc;
	}

	if (pc->is_default) {
		rc = xec_i2c_v3_bm_cr_init(pc->parent, pc->bitrate, pc->port_id);
	}

	return 0;
}

static DEVICE_API(i2c, xec_i2c_v3_bm_port_api) = {
	.configure = xec_i2c_v3_bm_vport_configure,
	.get_config = xec_i2c_v3_bm_vport_get_config,
	.transfer = xec_i2c_v3_bm_vport_transfer,
	.recover_bus = xec_i2c_v3_bm_vport_recover_bus,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

/* core driver device structure */
#define DT_DRV_COMPAT microchip_xec_i2c_v3_bm

#define XEC_I2C_V3_DFLT_FREQ(inst) I2C_BITRATE_STANDARD

#define XEC_I2C_V3_GIRQ(inst)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP(inst, girqs))
#define XEC_I2C_V3_GIRQ_POS(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP(inst, girqs))

#define XEC_I2C_V3_BM_CTRL_INIT(inst)                                                              \
	static void xec_i2c_v3_bm_irq_connect_##inst(void)                                         \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), xec_i2c_v3_bm_isr,    \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct xec_i2c_v3_bm_xcfg xec_i2c_v3_bm_xcfg_##inst = {                       \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.irq_connect = xec_i2c_v3_bm_irq_connect_##inst,                                   \
		.dflt_freq = XEC_I2C_V3_DFLT_FREQ(inst),                                           \
		.girq = XEC_I2C_V3_GIRQ(inst),                                                     \
		.girq_pos = XEC_I2C_V3_GIRQ_POS(inst),                                             \
		.enc_pcr = DT_INST_PROP(inst, pcr_scr),                                            \
	};                                                                                         \
	static struct xec_i2c_v3_bm_xdat xec_i2c_v3_bm_xdat_##inst;                                \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &xec_i2c_v3_bm_xdat_##inst,                        \
			      &xec_i2c_v3_bm_xcfg_##inst, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,   \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_V3_BM_CTRL_INIT)

/* port driver device structure */
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT microchip_xec_i2c_v3_bm_port

#define XEC_I2C_V3_BM_PORT_INIT(inst)                                                              \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	static const struct xec_i2c_v3_bm_port_xcfg xec_i2c_v3_bm_port_xcfg_##inst = {             \
		.parent = DEVICE_DT_GET(DT_INST_PHANDLE(inst, controller)),                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		.bitrate = DT_INST_PROP_OR(inst, clock_frequency, I2C_BITRATE_STANDARD),           \
		.port_id = (uint8_t)(DT_INST_PROP(inst, port) & 0x0FU),                            \
		.is_default = DT_INST_PROP(inst, default_port),                                    \
	};                                                                                         \
	I2C_DEVICE_DT_INST_DEFINE(                                                                 \
		inst, xec_i2c_v3_bm_port_init, NULL, NULL, &xec_i2c_v3_bm_port_xcfg_##inst,        \
		POST_KERNEL, CONFIG_I2C_MCHP_XEC_V3_BM_PORT_INIT_PRIORITY,                         \
		&xec_i2c_v3_bm_port_api);

DT_INST_FOREACH_STATUS_OKAY(XEC_I2C_V3_BM_PORT_INIT)
