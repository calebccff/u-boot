// SPDX-License-Identifier: GPL-2.0+
/*
 * RPMH RSC driver
 *
 * Copyright (C) 2023 Linaro
 *
 */

#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <dm/uclass.h>
#include <errno.h>
#include <remoteproc.h>

#include <asm/io.h>

#include <linux/bitops.h>
#include <linux/delay.h>

#include <dt-bindings/soc/qcom,rpmh-rsc.h>

#define USEC_PER_SEC			1000000L

#define RSC_DRV_ID			0

#define MAJOR_VER_MASK			0xFF
#define MAJOR_VER_SHIFT			16
#define MINOR_VER_MASK			0xFF
#define MINOR_VER_SHIFT			8

enum {
	RSC_DRV_TCS_OFFSET,
	RSC_DRV_CMD_OFFSET,
	DRV_SOLVER_CONFIG,
	DRV_PRNT_CHLD_CONFIG,
	RSC_DRV_IRQ_ENABLE,
	RSC_DRV_IRQ_STATUS,
	RSC_DRV_IRQ_CLEAR,
	RSC_DRV_CMD_WAIT_FOR_CMPL,
	RSC_DRV_CONTROL,
	RSC_DRV_STATUS,
	RSC_DRV_CMD_ENABLE,
	RSC_DRV_CMD_MSGID,
	RSC_DRV_CMD_ADDR,
	RSC_DRV_CMD_DATA,
	RSC_DRV_CMD_STATUS,
	RSC_DRV_CMD_RESP_DATA,
};

/* DRV HW Solver Configuration Information Register */
#define DRV_HW_SOLVER_MASK		1
#define DRV_HW_SOLVER_SHIFT		24

/* DRV TCS Configuration Information Register */
#define DRV_NUM_TCS_MASK		0x3F
#define DRV_NUM_TCS_SHIFT		6
#define DRV_NCPT_MASK			0x1F
#define DRV_NCPT_SHIFT			27

/* Offsets for CONTROL TCS Registers */
#define RSC_DRV_CTL_TCS_DATA_HI		0x38
#define RSC_DRV_CTL_TCS_DATA_HI_MASK	0xFFFFFF
#define RSC_DRV_CTL_TCS_DATA_HI_VALID	BIT(31)
#define RSC_DRV_CTL_TCS_DATA_LO		0x40
#define RSC_DRV_CTL_TCS_DATA_LO_MASK	0xFFFFFFFF
#define RSC_DRV_CTL_TCS_DATA_SIZE	32

#define TCS_AMC_MODE_ENABLE		BIT(16)
#define TCS_AMC_MODE_TRIGGER		BIT(24)

/* TCS CMD register bit mask */
#define CMD_MSGID_LEN			8
#define CMD_MSGID_RESP_REQ		BIT(8)
#define CMD_MSGID_WRITE			BIT(16)
#define CMD_STATUS_ISSUED		BIT(8)
#define CMD_STATUS_COMPL		BIT(16)

#define TCS_TYPE_NR			4
#define MAX_CMDS_PER_TCS		16
#define MAX_TCS_PER_TYPE		3
#define MAX_TCS_NR			(MAX_TCS_PER_TYPE * TCS_TYPE_NR)
#define MAX_TCS_SLOTS			(MAX_CMDS_PER_TCS * MAX_TCS_PER_TYPE)

struct rsc_drv;

#define MAX_RPMH_PAYLOAD	16

/**
 * rpmh_state: state for the request
 *
 * RPMH_SLEEP_STATE:       State of the resource when the processor subsystem
 *                         is powered down. There is no client using the
 *                         resource actively.
 * RPMH_WAKE_ONLY_STATE:   Resume resource state to the value previously
 *                         requested before the processor was powered down.
 * RPMH_ACTIVE_ONLY_STATE: Active or AMC mode requests. Resource state
 *                         is aggregated immediately.
 */
enum rpmh_state {
	RPMH_SLEEP_STATE,
	RPMH_WAKE_ONLY_STATE,
	RPMH_ACTIVE_ONLY_STATE,
};

/**
 * struct tcs_cmd: an individual request to RPMH.
 *
 * @addr: the address of the resource slv_id:18:16 | offset:0:15
 * @data: the resource state request
 * @wait: ensure that this command is complete before returning.
 *        Setting "wait" here only makes sense during rpmh_write_batch() for
 *        active-only transfers, this is because:
 *        rpmh_write() - Always waits.
 *                       (DEFINE_RPMH_MSG_ONSTACK will set .wait_for_compl)
 *        rpmh_write_async() - Never waits.
 *                       (There's no request completion callback)
 */
struct tcs_cmd {
	u32 addr;
	u32 data;
	u32 wait;
};

/**
 * struct tcs_request: A set of tcs_cmds sent together in a TCS
 *
 * @state:          state for the request.
 * @wait_for_compl: wait until we get a response from the h/w accelerator
 *                  (same as setting cmd->wait for all commands in the request)
 * @num_cmds:       the number of @cmds in this request
 * @cmds:           an array of tcs_cmds
 */
struct tcs_request {
	enum rpmh_state state;
	u32 wait_for_compl;
	u32 num_cmds;
	struct tcs_cmd *cmds;
};

#define BCM_TCS_CMD_COMMIT_SHFT		30
#define BCM_TCS_CMD_COMMIT_MASK		0x40000000
#define BCM_TCS_CMD_VALID_SHFT		29
#define BCM_TCS_CMD_VALID_MASK		0x20000000
#define BCM_TCS_CMD_VOTE_X_SHFT		14
#define BCM_TCS_CMD_VOTE_MASK		0x3fff
#define BCM_TCS_CMD_VOTE_Y_SHFT		0
#define BCM_TCS_CMD_VOTE_Y_MASK		0xfffc000

/* Construct a Bus Clock Manager (BCM) specific TCS command */
#define BCM_TCS_CMD(commit, valid, vote_x, vote_y)		\
	(((commit) << BCM_TCS_CMD_COMMIT_SHFT) |		\
	((valid) << BCM_TCS_CMD_VALID_SHFT) |			\
	((cpu_to_le32(vote_x) &					\
	BCM_TCS_CMD_VOTE_MASK) << BCM_TCS_CMD_VOTE_X_SHFT) |	\
	((cpu_to_le32(vote_y) &					\
	BCM_TCS_CMD_VOTE_MASK) << BCM_TCS_CMD_VOTE_Y_SHFT))

/**
 * struct tcs_group: group of Trigger Command Sets (TCS) to send state requests
 * to the controller
 *
 * @drv:       The controller.
 * @type:      Type of the TCS in this group - active, sleep, wake.
 * @mask:      Mask of the TCSes relative to all the TCSes in the RSC.
 * @offset:    Start of the TCS group relative to the TCSes in the RSC.
 * @num_tcs:   Number of TCSes in this type.
 * @ncpt:      Number of commands in each TCS.
 * @req:       Requests that are sent from the TCS; only used for ACTIVE_ONLY
 *             transfers (could be on a wake/sleep TCS if we are borrowing for
 *             an ACTIVE_ONLY transfer).
 *             Start: grab drv->lock, set req, set tcs_in_use, drop drv->lock,
 *                    trigger
 *             End: get irq, access req,
 *                  grab drv->lock, clear tcs_in_use, drop drv->lock
 * @slots:     Indicates which of @cmd_addr are occupied; only used for
 *             SLEEP / WAKE TCSs.  Things are tightly packed in the
 *             case that (ncpt < MAX_CMDS_PER_TCS).  That is if ncpt = 2 and
 *             MAX_CMDS_PER_TCS = 16 then bit[2] = the first bit in 2nd TCS.
 */
struct tcs_group {
	struct rsc_drv *drv;
	int type;
	u32 mask;
	u32 offset;
	int num_tcs;
	int ncpt;
	const struct tcs_request *req[MAX_TCS_PER_TYPE];
	DECLARE_BITMAP(slots, MAX_TCS_SLOTS);
};

/**
 * struct rpmh_request: the message to be sent to rpmh-rsc
 *
 * @msg: the request
 * @cmd: the payload that will be part of the @msg
 * @completion: triggered when request is done
 * @dev: the device making the request
 * @needs_free: check to free dynamically allocated request object
 */
struct rpmh_request {
	struct tcs_request msg;
	struct tcs_cmd cmd[MAX_RPMH_PAYLOAD];
	struct completion *completion;
	const struct device *dev;
	bool needs_free;
};

/**
 * struct rpmh_ctrlr: our representation of the controller
 *
 * @cache: the list of cached requests
 * @cache_lock: synchronize access to the cache data
 * @dirty: was the cache updated since flush
 * @batch_cache: Cache sleep and wake requests sent as batch
 */
struct rpmh_ctrlr {
	struct list_head cache;
	spinlock_t cache_lock;
	bool dirty;
	struct list_head batch_cache;
};

struct rsc_ver {
	u32 major;
	u32 minor;
};

/**
 * struct rsc_drv: the Direct Resource Voter (DRV) of the
 * Resource State Coordinator controller (RSC)
 *
 * @name:               Controller identifier.
 * @base:               Start address of the DRV registers in this controller.
 * @tcs_base:           Start address of the TCS registers in this controller.
 * @id:                 Instance id in the controller (Direct Resource Voter).
 * @num_tcs:            Number of TCSes in this DRV.
 * @rsc_pm:             CPU PM notifier for controller.
 *                      Used when solver mode is not present.
 * @cpus_in_pm:         Number of CPUs not in idle power collapse.
 *                      Used when solver mode and "power-domains" is not present.
 * @genpd_nb:           PM Domain notifier for cluster genpd notifications.
 * @tcs:                TCS groups.
 * @tcs_in_use:         S/W state of the TCS; only set for ACTIVE_ONLY
 *                      transfers, but might show a sleep/wake TCS in use if
 *                      it was borrowed for an active_only transfer.  You
 *                      must hold the lock in this struct (AKA drv->lock) in
 *                      order to update this.
 * @lock:               Synchronize state of the controller.  If RPMH's cache
 *                      lock will also be held, the order is: drv->lock then
 *                      cache_lock.
 * @tcs_wait:           Wait queue used to wait for @tcs_in_use to free up a
 *                      slot
 * @client:             Handle to the DRV's client.
 * @dev:                RSC device.
 */
struct rsc_drv {
	const char *name;
	void __iomem *base;
	void __iomem *tcs_base;
	int id;
	int num_tcs;
	struct notifier_block rsc_pm;
	struct notifier_block genpd_nb;
	atomic_t cpus_in_pm;
	struct tcs_group tcs[TCS_TYPE_NR];
	DECLARE_BITMAP(tcs_in_use, MAX_TCS_NR);
	struct rpmh_ctrlr client;
	struct device *dev;
	struct rsc_ver ver;
	u32 *regs;
};

/*
 * Here's a high level overview of how all the registers in RPMH work
 * together:
 *
 * - The main rpmh-rsc address is the base of a register space that can
 *   be used to find overall configuration of the hardware
 *   (DRV_PRNT_CHLD_CONFIG). Also found within the rpmh-rsc register
 *   space are all the TCS blocks. The offset of the TCS blocks is
 *   specified in the device tree by "qcom,tcs-offset" and used to
 *   compute tcs_base.
 * - TCS blocks come one after another. Type, count, and order are
 *   specified by the device tree as "qcom,tcs-config".
 * - Each TCS block has some registers, then space for up to 16 commands.
 *   Note that though address space is reserved for 16 commands, fewer
 *   might be present. See ncpt (num cmds per TCS).
 *
 * Here's a picture:
 *
 *  +---------------------------------------------------+
 *  |RSC                                                |
 *  | ctrl                                              |
 *  |                                                   |
 *  | Drvs:                                             |
 *  | +-----------------------------------------------+ |
 *  | |DRV0                                           | |
 *  | | ctrl/config                                   | |
 *  | | IRQ                                           | |
 *  | |                                               | |
 *  | | TCSes:                                        | |
 *  | | +------------------------------------------+  | |
 *  | | |TCS0  |  |  |  |  |  |  |  |  |  |  |  |  |  | |
 *  | | | ctrl | 0| 1| 2| 3| 4| 5| .| .| .| .|14|15|  | |
 *  | | |      |  |  |  |  |  |  |  |  |  |  |  |  |  | |
 *  | | +------------------------------------------+  | |
 *  | | +------------------------------------------+  | |
 *  | | |TCS1  |  |  |  |  |  |  |  |  |  |  |  |  |  | |
 *  | | | ctrl | 0| 1| 2| 3| 4| 5| .| .| .| .|14|15|  | |
 *  | | |      |  |  |  |  |  |  |  |  |  |  |  |  |  | |
 *  | | +------------------------------------------+  | |
 *  | | +------------------------------------------+  | |
 *  | | |TCS2  |  |  |  |  |  |  |  |  |  |  |  |  |  | |
 *  | | | ctrl | 0| 1| 2| 3| 4| 5| .| .| .| .|14|15|  | |
 *  | | |      |  |  |  |  |  |  |  |  |  |  |  |  |  | |
 *  | | +------------------------------------------+  | |
 *  | |                    ......                     | |
 *  | +-----------------------------------------------+ |
 *  | +-----------------------------------------------+ |
 *  | |DRV1                                           | |
 *  | | (same as DRV0)                                | |
 *  | +-----------------------------------------------+ |
 *  |                      ......                       |
 *  +---------------------------------------------------+
 */

static u32 rpmh_rsc_reg_offset_ver_2_7[] = {
	[RSC_DRV_TCS_OFFSET]		= 672,
	[RSC_DRV_CMD_OFFSET]		= 20,
	[DRV_SOLVER_CONFIG]		= 0x04,
	[DRV_PRNT_CHLD_CONFIG]		= 0x0C,
	[RSC_DRV_IRQ_ENABLE]		= 0x00,
	[RSC_DRV_IRQ_STATUS]		= 0x04,
	[RSC_DRV_IRQ_CLEAR]		= 0x08,
	[RSC_DRV_CMD_WAIT_FOR_CMPL]	= 0x10,
	[RSC_DRV_CONTROL]		= 0x14,
	[RSC_DRV_STATUS]		= 0x18,
	[RSC_DRV_CMD_ENABLE]		= 0x1C,
	[RSC_DRV_CMD_MSGID]		= 0x30,
	[RSC_DRV_CMD_ADDR]		= 0x34,
	[RSC_DRV_CMD_DATA]		= 0x38,
	[RSC_DRV_CMD_STATUS]		= 0x3C,
	[RSC_DRV_CMD_RESP_DATA]		= 0x40,
};

static u32 rpmh_rsc_reg_offset_ver_3_0[] = {
	[RSC_DRV_TCS_OFFSET]		= 672,
	[RSC_DRV_CMD_OFFSET]		= 24,
	[DRV_SOLVER_CONFIG]		= 0x04,
	[DRV_PRNT_CHLD_CONFIG]		= 0x0C,
	[RSC_DRV_IRQ_ENABLE]		= 0x00,
	[RSC_DRV_IRQ_STATUS]		= 0x04,
	[RSC_DRV_IRQ_CLEAR]		= 0x08,
	[RSC_DRV_CMD_WAIT_FOR_CMPL]	= 0x20,
	[RSC_DRV_CONTROL]		= 0x24,
	[RSC_DRV_STATUS]		= 0x28,
	[RSC_DRV_CMD_ENABLE]		= 0x2C,
	[RSC_DRV_CMD_MSGID]		= 0x34,
	[RSC_DRV_CMD_ADDR]		= 0x38,
	[RSC_DRV_CMD_DATA]		= 0x3C,
	[RSC_DRV_CMD_STATUS]		= 0x40,
	[RSC_DRV_CMD_RESP_DATA]		= 0x44,
};

static inline void __iomem *
tcs_reg_addr(const struct rsc_drv *drv, int reg, int tcs_id)
{
	return drv->tcs_base + drv->regs[RSC_DRV_TCS_OFFSET] * tcs_id + reg;
}

static inline void __iomem *
tcs_cmd_addr(const struct rsc_drv *drv, int reg, int tcs_id, int cmd_id)
{
	return tcs_reg_addr(drv, reg, tcs_id) + drv->regs[RSC_DRV_CMD_OFFSET] * cmd_id;
}

static u32 read_tcs_cmd(const struct rsc_drv *drv, int reg, int tcs_id,
			int cmd_id)
{
	return readl_relaxed(tcs_cmd_addr(drv, reg, tcs_id, cmd_id));
}

static u32 read_tcs_reg(const struct rsc_drv *drv, int reg, int tcs_id)
{
	return readl_relaxed(tcs_reg_addr(drv, reg, tcs_id));
}

static void write_tcs_cmd(const struct rsc_drv *drv, int reg, int tcs_id,
			  int cmd_id, u32 data)
{
	printk("BHUPESH: %s: writing: 0x%x at 0x%pS, tcs_id: %d, cmd_id: %d, reg: %d\n",
		  __func__, data, tcs_cmd_addr(drv, reg, tcs_id, cmd_id), tcs_id, cmd_id, reg);
	writel_relaxed(data, tcs_cmd_addr(drv, reg, tcs_id, cmd_id));
}

static void write_tcs_reg(const struct rsc_drv *drv, int reg, int tcs_id,
			  u32 data)
{
	printk("BHUPESH: %s: writing: 0x%x at 0x%pS, tcs_id: %d, reg: %d\n",
		  __func__, data, tcs_reg_addr(drv, reg, tcs_id), tcs_id, reg);
	writel_relaxed(data, tcs_reg_addr(drv, reg, tcs_id));
}

static void write_tcs_reg_sync(const struct rsc_drv *drv, int reg, int tcs_id,
			       u32 data)
{
	int i;

	writel(data, tcs_reg_addr(drv, reg, tcs_id));
	printk("BHUPESH: %s: writing: 0x%x at 0x%pS, tcs_id: %d, reg: %d\n",
		  __func__, data, tcs_reg_addr(drv, reg, tcs_id), tcs_id, reg);

	/*
	 * Wait until we read back the same value.  Use a counter rather than
	 * ktime for timeout since this may be called after timekeeping stops.
	 */
	for (i = 0; i < USEC_PER_SEC; i++) {
		if (readl(tcs_reg_addr(drv, reg, tcs_id)) == data)
			return;
		udelay(1);
	}
	pr_err("%s: error writing %#x to %d:%#x\n", drv->name,
	       data, tcs_id, reg);
}

/**
 * __tcs_set_trigger() - Start xfer on a TCS or unset trigger on a borrowed TCS
 * @drv:     The controller.
 * @tcs_id:  The global ID of this TCS.
 * @trigger: If true then untrigger/retrigger. If false then just untrigger.
 *
 * In the normal case we only ever call with "trigger=true" to start a
 * transfer. That will un-trigger/disable the TCS from the last transfer
 * then trigger/enable for this transfer.
 *
 * If we borrowed a wake TCS for an active-only transfer we'll also call
 * this function with "trigger=false" to just do the un-trigger/disable
 * before using the TCS for wake purposes again.
 *
 * Note that the AP is only in charge of triggering active-only transfers.
 * The AP never triggers sleep/wake values using this function.
 */
static void __tcs_set_trigger(struct rsc_drv *drv, int tcs_id, bool trigger)
{
	u32 enable;
	u32 reg = drv->regs[RSC_DRV_CONTROL];

	/*
	 * HW req: Clear the DRV_CONTROL and enable TCS again
	 * While clearing ensure that the AMC mode trigger is cleared
	 * and then the mode enable is cleared.
	 */
	enable = read_tcs_reg(drv, reg, tcs_id);
	enable &= ~TCS_AMC_MODE_TRIGGER;
	write_tcs_reg_sync(drv, reg, tcs_id, enable);
	enable &= ~TCS_AMC_MODE_ENABLE;
	write_tcs_reg_sync(drv, reg, tcs_id, enable);

	if (trigger) {
		/* Enable the AMC mode on the TCS and then trigger the TCS */
		enable = TCS_AMC_MODE_ENABLE;
		write_tcs_reg_sync(drv, reg, tcs_id, enable);
		enable |= TCS_AMC_MODE_TRIGGER;
		write_tcs_reg(drv, reg, tcs_id, enable);
	}
}

/**
 * __tcs_buffer_write() - Write to TCS hardware from a request; don't trigger.
 * @drv:    The controller.
 * @tcs_id: The global ID of this TCS.
 * @cmd_id: The index within the TCS to start writing.
 * @msg:    The message we want to send, which will contain several addr/data
 *          pairs to program (but few enough that they all fit in one TCS).
 *
 * This is used for all types of transfers (active, sleep, and wake).
 */
static void __tcs_buffer_write(struct rsc_drv *drv, int tcs_id, int cmd_id,
			       const struct tcs_request *msg)
{
	u32 msgid;
	u32 cmd_msgid = CMD_MSGID_LEN | CMD_MSGID_WRITE;
	u32 cmd_enable = 0;
	struct tcs_cmd *cmd;
	int i, j;

	/* Convert all commands to RR when the request has wait_for_compl set */
	cmd_msgid |= msg->wait_for_compl ? CMD_MSGID_RESP_REQ : 0;

	for (i = 0, j = cmd_id; i < msg->num_cmds; i++, j++) {
		cmd = &msg->cmds[i];
		cmd_enable |= BIT(j);
		msgid = cmd_msgid;
		/*
		 * Additionally, if the cmd->wait is set, make the command
		 * response reqd even if the overall request was fire-n-forget.
		 */
		msgid |= cmd->wait ? CMD_MSGID_RESP_REQ : 0;

		write_tcs_cmd(drv, drv->regs[RSC_DRV_CMD_MSGID], tcs_id, j, msgid);
		write_tcs_cmd(drv, drv->regs[RSC_DRV_CMD_ADDR], tcs_id, j, cmd->addr);
		write_tcs_cmd(drv, drv->regs[RSC_DRV_CMD_DATA], tcs_id, j, cmd->data);
	}

	cmd_enable |= read_tcs_reg(drv, drv->regs[RSC_DRV_CMD_ENABLE], tcs_id);
	write_tcs_reg(drv, drv->regs[RSC_DRV_CMD_ENABLE], tcs_id, cmd_enable);
}

/**
 * get_tcs_for_msg() - Get the tcs_group used to send the given message.
 * @drv: The RSC controller.
 * @msg: The message we want to send.
 *
 * This is normally pretty straightforward except if we are trying to send
 * an ACTIVE_ONLY message but don't have any active_only TCSes.
 *
 * Return: A pointer to a tcs_group or an ERR_PTR.
 */
static struct tcs_group *get_tcs_for_msg(struct rsc_drv *drv,
					 const struct tcs_request *msg)
{
	int type;
	struct tcs_group *tcs;

	switch (msg->state) {
	case RPMH_ACTIVE_ONLY_STATE:
		type = ACTIVE_TCS;
		break;
	case RPMH_WAKE_ONLY_STATE:
		type = WAKE_TCS;
		break;
	case RPMH_SLEEP_STATE:
		type = SLEEP_TCS;
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	/*
	 * If we are making an active request on a RSC that does not have a
	 * dedicated TCS for active state use, then re-purpose a wake TCS to
	 * send active votes. This is safe because we ensure any active-only
	 * transfers have finished before we use it (maybe by running from
	 * the last CPU in PM code).
	 */
	tcs = &drv->tcs[type];
	if (msg->state == RPMH_ACTIVE_ONLY_STATE && !tcs->num_tcs)
		tcs = &drv->tcs[WAKE_TCS];

	return tcs;
}

/**
 * rpmh_rsc_send_data() - Write / trigger active-only message.
 * @drv: The controller.
 * @msg: The data to be sent.
 *
 * NOTES:
 * - This is only used for "ACTIVE_ONLY" since the limitations of this
 *   function don't make sense for sleep/wake cases.
 * - To do the transfer, we will grab a whole TCS for ourselves--we don't
 *   try to share. If there are none available we'll wait indefinitely
 *   for a free one.
 * - This function will not wait for the commands to be finished, only for
 *   data to be programmed into the RPMh. See rpmh_tx_done() which will
 *   be called when the transfer is fully complete.
 * - This function must be called with interrupts enabled. If the hardware
 *   is busy doing someone else's transfer we need that transfer to fully
 *   finish so that we can have the hardware, and to fully finish it needs
 *   the interrupt handler to run. If the interrupts is set to run on the
 *   active CPU this can never happen if interrupts are disabled.
 *
 * Return: 0 on success, -EINVAL on error.
 */
int rpmh_rsc_send_data(struct rsc_drv *drv, const struct tcs_request *msg)
{
	struct tcs_group *tcs;
	int tcs_id;

	tcs = get_tcs_for_msg(drv, msg);
	if (IS_ERR(tcs))
		return PTR_ERR(tcs);

	tcs->req[tcs_id - tcs->offset] = msg;
	set_bit(tcs_id, drv->tcs_in_use);
	if (msg->state == RPMH_ACTIVE_ONLY_STATE && tcs->type != ACTIVE_TCS) {
		/*
		 * Clear previously programmed WAKE commands in selected
		 * repurposed TCS to avoid triggering them. tcs->slots will be
		 * cleaned from rpmh_flush() by invoking rpmh_rsc_invalidate()
		 */
		write_tcs_reg_sync(drv, drv->regs[RSC_DRV_CMD_ENABLE], tcs_id, 0);
	}

	/*
	 * These two can be done after the lock is released because:
	 * - We marked "tcs_in_use" under lock.
	 * - Once "tcs_in_use" has been marked nobody else could be writing
	 *   to these registers until the interrupt goes off.
	 * - The interrupt can't go off until we trigger w/ the last line
	 *   of __tcs_set_trigger() below.
	 */
	__tcs_buffer_write(drv, tcs_id, 0, msg);
	__tcs_set_trigger(drv, tcs_id, true);

	return 0;
}

static int rpmh_probe_tcs_config(struct udevice *dev, struct rsc_drv *drv)
{
	struct tcs_type_config {
		u32 type;
		u32 n;
	} tcs_cfg[TCS_TYPE_NR] = { { 0 } };
	ofnode dn = dev->node_;
	u32 config, max_tcs, ncpt, offset;
	int i, ret, n, st = 0;
	struct tcs_group *tcs;

	ret = ofnode_read_u32(dn, "qcom,tcs-offset", &offset);
	if (ret)
		return ret;
	drv->tcs_base = drv->base + offset;

	config = readl_relaxed(drv->base + drv->regs[DRV_PRNT_CHLD_CONFIG]);

	max_tcs = config;
	max_tcs &= DRV_NUM_TCS_MASK << (DRV_NUM_TCS_SHIFT * drv->id);
	max_tcs = max_tcs >> (DRV_NUM_TCS_SHIFT * drv->id);

	ncpt = config & (DRV_NCPT_MASK << DRV_NCPT_SHIFT);
	ncpt = ncpt >> DRV_NCPT_SHIFT;

/* FIXME */
#if 0
	n = of_property_count_u32_elems(dn, "qcom,tcs-config");
	if (n != 2 * TCS_TYPE_NR)
		return -EINVAL;
#else
	n = 8;
#endif
	for (i = 0; i < TCS_TYPE_NR; i++) {
		ret = ofnode_read_u32_index(dn, "qcom,tcs-config",
						 i * 2, &tcs_cfg[i].type);
		if (ret)
			return ret;
		if (tcs_cfg[i].type >= TCS_TYPE_NR)
			return -EINVAL;

		ret = ofnode_read_u32_index(dn, "qcom,tcs-config",
						 i * 2 + 1, &tcs_cfg[i].n);
		if (ret)
			return ret;
		if (tcs_cfg[i].n > MAX_TCS_PER_TYPE)
			return -EINVAL;
	}

	for (i = 0; i < TCS_TYPE_NR; i++) {
		tcs = &drv->tcs[tcs_cfg[i].type];
		if (tcs->drv)
			return -EINVAL;
		tcs->drv = drv;
		tcs->type = tcs_cfg[i].type;
		tcs->num_tcs = tcs_cfg[i].n;
		tcs->ncpt = ncpt;

		if (!tcs->num_tcs || tcs->type == CONTROL_TCS)
			continue;

		if (st + tcs->num_tcs > max_tcs ||
		    st + tcs->num_tcs >= BITS_PER_BYTE * sizeof(tcs->mask))
			return -EINVAL;

		tcs->mask = ((1 << tcs->num_tcs) - 1) << st;
		tcs->offset = st;
		st += tcs->num_tcs;
	}

	drv->num_tcs = st;

	return 0;
}

static int rpmh_drv_load(struct udevice *dev, ulong addr, ulong size)
{
	return 0;
}

static int rpmh_drv_start(struct udevice *dev)
{
	return 0;
}

static int rpmh_drv_stop(struct udevice *dev)
{
	return 0;
}

static int rpmh_rsc_probe(struct udevice *dev)
{
	struct rsc_drv *drv = dev_get_priv(dev);
	char drv_id[10] = {0};
	int ret;
	u32 rsc_id;
	ofnode dn;

	printf("%s: Entering function\n", __func__);
	dn = dev->node_;
	ret = ofnode_read_u32(dn, "qcom,drv-id", &drv->id);
	if (ret)
		return ret;

	snprintf(drv_id, ARRAY_SIZE(drv_id), "drv-%d", drv->id);
	drv->base = (void __iomem *)dev_read_addr_name(dev, drv_id);
	if (IS_ERR(drv->base))
		return PTR_ERR(drv->base);

	rsc_id = readl_relaxed(drv->base + RSC_DRV_ID);
	drv->ver.major = rsc_id & (MAJOR_VER_MASK << MAJOR_VER_SHIFT);
	drv->ver.major >>= MAJOR_VER_SHIFT;
	drv->ver.minor = rsc_id & (MINOR_VER_MASK << MINOR_VER_SHIFT);
	drv->ver.minor >>= MINOR_VER_SHIFT;

	printf("%s: drv->ver.major = %d\n", __func__, drv->ver.major);
	if (drv->ver.major == 3)
		drv->regs = rpmh_rsc_reg_offset_ver_3_0;
	else
		drv->regs = rpmh_rsc_reg_offset_ver_2_7;

	ret = rpmh_probe_tcs_config(dev, drv);
	if (ret)
		return ret;

	printk("BHUPESH: %s: writing: 0x%x at 0x%p\n",
		  __func__, drv->tcs[ACTIVE_TCS].mask, drv->tcs_base + drv->regs[RSC_DRV_IRQ_ENABLE]);

	/* Enable the active TCS to send requests immediately */
	writel_relaxed(drv->tcs[ACTIVE_TCS].mask,
		       drv->tcs_base + drv->regs[RSC_DRV_IRQ_ENABLE]);
writel(0x3 , 0x0000000018220d00);
writel(0x10108 , 0x0000000018220d30);
writel(0x300a0 , 0x0000000018220d34);
writel(0x3 , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x0 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x30080 , 0x0000000018220d34);
writel(0x5 , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10008 , 0x0000000018220d30);
writel(0x30080 , 0x0000000018220d34);
writel(0x5 , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x30010 , 0x0000000018220d34);
writel(0x8 , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x30000 , 0x0000000018220d34);
writel(0x8 , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30010 , 0x0000000018220d34);
writel( 0x8 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x30000 , 0x0000000018220fd4);
writel( 0x8 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x0 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x40508 , 0x0000000018220d34);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x6 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x40408 , 0x0000000018220d34);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220fd0);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x42e08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel( 0x30080 , 0x0000000018220fd4);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x0 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel( 0x10108 , 0x0000000018220d30);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x41d08 , 0x0000000018220d34);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x7 , 0x0000000018220d38);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220fd0);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x41e08 , 0x0000000018220d34);
writel(0x0 , 0x0000000018220fb4);
writel( 0x7 , 0x0000000018220d38);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x1010000 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220fbc);
writel(0x0 , 0x0000000018220d14);
writel( 0x10008 , 0x0000000018220fd0);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x40e08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel( 0x30080 , 0x0000000018220fd4);
writel(0x10000 , 0x0000000018220d14);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x1010000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220d1c);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x40f08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x41008 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel( 0x5 , 0x0000000018220fd8);
writel(0x0 , 0x0000000018220d14);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x41108 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x42f08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x41208 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x41308 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x41408 , 0x0000000018220d34);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel( 0x10008 , 0x0000000018220fd0);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x40108 , 0x0000000018220d34);
writel( 0x43308 , 0x0000000018220fd4);
writel( 0x7 , 0x0000000018220d38);
writel( 0x7 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x40108 , 0x0000000018220d34);
writel( 0x43308 , 0x0000000018220fd4);
writel( 0x7 , 0x0000000018220d38);
writel( 0x7 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x40208 , 0x0000000018220d34);
writel( 0x41f08 , 0x0000000018220fd4);
writel( 0x7 , 0x0000000018220d38);
writel( 0x7 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40908 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x43408 , 0x0000000018220fd4);
writel( 0x7 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x41708 , 0x0000000018220d34);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x41508 , 0x0000000018220fd4);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x7 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41908 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220fd0);
writel(0x10000 , 0x0000000018220d14);
writel( 0x41608 , 0x0000000018220fd4);
writel(0x0 , 0x0000000018220d14);
writel( 0x7 , 0x0000000018220fd8);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x1010000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220d1c);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41b08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel( 0x10108 , 0x0000000018220fd0);
writel(0x0 , 0x0000000018220d14);
writel( 0x43008 , 0x0000000018220fd4);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x7 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x42008 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41c08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40a08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x42108 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40b08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x42208 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x42308 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40c08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40d08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41a08 , 0x0000000018220d34);
writel( 0x7 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40500 , 0x0000000018220fd4);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0xbd8 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220fd0);
writel(0x0 , 0x0000000018220d14);
writel( 0x40504 , 0x0000000018220fd4);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40d00 , 0x0000000018220fd4);
writel( 0xb28 , 0x0000000018220fd8);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x40d04 , 0x0000000018220fd4);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1 , 0x0000000018220fd8);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x0 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x10108 , 0x0000000018220d30);
writel(0x0 , 0x0000000018220fb4);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel(0x0 , 0x0000000018220fb4);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel( 0x40400 , 0x0000000018220fd4);
writel(0x0 , 0x0000000018220d14);
writel( 0x548 , 0x0000000018220fd8);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x40404 , 0x0000000018220fd4);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x1 , 0x0000000018220fd8);
writel( 0x10008 , 0x0000000018220d30);
writel( 0x30080 , 0x0000000018220d34);
writel( 0x5 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x50074 , 0x0000000018220d34);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x10008 , 0x0000000018220fd0);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x30080 , 0x0000000018220fd4);
writel( 0x5 , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x41b00 , 0x0000000018220d34);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x4b0 , 0x0000000018220d38);
writel( 0x50074 , 0x0000000018220fd4);
writel( 0x1 , 0x0000000018220d1c);
writel( 0x6fffffff , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x50074 , 0x0000000018220fd4);
writel( 0x6fffffff , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x50074 , 0x0000000018220fd4);
writel( 0x6fffffff , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x50074 , 0x0000000018220fd4);
writel( 0x6fffffff , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220fd0);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41b04 , 0x0000000018220d34);
writel( 0x1 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x50074 , 0x0000000018220fd4);
writel( 0x6fffffff , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40200 , 0x0000000018220d34);
writel( 0x398 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x40204 , 0x0000000018220d34);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x1 , 0x0000000018220d38);
writel( 0x50074 , 0x0000000018220fd4);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x6fffffff , 0x0000000018220fd8);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x41900 , 0x0000000018220d34);
writel( 0x370 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220d14);
writel( 0x10108 , 0x0000000018220fd0);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x50074 , 0x0000000018220fd4);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41904 , 0x0000000018220d34);
writel( 0x1 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41c00 , 0x0000000018220d34);
writel( 0x4b0 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41c04 , 0x0000000018220d34);
writel( 0x1 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40900 , 0x0000000018220d34);
writel( 0xc00 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40904 , 0x0000000018220d34);
writel( 0x1 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220fbc);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40100 , 0x0000000018220d34);
writel( 0x770 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x40104 , 0x0000000018220d34);
writel( 0x1 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x42100 , 0x0000000018220d34);
writel( 0x708 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x42104 , 0x0000000018220d34);
writel( 0x1 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x6fffffff , 0x0000000018220fd8);
writel( 0x1 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41a00 , 0x0000000018220d34);
writel( 0x390 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel( 0x40204 , 0x0000000018220d34);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x41a04 , 0x0000000018220d34);
writel( 0x1 , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel( 0x1010000 , 0x0000000018220fb4);
writel( 0x0 , 0x0000000018220fbc);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);
writel( 0x6fffffff , 0x0000000018220d38);
writel( 0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel( 0x1010000 , 0x0000000018220d14);
writel( 0x0 , 0x0000000018220d1c);
writel( 0x10108 , 0x0000000018220d30);
writel( 0x50074 , 0x0000000018220d34);

writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50074 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50044 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50048 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x500a4 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x5004c , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50030 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50028 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x5002c , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50034 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50090 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50090 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x5008c , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50038 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50000 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x10108 , 0x0000000018220d44);
writel(0x5007c , 0x0000000018220d48);
writel(0x40000000 , 0x0000000018220d4c);
writel(0x3 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50088 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x5006c , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50088 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50060 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50058 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x500a0 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50098 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50098 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x10108 , 0x0000000018220d30);
writel(0x50090 , 0x0000000018220d34);
writel(0x50098 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x10108 , 0x0000000018220d30);
writel(0x50090 , 0x0000000018220d34);
writel(0x50098 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50098 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x500a0 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x500a0 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x5009c , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x50094 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x10008 , 0x0000000018220fd0);
writel(0x5000c , 0x0000000018220d34);
writel(0x41a04 , 0x0000000018220fd4);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x0 , 0x0000000018220fd8);
writel(0x1 , 0x0000000018220fbc);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220fb4);
writel(0x10108 , 0x0000000018220d30);
writel(0x1010000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fbc);
writel(0x5000c , 0x0000000018220d34);
writel(0x10008 , 0x0000000018220fd0);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x10108 , 0x0000000018220d30);
writel(0x42104 , 0x0000000018220fd4);
writel(0x50014 , 0x0000000018220d34);
writel(0x0 , 0x0000000018220fd8);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);
writel(0x1 , 0x0000000018220fbc);
writel(0x10108 , 0x0000000018220d30);
writel(0x10000 , 0x0000000018220fb4);
writel(0x0 , 0x0000000018220fb4);
writel(0x10000 , 0x0000000018220fb4);
writel(0x1010000 , 0x0000000018220fb4);
writel(0x50010 , 0x0000000018220d34);
writel(0x6fffffff , 0x0000000018220d38);
writel(0x1 , 0x0000000018220d1c);
writel(0x10000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d14);
writel(0x10000 , 0x0000000018220d14);
writel(0x1010000 , 0x0000000018220d14);
writel(0x0 , 0x0000000018220d1c);

	printf("%s: Exiting function successfully\n", __func__);
	return 0;
}

static const struct dm_rproc_ops rpmh_drv_ops = {
	.load = rpmh_drv_load,
	.start = rpmh_drv_start,
	.stop =  rpmh_drv_stop,
};

static const struct udevice_id rpmh_drv_ids[] = {
	{ .compatible = "qcom,rpmh-rsc" },
	{ }
};

U_BOOT_DRIVER(rpmh_rsc) = {
	.name = "rpmh-rsc",
	.id = UCLASS_REMOTEPROC,
	.of_match = rpmh_drv_ids,
	.ops = &rpmh_drv_ops,
	.probe = rpmh_rsc_probe,
	.priv_auto = sizeof(struct rsc_drv),
};
