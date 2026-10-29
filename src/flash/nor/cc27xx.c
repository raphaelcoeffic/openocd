// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * TI CC27xx flash driver (SimpleLink Low Power F3, also CC23x1).
 *
 * Banks: main flash, the CCFG and the SCFG sector. Two ways to program them:
 *
 * - On a booted device, halted: the boot ROM's flash functions (HAPI table),
 *   called on the target, erase and program per sector. No chip erase, the
 *   HSM firmware and the other sectors are left alone. The sectors the boot
 *   ROM protects (CCFG.flashProt, often the CCFG and SCFG themselves) can't
 *   be changed this way until a chip erase.
 *
 * - While the boot ROM is in SACI, its command interface on the SEC-AP (blank
 *   device, after "cc27xx mass_erase"): SACI programming commands. Only a
 *   chip erase erases there; erasing checks that the sectors are blank.
 *
 * With SWD connected, the boot ROM stops in SACI after any reset; the AHB-AP
 * is closed until a SACI command lets the boot go on. The "cc27xx
 * reset_assert" and "cc27xx saci_exit" commands do the reset for the target's
 * reset events: reset through the SEC-AP, then leave SACI and either run or
 * halt at the application's first instruction. A device without a valid
 * CCFG, SCFG or application stays in SACI.
 *
 * Reference: CC27xx Technical Reference Manual (SWCU195), driverlib hapi.h.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/crc32.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/arm_adi_v5.h>
#include <target/armv7m.h>
#include <target/breakpoints.h>
#include <target/cortex_m.h>
#include <target/register.h>

#define CC27XX_SECTOR_SIZE		0x800
#define CC27XX_MAIN_BASE		0x00000000
#define CC27XX_CCFG_BASE		0x4e020000
#define CC27XX_CCFG_SIZE		0x800
#define CC27XX_SCFG_BASE		0x4e040000
#define CC27XX_SCFG_SIZE		0x400
/* CCFG bootCfg: the user boot loader's and the application's vector tables */
#define CC27XX_CCFG_BLDR_VTOR		(CC27XX_CCFG_BASE + 0x0)
#define CC27XX_CCFG_APP_VTOR		(CC27XX_CCFG_BASE + 0x8)

/* Boot ROM and its function table (driverlib hapi.h) */
#define CC27XX_ROM_BASE			0x0f000000
#define CC27XX_HAPI_TABLE		0x0f00004c
#define CC27XX_HAPI_SECTOR_ERASE	3	/* (key, address) */
#define CC27XX_HAPI_PROGRAM		5	/* (key, buffer, address, count) */
#define CC27XX_FLASH_API_KEY		0xb7e3a08f
#define CC27XX_FAPI_STATUS_SUCCESS	0

#define CC27XX_VIMS_BASE		0x40024000
#define CC27XX_VIMS_DESCEX		(CC27XX_VIMS_BASE + 0x004)
#define CC27XX_VIMS_WEPRA		(CC27XX_VIMS_BASE + 0x410)
#define CC27XX_VIMS_WEPRB0		(CC27XX_VIMS_BASE + 0x414)
#define CC27XX_VIMS_WEPRB1		(CC27XX_VIMS_BASE + 0x418)
#define CC27XX_VIMS_WEPRAUX		(CC27XX_VIMS_BASE + 0x41c)
#define CC27XX_VIMS_WEPRAUX_CCFG	BIT(0)
#define CC27XX_VIMS_WEPRAUX_SCFG	BIT(3)
#define CC27XX_VIMS_CCHCTL		(CC27XX_VIMS_BASE + 0x424)
#define CC27XX_VIMS_CCHSTA		(CC27XX_VIMS_BASE + 0x428)
#define CC27XX_VIMS_CCHSTA_BUSY		BIT(1)

/* DESCEX: flash and ROM size in KB, minus one */
#define CC27XX_DESCEX_FLSZ(v)		(((v) >> 15) & 0xfff)
#define CC27XX_DESCEX_ROMSZ(v)		((v) & 0x7fff)

/* CCFG: CRC32 of each record, stored right after it */
#define CC27XX_CCFG_BOOT_START		0x000
#define CC27XX_CCFG_BOOT_LEN		0x00c
#define CC27XX_CCFG_CENTRAL_START	0x010
#define CC27XX_CCFG_CENTRAL_LEN		0x73c
#define CC27XX_CCFG_DEBUG_START		0x7d0
#define CC27XX_CCFG_DEBUG_LEN		0x02c

/* CFG-AP: device identification and boot status */
#define CC27XX_CFG_AP			1
#define CC27XX_CFG_AP_DEVICEID		0x0
#define CC27XX_CFG_AP_PARTID		0x4
#define CC27XX_CFG_AP_DEVICESTATUS	0xc
#define CC27XX_BOOTSTA(status)		(((status) >> 8) & 0xff)

/* SEC-AP: the SACI mailbox, and a system reset */
#define CC27XX_SEC_AP			2
#define CC27XX_SEC_AP_TXD		0x00
#define CC27XX_SEC_AP_TXCTL		0x04
#define CC27XX_SEC_AP_RXD		0x08
#define CC27XX_SEC_AP_RXCTL		0x0c
#define CC27XX_SEC_AP_RSTCTL		0x18
#define CC27XX_TXCTL_TXD_FULL		BIT(0)
#define CC27XX_TXCTL_CMD_START		BIT(1)
#define CC27XX_RXCTL_RXD_FULL		BIT(0)
#define CC27XX_RSTCTL_RESET		0x5c

/* SACI commands: first word = ID | sequence << 8 | command specific << 16 */
#define CC27XX_SACI_NOP			0x01
#define CC27XX_SACI_EXIT_SACI_HALT	0x07
#define CC27XX_SACI_ERASE_CHIP		0x09
#define CC27XX_SACI_PROG_CCFG		0x0c
#define CC27XX_SACI_PROG_MAIN_SECTOR	0x0e
#define CC27XX_SACI_VERIFY_MAIN		0x10
#define CC27XX_SACI_VERIFY_CCFG		0x11
#define CC27XX_SACI_EXIT_SACI_RUN	0x15
#define CC27XX_SACI_PROG_SCFG		0x1a
#define CC27XX_SACI_ERASE_MAIN_APP	0x1c
#define CC27XX_SACI_RETAIN_SECTORS	BIT(16)	/* ERASE_CHIP, ERASE_MAIN_APP */
#define CC27XX_SACI_BLANK_CHECK		BIT(31)	/* VERIFY_MAIN, VERIFY_CCFG */
#define CC27XX_SACI_MAX_WORDS		(3 + CC27XX_SECTOR_SIZE / 4)
/* Response: ID | sequence << 8 | result << 16 | data words << 24 */
#define CC27XX_SACI_SUCCESS		0x00
#define CC27XX_SACI_NOT_ALLOWED		0x86
#define CC27XX_SACI_BLANK_CHECK_FAILED	0x89

/* Boot status (TRM table 9-2) */
#define CC27XX_BOOTSTA_SACI		0x20
#define CC27XX_BOOTSTA_BLDR_WAITLOOP	0x81
#define CC27XX_BOOTSTA_APP_WAITLOOP	0xc1

#define CC27XX_SACI_TIMEOUT_MS		1000
#define CC27XX_SACI_ERASE_TIMEOUT_MS	10000
#define CC27XX_ROM_CALL_TIMEOUT_MS	5000
/* Instructions from the boot ROM's wait loop to the application: 7 seen */
#define CC27XX_MAX_ROM_STEPS		64

/* Working area: two bkpt (the ROM calls' return), the stack, a sector buffer */
#define CC27XX_WA_STACK_SIZE		0x200
#define CC27XX_WA_BUFFER		(4 + CC27XX_WA_STACK_SIZE)
#define CC27XX_WA_SIZE			(CC27XX_WA_BUFFER + CC27XX_SECTOR_SIZE)

struct cc27xx_bank {
	bool probed;
	uint32_t config_size;
	uint32_t device_id;
	uint32_t part_id;
	uint32_t flash_kb;
	uint32_t sector_erase;
	uint32_t program;
	/* Main bank: the boot ROM is kept in SACI, don't leave it */
	bool saci_hold;
	/* Main bank: examined in SACI by the poller (AHB-AP reads as 0) */
	bool reexamine;
};

static const struct {
	uint8_t code;
	const char *name;
} cc27xx_saci_results[] = {
	{ 0x80, "invalid command" },
	{ 0x81, "invalid address" },
	{ 0x82, "invalid size" },
	{ 0x83, "invalid key" },
	{ 0x84, "flash FSM error" },
	{ 0x85, "buffer overflow" },
	{ CC27XX_SACI_NOT_ALLOWED, "not allowed" },
	{ 0x87, "CRC32 mismatch" },
	{ CC27XX_SACI_BLANK_CHECK_FAILED, "not blank" },
	{ 0x8e, "HSM failed to boot" },
};

static const char *cc27xx_saci_result_name(uint8_t result)
{
	for (size_t i = 0; i < ARRAY_SIZE(cc27xx_saci_results); i++)
		if (cc27xx_saci_results[i].code == result)
			return cc27xx_saci_results[i].name;
	return "failed";
}

/* The main bank's data holds the target's SACI state */
static struct cc27xx_bank *cc27xx_target_bank(struct target *target)
{
	for (struct flash_bank *bank = flash_bank_list(); bank; bank = bank->next)
		if (bank->driver == &cc27xx_flash && bank->target == target &&
			bank->base == CC27XX_MAIN_BASE)
			return bank->driver_priv;
	return NULL;
}

static void cc27xx_set_saci_hold(struct target *target, bool hold)
{
	struct cc27xx_bank *cc27xx_bank = cc27xx_target_bank(target);
	if (!cc27xx_bank)
		return;
	cc27xx_bank->saci_hold = hold;
	if (hold)
		cc27xx_bank->reexamine = true;
}

static int cc27xx_ap_read(struct target *target, unsigned int ap_num,
	unsigned int reg, uint32_t *value)
{
	struct adiv5_dap *dap = target_to_armv7m(target)->arm.dap;
	struct adiv5_ap *ap = dap_get_ap(dap, ap_num);
	if (!ap)
		return ERROR_FAIL;

	int retval = dap_queue_ap_read(ap, reg, value);
	if (retval == ERROR_OK)
		retval = dap_run(dap);
	dap_put_ap(ap);
	return retval;
}

static int cc27xx_ap_write(struct target *target, unsigned int ap_num,
	unsigned int reg, uint32_t value)
{
	struct adiv5_dap *dap = target_to_armv7m(target)->arm.dap;
	struct adiv5_ap *ap = dap_get_ap(dap, ap_num);
	if (!ap)
		return ERROR_FAIL;

	int retval = dap_queue_ap_write(ap, reg, value);
	if (retval == ERROR_OK)
		retval = dap_run(dap);
	dap_put_ap(ap);
	return retval;
}

static int cc27xx_bootsta(struct target *target, uint8_t *bootsta)
{
	uint32_t status;
	int retval = cc27xx_ap_read(target, CC27XX_CFG_AP,
		CC27XX_CFG_AP_DEVICESTATUS, &status);
	if (retval != ERROR_OK)
		return retval;

	*bootsta = CC27XX_BOOTSTA(status);
	return ERROR_OK;
}

static bool cc27xx_in_saci(struct target *target)
{
	uint8_t bootsta;
	return cc27xx_bootsta(target, &bootsta) == ERROR_OK &&
		bootsta == CC27XX_BOOTSTA_SACI;
}

/* Wait until the boot status is one of two values */
static int cc27xx_wait_bootsta(struct target *target, uint8_t a, uint8_t b,
	uint8_t *bootsta)
{
	int64_t start = timeval_ms();
	int retval;

	do {
		retval = cc27xx_bootsta(target, bootsta);
		if (retval == ERROR_OK && (*bootsta == a || *bootsta == b))
			return ERROR_OK;
		alive_sleep(1);
	} while (timeval_ms() - start < CC27XX_SACI_TIMEOUT_MS);

	return retval == ERROR_OK ? ERROR_TIMEOUT_REACHED : retval;
}

/* Wait until a SEC-AP flag (TXD_FULL or RXD_FULL) has the value */
static int cc27xx_saci_wait(struct target *target, unsigned int reg,
	bool full, unsigned int timeout_ms)
{
	int64_t start = timeval_ms();
	uint32_t ctl;

	do {
		int retval = cc27xx_ap_read(target, CC27XX_SEC_AP, reg, &ctl);
		if (retval != ERROR_OK)
			return retval;
		if (!!(ctl & BIT(0)) == full)
			return ERROR_OK;
		alive_sleep(1);
	} while (timeval_ms() - start < timeout_ms);

	LOG_TARGET_ERROR(target, "SACI timeout (%s)",
		reg == CC27XX_SEC_AP_TXCTL ? "command" : "response");
	return ERROR_TIMEOUT_REACHED;
}

/* Send a command (TRM 9.3.1.1); the words from the third on go in one batch */
static int cc27xx_saci_send(struct target *target, const uint32_t *words,
	unsigned int count)
{
	int retval = cc27xx_saci_wait(target, CC27XX_SEC_AP_TXCTL, false,
		CC27XX_SACI_TIMEOUT_MS);
	if (retval == ERROR_OK)
		retval = cc27xx_ap_write(target, CC27XX_SEC_AP, CC27XX_SEC_AP_TXCTL,
			CC27XX_TXCTL_CMD_START);
	if (retval == ERROR_OK)
		retval = cc27xx_ap_write(target, CC27XX_SEC_AP, CC27XX_SEC_AP_TXD, words[0]);
	if (retval != ERROR_OK || count == 1)
		goto done;

	retval = cc27xx_saci_wait(target, CC27XX_SEC_AP_TXCTL, false,
		CC27XX_SACI_TIMEOUT_MS);
	if (retval == ERROR_OK)
		retval = cc27xx_ap_write(target, CC27XX_SEC_AP, CC27XX_SEC_AP_TXCTL, 0);
	if (retval == ERROR_OK)
		retval = cc27xx_ap_write(target, CC27XX_SEC_AP, CC27XX_SEC_AP_TXD, words[1]);
	if (retval != ERROR_OK || count == 2)
		goto done;

	retval = cc27xx_saci_wait(target, CC27XX_SEC_AP_TXCTL, false,
		CC27XX_SACI_TIMEOUT_MS);
	if (retval != ERROR_OK)
		return retval;

	struct adiv5_dap *dap = target_to_armv7m(target)->arm.dap;
	struct adiv5_ap *ap = dap_get_ap(dap, CC27XX_SEC_AP);
	if (!ap)
		return ERROR_FAIL;
	for (unsigned int i = 2; i < count && retval == ERROR_OK; i++)
		retval = dap_queue_ap_write(ap, CC27XX_SEC_AP_TXD, words[i]);
	if (retval == ERROR_OK)
		retval = dap_run(dap);
	dap_put_ap(ap);

done:
	if (retval == ERROR_OK)
		retval = cc27xx_saci_wait(target, CC27XX_SEC_AP_TXCTL, false,
			CC27XX_SACI_TIMEOUT_MS);
	return retval;
}

/*
 * Run a SACI command: send it, read its response. The result code is in
 * *result, the response data (at most max words) in data.
 */
static int cc27xx_saci_call(struct target *target, uint32_t *words,
	unsigned int count, uint32_t *data, unsigned int max,
	unsigned int timeout_ms, uint8_t *result)
{
	static uint8_t sequence;
	uint32_t header;

	words[0] = (words[0] & ~0xff00) | (sequence++ << 8);
	int retval = cc27xx_saci_send(target, words, count);
	if (retval != ERROR_OK)
		return retval;

	/* Skip a response left by an earlier command */
	do {
		retval = cc27xx_saci_wait(target, CC27XX_SEC_AP_RXCTL, true, timeout_ms);
		if (retval == ERROR_OK)
			retval = cc27xx_ap_read(target, CC27XX_SEC_AP, CC27XX_SEC_AP_RXD, &header);
		if (retval != ERROR_OK)
			return retval;
		if ((header & 0xffff) != (words[0] & 0xffff))
			LOG_TARGET_DEBUG(target, "SACI response 0x%08" PRIx32 " skipped", header);
	} while ((header & 0xffff) != (words[0] & 0xffff));

	for (unsigned int i = 0; i < (header >> 24); i++) {
		uint32_t word;
		retval = cc27xx_saci_wait(target, CC27XX_SEC_AP_RXCTL, true,
			CC27XX_SACI_TIMEOUT_MS);
		if (retval == ERROR_OK)
			retval = cc27xx_ap_read(target, CC27XX_SEC_AP, CC27XX_SEC_AP_RXD, &word);
		if (retval != ERROR_OK)
			return retval;
		if (i < max)
			data[i] = word;
	}

	*result = (header >> 16) & 0xff;
	LOG_TARGET_DEBUG(target, "SACI command 0x%02" PRIx32 ": result 0x%02" PRIx8,
		words[0] & 0xff, *result);
	return ERROR_OK;
}

/* A SACI command that must succeed */
static int cc27xx_saci_run(struct target *target, uint32_t *words,
	unsigned int count, unsigned int timeout_ms)
{
	uint8_t result;
	int retval = cc27xx_saci_call(target, words, count, NULL, 0, timeout_ms, &result);
	if (retval != ERROR_OK)
		return retval;
	if (result != CC27XX_SACI_SUCCESS) {
		LOG_TARGET_ERROR(target, "SACI command 0x%02" PRIx32 " failed: %s (0x%02" PRIx8 ")",
			words[0] & 0xff, cc27xx_saci_result_name(result), result);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

/*
 * Reset through the SEC-AP (works in any state, without the AHB-AP), wait
 * for the boot ROM to enter SACI, and keep it there (a first command stops
 * its timeout).
 */
static int cc27xx_reset_to_saci(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	uint8_t bootsta;

	/* The reset may cut the write's acknowledge */
	cc27xx_ap_write(target, CC27XX_SEC_AP, CC27XX_SEC_AP_RSTCTL, CC27XX_RSTCTL_RESET);

	if (armv7m->arm.core_cache)
		register_cache_invalidate(armv7m->arm.core_cache);
	target->state = TARGET_RESET;
	cc27xx_set_saci_hold(target, false);

	/* The SWD connection survives the reset; clear the errors it left once
	 * the DP answers again */
	int64_t start = timeval_ms();
	int retval;
	do {
		alive_sleep(10);
		retval = dap_dp_init_or_reconnect(armv7m->arm.dap);
	} while (retval != ERROR_OK && timeval_ms() - start < CC27XX_SACI_TIMEOUT_MS);
	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "no DP after the reset");
		return retval;
	}

	retval = cc27xx_wait_bootsta(target, CC27XX_BOOTSTA_SACI,
		CC27XX_BOOTSTA_SACI, &bootsta);
	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "no SACI after the reset (boot status 0x%02" PRIx8 ")",
			bootsta);
		return retval;
	}

	uint32_t nop = CC27XX_SACI_NOP;
	return cc27xx_saci_run(target, &nop, 1, CC27XX_SACI_TIMEOUT_MS);
}

COMMAND_HANDLER(cc27xx_handle_reset_assert_command)
{
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (!is_armv7m(target_to_armv7m(target)))
		return ERROR_TARGET_INVALID;

	return cc27xx_reset_to_saci(target);
}

/* Run from the boot ROM's wait loop to the entry in the CCFG's vector table */
static int cc27xx_run_to_entry(struct target *target, uint8_t bootsta)
{
	uint32_t vtor, entry;

	int retval = target_read_u32(target, bootsta == CC27XX_BOOTSTA_BLDR_WAITLOOP ?
		CC27XX_CCFG_BLDR_VTOR : CC27XX_CCFG_APP_VTOR, &vtor);
	if (retval == ERROR_OK && vtor < CC27XX_ROM_BASE && !(vtor & 3))
		retval = target_read_u32(target, vtor + 4, &entry);
	else
		retval = ERROR_FAIL;
	if (retval != ERROR_OK || (entry & ~1) >= CC27XX_ROM_BASE)
		return ERROR_FAIL;

	entry &= ~1;
	retval = breakpoint_add(target, entry, 2, BKPT_HARD);
	if (retval != ERROR_OK)
		return retval;

	retval = target_resume(target, true, 0, false, false);
	if (retval == ERROR_OK)
		retval = target_wait_state(target, TARGET_HALTED, CC27XX_SACI_TIMEOUT_MS);
	breakpoint_remove(target, entry);
	if (retval != ERROR_OK) {
		target_halt(target);
		target_wait_state(target, TARGET_HALTED, CC27XX_SACI_TIMEOUT_MS);
		return ERROR_FAIL;
	}

	struct armv7m_common *armv7m = target_to_armv7m(target);
	if (buf_get_u32(armv7m->arm.pc->value, 0, 32) != entry)
		return ERROR_FAIL;
	LOG_TARGET_DEBUG(target, "application entry 0x%08" PRIx32, entry);
	return ERROR_OK;
}

/*
 * Halt in the boot ROM's wait loop, then run to the application's entry
 * (or the user boot loader's), or step there when the CCFG doesn't tell it.
 */
static int cc27xx_halt_at_entry(struct target *target, uint8_t bootsta)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	uint8_t buf[4];

	int retval = mem_ap_write_atomic_u32(armv7m->debug_ap, DCB_DHCSR,
		DBGKEY | C_DEBUGEN | C_HALT);
	if (retval != ERROR_OK)
		return retval;
	target->debug_reason = DBG_REASON_DBGRQ;

	/* Twice: out of TARGET_RESET, then the halt */
	retval = target_poll(target);
	if (retval == ERROR_OK && target->state != TARGET_HALTED)
		retval = target_poll(target);
	if (retval != ERROR_OK)
		return retval;
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted in the boot ROM");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* R3 = 0 lets the boot ROM leave its debugger wait loop */
	struct reg *r3 = register_get_by_name(armv7m->arm.core_cache, "r3", false);
	if (!r3)
		return ERROR_FAIL;
	buf_set_u32(buf, 0, 32, 0);
	retval = r3->type->set(r3, buf);
	if (retval != ERROR_OK)
		return retval;

	if (cc27xx_run_to_entry(target, bootsta) == ERROR_OK)
		return ERROR_OK;

	uint32_t descex;
	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, CC27XX_VIMS_DESCEX, &descex);
	if (retval != ERROR_OK)
		return retval;
	uint32_t rom_end = CC27XX_ROM_BASE + (CC27XX_DESCEX_ROMSZ(descex) + 1) * 1024;

	for (int i = 0; i < CC27XX_MAX_ROM_STEPS; i++) {
		uint32_t pc = buf_get_u32(armv7m->arm.pc->value, 0, 32);
		if (pc < CC27XX_ROM_BASE || pc >= rom_end) {
			LOG_TARGET_DEBUG(target, "application entry 0x%08" PRIx32 " after %d steps",
				pc, i);
			return ERROR_OK;
		}
		retval = target_step(target, true, 0, false);
		if (retval != ERROR_OK)
			return retval;
	}

	LOG_TARGET_WARNING(target, "still in the boot ROM after %d steps",
		CC27XX_MAX_ROM_STEPS);
	return ERROR_OK;
}

/*
 * Leave SACI after a reset: halt at the application's entry on a reset halt
 * or init, else run. Nothing to do when the boot ROM isn't in SACI, or is
 * kept there.
 */
COMMAND_HANDLER(cc27xx_handle_saci_exit_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cc27xx_bank *cc27xx_bank = cc27xx_target_bank(target);
	uint8_t bootsta, result;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (!is_armv7m(target_to_armv7m(target)))
		return ERROR_TARGET_INVALID;

	int retval = cc27xx_bootsta(target, &bootsta);
	if (retval != ERROR_OK)
		return retval;
	if (bootsta != CC27XX_BOOTSTA_SACI) {
		LOG_TARGET_DEBUG(target, "not in SACI (boot status 0x%02" PRIx8 ")", bootsta);
		return ERROR_OK;
	}
	if (cc27xx_bank && cc27xx_bank->saci_hold) {
		LOG_TARGET_DEBUG(target, "kept in SACI");
		return ERROR_OK;
	}

	bool halt = target->reset_halt;
	LOG_TARGET_DEBUG(target, "SACI exit, %s", halt ? "halt" : "run");
	uint32_t command = halt ? CC27XX_SACI_EXIT_SACI_HALT : CC27XX_SACI_EXIT_SACI_RUN;
	retval = cc27xx_saci_call(target, &command, 1, NULL, 0, CC27XX_SACI_TIMEOUT_MS,
		&result);
	if (retval != ERROR_OK)
		return retval;
	if (result != CC27XX_SACI_SUCCESS) {
		/* Blank, or a SACI flash command since the reset */
		LOG_TARGET_WARNING(target, "the boot ROM stays in SACI (%s): no valid CCFG, SCFG "
			"or application; flash writes go through SACI until the next reset",
			cc27xx_saci_result_name(result));
		cc27xx_set_saci_hold(target, true);
		return ERROR_OK;
	}
	if (halt) {
		retval = cc27xx_wait_bootsta(target, CC27XX_BOOTSTA_APP_WAITLOOP,
			CC27XX_BOOTSTA_BLDR_WAITLOOP, &bootsta);
		if (retval != ERROR_OK) {
			LOG_TARGET_ERROR(target, "boot ROM not waiting for the debugger (boot status 0x%02" PRIx8 ")",
				bootsta);
			return retval;
		}
	}

	/* Not examined yet, or examined while in SACI: the AHB-AP is open now */
	if (!target_was_examined(target) || (cc27xx_bank && cc27xx_bank->reexamine)) {
		if (cc27xx_bank)
			cc27xx_bank->reexamine = false;
		target->examined = false;
		retval = target_examine_one(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return halt ? cc27xx_halt_at_entry(target, bootsta) : ERROR_OK;
}

/*
 * Chip erase through SACI, keeping the HSM firmware. The device stays in
 * SACI until the next reset: the flash writes until then go through SACI.
 */
COMMAND_HANDLER(cc27xx_handle_mass_erase_command)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t retained[3] = { 0 };
	uint8_t result;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (!is_armv7m(target_to_armv7m(target)))
		return ERROR_TARGET_INVALID;

	int retval = cc27xx_reset_to_saci(target);
	if (retval != ERROR_OK)
		return retval;
	cc27xx_set_saci_hold(target, true);

	/* Retaining needs a valid CCFG; it always keeps the HSM firmware */
	uint32_t erase[2] = {
		CC27XX_SACI_ERASE_CHIP | CC27XX_SACI_RETAIN_SECTORS, CC27XX_FLASH_API_KEY
	};
	retval = cc27xx_saci_call(target, erase, 2, retained, ARRAY_SIZE(retained),
		CC27XX_SACI_ERASE_TIMEOUT_MS, &result);
	if (retval != ERROR_OK)
		return retval;

	if (result == CC27XX_SACI_NOT_ALLOWED) {
		/* No valid CCFG: a blank one is kept, ERASE_MAIN_APP always keeps the
		 * HSM firmware; else only a chip erase without retaining is left */
		uint32_t blank_check[5] = { CC27XX_SACI_VERIFY_CCFG | CC27XX_SACI_BLANK_CHECK };
		retval = cc27xx_saci_call(target, blank_check, ARRAY_SIZE(blank_check),
			NULL, 0, CC27XX_SACI_TIMEOUT_MS, &result);
		if (retval != ERROR_OK)
			return retval;
		if (result == CC27XX_SACI_SUCCESS) {
			LOG_TARGET_INFO(target, "no CCFG: erasing the application sectors");
			erase[0] = CC27XX_SACI_ERASE_MAIN_APP;
		} else {
			LOG_TARGET_WARNING(target, "invalid CCFG: erasing the HSM firmware too");
			erase[0] = CC27XX_SACI_ERASE_CHIP;
		}
		retval = cc27xx_saci_call(target, erase, 2, NULL, 0,
			CC27XX_SACI_ERASE_TIMEOUT_MS, &result);
		if (retval != ERROR_OK)
			return retval;
	}

	if (result != CC27XX_SACI_SUCCESS) {
		LOG_TARGET_ERROR(target, "chip erase failed: %s (0x%02" PRIx8 ")",
			cc27xx_saci_result_name(result), result);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	for (struct flash_bank *bank = flash_bank_list(); bank; bank = bank->next) {
		if (bank->driver != &cc27xx_flash || bank->target != target)
			continue;
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;
	}

	command_print(CMD, "cc27xx: chip erased, retained sector masks 0x%08" PRIx32
		" 0x%08" PRIx32 " 0x%08" PRIx32 "; in SACI until the next reset",
		retained[0], retained[1], retained[2]);
	return ERROR_OK;
}

static int cc27xx_rom_call(struct flash_bank *bank, struct working_area *wa,
	uint32_t function, uint32_t r1, uint32_t r2, uint32_t r3)
{
	struct target *target = bank->target;
	struct reg_param reg_params[7];
	struct armv7m_algorithm armv7m_info = {
		.common_magic = ARMV7M_COMMON_MAGIC,
		.core_mode = ARM_MODE_THREAD,
	};
	int num_regs = 6;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);
	init_reg_param(&reg_params[5], "lr", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, CC27XX_FLASH_API_KEY);
	buf_set_u32(reg_params[1].value, 0, 32, r1);
	buf_set_u32(reg_params[2].value, 0, 32, r2);
	buf_set_u32(reg_params[3].value, 0, 32, r3);
	buf_set_u32(reg_params[4].value, 0, 32, wa->address + CC27XX_WA_BUFFER);
	buf_set_u32(reg_params[5].value, 0, 32, wa->address | 1);

	/* The application's secure stack limit would fault the ROM's stack */
	if (register_get_by_name(target->reg_cache, "msplim_s", true)) {
		init_reg_param(&reg_params[num_regs], "msplim_s", 32, PARAM_OUT);
		buf_set_u32(reg_params[num_regs].value, 0, 32, 0);
		num_regs++;
	}

	int retval = target_run_algorithm(target, 0, NULL, num_regs, reg_params,
		function & ~1, wa->address, CC27XX_ROM_CALL_TIMEOUT_MS, &armv7m_info);
	uint32_t status = buf_get_u32(reg_params[0].value, 0, 32);

	for (int i = 0; i < num_regs; i++)
		destroy_reg_param(&reg_params[i]);

	if (retval != ERROR_OK)
		return retval;
	if (status != CC27XX_FAPI_STATUS_SUCCESS) {
		LOG_ERROR("cc27xx: flash function at 0x%08" PRIx32 " failed, status 0x%" PRIx32,
			function, status);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

/* The boot ROM's flash functions, read once the device is booted */
static int cc27xx_rom_probe(struct flash_bank *bank)
{
	struct cc27xx_bank *cc27xx_bank = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t descex;

	if (cc27xx_bank->sector_erase)
		return ERROR_OK;

	int retval = target_read_u32(target, CC27XX_VIMS_DESCEX, &descex);
	if (retval == ERROR_OK)
		retval = target_read_u32(target,
			CC27XX_HAPI_TABLE + 4 * CC27XX_HAPI_SECTOR_ERASE, &cc27xx_bank->sector_erase);
	if (retval == ERROR_OK)
		retval = target_read_u32(target,
			CC27XX_HAPI_TABLE + 4 * CC27XX_HAPI_PROGRAM, &cc27xx_bank->program);
	if (retval != ERROR_OK) {
		cc27xx_bank->sector_erase = 0;
		return retval;
	}

	uint32_t rom_end = CC27XX_ROM_BASE + (CC27XX_DESCEX_ROMSZ(descex) + 1) * 1024;
	if ((cc27xx_bank->sector_erase & ~1) < CC27XX_ROM_BASE ||
		(cc27xx_bank->sector_erase & ~1) >= rom_end ||
		(cc27xx_bank->program & ~1) < CC27XX_ROM_BASE ||
		(cc27xx_bank->program & ~1) >= rom_end) {
		LOG_ERROR("cc27xx: no flash functions in the boot ROM (0x%08" PRIx32 ", 0x%08" PRIx32 ")",
			cc27xx_bank->sector_erase, cc27xx_bank->program);
		cc27xx_bank->sector_erase = 0;
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

struct cc27xx_session {
	struct working_area *wa;
	struct cortex_m_saved_security security;
	uint32_t cchctl;
};

/* Working area, secure state, and cache and line buffer off (TI's FlashProgram) */
static int cc27xx_begin(struct flash_bank *bank, struct cc27xx_session *s)
{
	struct target *target = bank->target;
	static const uint8_t bkpt[4] = { 0x00, 0xbe, 0x00, 0xbe };
	uint32_t cchsta;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = cc27xx_rom_probe(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_alloc_working_area(target, CC27XX_WA_SIZE, &s->wa);
	if (retval != ERROR_OK) {
		LOG_ERROR("cc27xx: no %d byte working area", CC27XX_WA_SIZE);
		return retval;
	}

	retval = target_write_buffer(target, s->wa->address, sizeof(bkpt), bkpt);
	if (retval == ERROR_OK)
		retval = cortex_m_set_secure(target, &s->security);
	if (retval == ERROR_OK)
		retval = target_read_u32(target, CC27XX_VIMS_CCHCTL, &s->cchctl);
	if (retval == ERROR_OK)
		retval = target_write_u32(target, CC27XX_VIMS_CCHCTL, 0);
	for (int i = 0; retval == ERROR_OK && i < 100; i++) {
		retval = target_read_u32(target, CC27XX_VIMS_CCHSTA, &cchsta);
		if (!(cchsta & CC27XX_VIMS_CCHSTA_BUSY))
			break;
	}

	if (retval != ERROR_OK)
		target_free_working_area(target, s->wa);
	return retval;
}

static int cc27xx_end(struct flash_bank *bank, struct cc27xx_session *s)
{
	struct target *target = bank->target;

	int retval = target_write_u32(target, CC27XX_VIMS_CCHCTL, s->cchctl);
	int retval2 = cortex_m_security_restore(target, &s->security);
	target_free_working_area(target, s->wa);
	return retval != ERROR_OK ? retval : retval2;
}

static bool cc27xx_ccfg_valid(const uint8_t *ccfg)
{
	static const struct {
		const char *name;
		uint32_t start;
		uint32_t len;
	} records[] = {
		{ "boot", CC27XX_CCFG_BOOT_START, CC27XX_CCFG_BOOT_LEN },
		{ "central", CC27XX_CCFG_CENTRAL_START, CC27XX_CCFG_CENTRAL_LEN },
		{ "debug", CC27XX_CCFG_DEBUG_START, CC27XX_CCFG_DEBUG_LEN },
	};
	bool valid = true;

	for (size_t i = 0; i < ARRAY_SIZE(records); i++) {
		const uint8_t *record = ccfg + records[i].start;
		uint32_t crc = ~crc32_le(CRC32_POLY_LE, 0xffffffff, record, records[i].len);
		uint32_t stored = le_to_h_u32(record + records[i].len);
		if (crc != stored) {
			LOG_ERROR("cc27xx: CCFG %s record CRC 0x%08" PRIx32 ", computed 0x%08" PRIx32,
				records[i].name, stored, crc);
			valid = false;
		}
	}
	return valid;
}

static bool cc27xx_is_ccfg(struct flash_bank *bank)
{
	return bank->base == CC27XX_CCFG_BASE;
}

static bool cc27xx_is_scfg(struct flash_bank *bank)
{
	return bank->base == CC27XX_SCFG_BASE;
}

static int cc27xx_protect_check(struct flash_bank *bank);

/* Fail on a protected sector in [offset, offset + count) */
static int cc27xx_check_unprotected(struct flash_bank *bank, uint32_t offset,
	uint32_t count)
{
	/* Protection changes at reset */
	int retval = cc27xx_protect_check(bank);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		const struct flash_sector *sector = &bank->sectors[i];
		if (sector->is_protected != 1 || offset >= sector->offset + sector->size ||
			offset + count <= sector->offset)
			continue;
		if (cc27xx_is_ccfg(bank) || cc27xx_is_scfg(bank))
			LOG_ERROR("cc27xx: the %s is write/erase protected (CCFG.flashProt): "
				"only a chip erase (cc27xx mass_erase) lets it be rewritten",
				cc27xx_is_ccfg(bank) ? "CCFG" : "SCFG");
		else
			LOG_ERROR("cc27xx: sector %u (0x%08" TARGET_PRIxADDR ") is write/erase protected",
				i, bank->base + sector->offset);
		return ERROR_FLASH_PROTECTED;
	}
	return ERROR_OK;
}

/* In SACI, only a chip erase erases: check that the sectors are blank */
static int cc27xx_saci_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	struct target *target = bank->target;
	uint8_t result;
	int retval;

	if (cc27xx_is_scfg(bank)) {
		/* No blank check for the SCFG: programming it fails if not blank */
		return ERROR_OK;
	} else if (cc27xx_is_ccfg(bank)) {
		uint32_t words[5] = { CC27XX_SACI_VERIFY_CCFG | CC27XX_SACI_BLANK_CHECK };
		retval = cc27xx_saci_call(target, words, ARRAY_SIZE(words), NULL, 0,
			CC27XX_SACI_TIMEOUT_MS, &result);
	} else {
		uint32_t words[4] = {
			CC27XX_SACI_VERIFY_MAIN | CC27XX_SACI_BLANK_CHECK,
			bank->base + bank->sectors[first].offset,
			(last - first + 1) * CC27XX_SECTOR_SIZE,
			0
		};
		retval = cc27xx_saci_call(target, words, ARRAY_SIZE(words), NULL, 0,
			CC27XX_SACI_TIMEOUT_MS, &result);
	}
	if (retval != ERROR_OK)
		return retval;

	if (result == CC27XX_SACI_BLANK_CHECK_FAILED) {
		LOG_ERROR("cc27xx: not blank, and the device is in SACI: only a chip erase "
			"(cc27xx mass_erase) erases there");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	if (result != CC27XX_SACI_SUCCESS) {
		LOG_ERROR("cc27xx: blank check failed: %s (0x%02" PRIx8 ")",
			cc27xx_saci_result_name(result), result);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int cc27xx_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	struct cc27xx_bank *cc27xx_bank = bank->driver_priv;
	struct cc27xx_session s;

	if (cc27xx_in_saci(bank->target))
		return cc27xx_saci_erase(bank, first, last);

	int retval = cc27xx_check_unprotected(bank, bank->sectors[first].offset,
		bank->sectors[last].offset + bank->sectors[last].size - bank->sectors[first].offset);
	if (retval != ERROR_OK)
		return retval;

	/* Erased by the write, once the new CCFG is known valid: an erased
	 * CCFG stops the boot before the application */
	if (cc27xx_is_ccfg(bank) || cc27xx_is_scfg(bank)) {
		LOG_WARNING("cc27xx: the %s isn't erased on its own; writing it erases it",
			cc27xx_is_ccfg(bank) ? "CCFG" : "SCFG");
		return ERROR_OK;
	}

	retval = cc27xx_begin(bank, &s);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = first; i <= last && retval == ERROR_OK; i++) {
		retval = cc27xx_rom_call(bank, s.wa, cc27xx_bank->sector_erase,
			bank->base + bank->sectors[i].offset, 0, 0);
		keep_alive();
	}

	int retval2 = cc27xx_end(bank, &s);
	return retval != ERROR_OK ? retval : retval2;
}

/* Little-endian words from bytes, the last one padded */
static unsigned int cc27xx_pack_words(uint32_t *words, const uint8_t *buffer,
	uint32_t count, uint8_t pad)
{
	unsigned int n = DIV_ROUND_UP(count, 4);
	for (unsigned int i = 0; i < n; i++) {
		uint8_t bytes[4] = { pad, pad, pad, pad };
		memcpy(bytes, buffer + 4 * i, MIN(4, count - 4 * i));
		words[i] = le_to_h_u32(bytes);
	}
	return n;
}

/* Write through SACI: per sector (main), or the whole CCFG or SCFG */
static int cc27xx_saci_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t *words = malloc(CC27XX_SACI_MAX_WORDS * sizeof(*words));
	int retval = ERROR_OK;

	if (!words)
		return ERROR_FAIL;

	if (cc27xx_is_ccfg(bank)) {
		words[0] = CC27XX_SACI_PROG_CCFG;
		words[1] = CC27XX_FLASH_API_KEY;
		unsigned int n = cc27xx_pack_words(words + 2, buffer, count, 0xff);
		retval = cc27xx_saci_run(target, words, 2 + n, CC27XX_SACI_TIMEOUT_MS);
		if (retval == ERROR_OK)
			LOG_INFO("cc27xx: CCFG programmed, used from the next reset on");
	} else if (cc27xx_is_scfg(bank)) {
		/* Whole sector: the part not given stays blank */
		uint8_t *scfg = malloc(CC27XX_SCFG_SIZE);
		if (!scfg) {
			free(words);
			return ERROR_FAIL;
		}
		memset(scfg, 0xff, CC27XX_SCFG_SIZE);
		memcpy(scfg + offset, buffer, count);
		words[0] = CC27XX_SACI_PROG_SCFG | (CC27XX_SCFG_SIZE << 16);
		words[1] = CC27XX_FLASH_API_KEY;
		unsigned int n = cc27xx_pack_words(words + 2, scfg, CC27XX_SCFG_SIZE, 0xff);
		free(scfg);
		retval = cc27xx_saci_run(target, words, 2 + n, CC27XX_SACI_TIMEOUT_MS);
	} else {
		uint8_t *chunk = malloc(CC27XX_SECTOR_SIZE);
		if (!chunk) {
			free(words);
			return ERROR_FAIL;
		}

		/* At most a sector per command, from a word boundary (0xff before) */
		while (count > 0 && retval == ERROR_OK) {
			uint32_t lead = offset % 4;
			uint32_t size = CC27XX_SECTOR_SIZE - (offset % CC27XX_SECTOR_SIZE);
			if (size > count + lead)
				size = count + lead;

			memset(chunk, 0xff, lead);
			memcpy(chunk + lead, buffer, size - lead);
			words[0] = CC27XX_SACI_PROG_MAIN_SECTOR | (size << 16);
			words[1] = CC27XX_FLASH_API_KEY;
			words[2] = bank->base + offset - lead;
			unsigned int n = cc27xx_pack_words(words + 3, chunk, size, 0);
			retval = cc27xx_saci_run(target, words, 3 + n, CC27XX_SACI_TIMEOUT_MS);

			buffer += size - lead;
			offset += size - lead;
			count -= size - lead;
			keep_alive();
		}
		free(chunk);
	}

	free(words);
	return retval;
}

static int cc27xx_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct cc27xx_bank *cc27xx_bank = bank->driver_priv;
	struct target *target = bank->target;
	struct cc27xx_session s;

	if (cc27xx_is_ccfg(bank)) {
		if (offset != 0 || count != CC27XX_CCFG_SIZE) {
			LOG_ERROR("cc27xx: the CCFG is written whole (0x%x bytes)", CC27XX_CCFG_SIZE);
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}
		if (!cc27xx_ccfg_valid(buffer))
			return ERROR_FAIL;
	}

	if (cc27xx_in_saci(target))
		return cc27xx_saci_write(bank, buffer, offset, count);

	if (cc27xx_is_scfg(bank) && (offset != 0 || count != CC27XX_SCFG_SIZE)) {
		LOG_ERROR("cc27xx: the SCFG is written whole (0x%x bytes)", CC27XX_SCFG_SIZE);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	int retval = cc27xx_check_unprotected(bank, offset, count);
	if (retval != ERROR_OK)
		return retval;

	retval = cc27xx_begin(bank, &s);
	if (retval != ERROR_OK)
		return retval;

	if (cc27xx_is_ccfg(bank) || cc27xx_is_scfg(bank))
		retval = cc27xx_rom_call(bank, s.wa, cc27xx_bank->sector_erase,
			bank->base, 0, 0);

	/* At most a sector per call, never across a sector (and bank) boundary */
	while (count > 0 && retval == ERROR_OK) {
		uint32_t size = CC27XX_SECTOR_SIZE - (offset % CC27XX_SECTOR_SIZE);
		if (size > count)
			size = count;

		retval = target_write_buffer(target, s.wa->address + CC27XX_WA_BUFFER,
			size, buffer);
		if (retval == ERROR_OK)
			retval = cc27xx_rom_call(bank, s.wa, cc27xx_bank->program,
				s.wa->address + CC27XX_WA_BUFFER, bank->base + offset, size);

		buffer += size;
		offset += size;
		count -= size;
		keep_alive();
	}

	int retval2 = cc27xx_end(bank, &s);
	return retval != ERROR_OK ? retval : retval2;
}

static int cc27xx_protect_check(struct flash_bank *bank)
{
	uint32_t wepra, weprb0, weprb1, wepraux;
	int retval;

	/* VIMS not readable in SACI: protection unknown, the commands check it */
	if (cc27xx_in_saci(bank->target)) {
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_protected = -1;
		return ERROR_OK;
	}

	/* The boot ROM applies CCFG.flashProt before the application starts */
	if (cc27xx_is_ccfg(bank) || cc27xx_is_scfg(bank)) {
		retval = target_read_u32(bank->target, CC27XX_VIMS_WEPRAUX, &wepraux);
		if (retval != ERROR_OK)
			return retval;
		bank->sectors[0].is_protected = !(wepraux & (cc27xx_is_ccfg(bank) ?
			CC27XX_VIMS_WEPRAUX_CCFG : CC27XX_VIMS_WEPRAUX_SCFG));
		return ERROR_OK;
	}

	retval = target_read_u32(bank->target, CC27XX_VIMS_WEPRA, &wepra);
	if (retval == ERROR_OK)
		retval = target_read_u32(bank->target, CC27XX_VIMS_WEPRB0, &weprb0);
	if (retval == ERROR_OK)
		retval = target_read_u32(bank->target, CC27XX_VIMS_WEPRB1, &weprb1);
	if (retval != ERROR_OK)
		return retval;

	/* 0 = protected, sticky until reset (driverlib FlashGetProtection) */
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		uint32_t bits;
		if (i >= 512) {
			bank->sectors[i].is_protected = -1;
			continue;
		}
		if (i < 32)
			bits = wepra >> i;
		else if (i < 256)
			bits = weprb0 >> (i / 8);
		else
			bits = weprb1 >> ((i / 8) % 32);
		bank->sectors[i].is_protected = !(bits & 1);
	}
	return ERROR_OK;
}

static int cc27xx_probe(struct flash_bank *bank)
{
	struct cc27xx_bank *cc27xx_bank = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t descex, sector_size = CC27XX_SECTOR_SIZE;

	if (!is_armv7m(target_to_armv7m(target))) {
		LOG_ERROR("cc27xx: not a Cortex-M target");
		return ERROR_TARGET_INVALID;
	}

	/* For the info only */
	if (cc27xx_ap_read(target, CC27XX_CFG_AP, CC27XX_CFG_AP_DEVICEID,
			&cc27xx_bank->device_id) != ERROR_OK ||
		cc27xx_ap_read(target, CC27XX_CFG_AP, CC27XX_CFG_AP_PARTID,
			&cc27xx_bank->part_id) != ERROR_OK) {
		cc27xx_bank->device_id = 0;
		cc27xx_bank->part_id = 0;
	}

	uint32_t size;
	if (cc27xx_is_ccfg(bank)) {
		size = CC27XX_CCFG_SIZE;
	} else if (cc27xx_is_scfg(bank)) {
		size = CC27XX_SCFG_SIZE;
		sector_size = CC27XX_SCFG_SIZE;
	} else if (cc27xx_in_saci(target)) {
		/* VIMS not readable: the size from the bank command, or known */
		if (cc27xx_bank->config_size)
			cc27xx_bank->flash_kb = cc27xx_bank->config_size / 1024;
		if (!cc27xx_bank->flash_kb) {
			LOG_ERROR("cc27xx: flash size unknown in SACI: give it in the flash bank command");
			return ERROR_FAIL;
		}
		size = cc27xx_bank->flash_kb * 1024;
	} else {
		int retval = target_read_u32(target, CC27XX_VIMS_DESCEX, &descex);
		if (retval != ERROR_OK)
			return retval;
		cc27xx_bank->flash_kb = CC27XX_DESCEX_FLSZ(descex) + 1;
		size = cc27xx_bank->flash_kb * 1024;
	}

	free(bank->sectors);
	bank->sectors = NULL;
	bank->size = size;
	bank->num_sectors = size / sector_size;
	bank->sectors = alloc_block_array(0, sector_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	cc27xx_bank->probed = true;
	return cc27xx_protect_check(bank);
}

static int cc27xx_auto_probe(struct flash_bank *bank)
{
	struct cc27xx_bank *cc27xx_bank = bank->driver_priv;

	if (cc27xx_bank->probed)
		return ERROR_OK;
	return cc27xx_probe(bank);
}

static int cc27xx_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct cc27xx_bank *cc27xx_bank = bank->driver_priv;

	command_print_sameline(cmd,
		"CC27xx: device ID 0x%08" PRIx32 ", part ID 0x%08" PRIx32 ", %" PRIu32 " KB flash%s",
		cc27xx_bank->device_id, cc27xx_bank->part_id, cc27xx_bank->flash_kb,
		cc27xx_in_saci(bank->target) ? ", in SACI" : "");
	return ERROR_OK;
}

/*
 * flash bank <name> cc27xx <base> <size> 0 0 <target>: base 0 (main, size
 * 0 = read from the device), 0x4e020000 (CCFG) or 0x4e040000 (SCFG)
 */
FLASH_BANK_COMMAND_HANDLER(cc27xx_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (bank->base != CC27XX_MAIN_BASE && bank->base != CC27XX_CCFG_BASE &&
		bank->base != CC27XX_SCFG_BASE) {
		LOG_ERROR("cc27xx: bank base 0x%08" TARGET_PRIxADDR ": 0x0 (main), 0x%08x (CCFG) "
			"or 0x%08x (SCFG)", bank->base, CC27XX_CCFG_BASE, CC27XX_SCFG_BASE);
		return ERROR_FAIL;
	}

	struct cc27xx_bank *cc27xx_bank = calloc(1, sizeof(*cc27xx_bank));
	if (!cc27xx_bank)
		return ERROR_FAIL;

	cc27xx_bank->config_size = bank->size;
	bank->driver_priv = cc27xx_bank;
	bank->write_start_alignment = 0;
	bank->write_end_alignment = 0;
	return ERROR_OK;
}

static const struct command_registration cc27xx_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = cc27xx_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Erase main flash, CCFG and SCFG through SACI, keeping the "
			"HSM firmware. Flash writes go through SACI until the next reset.",
	},
	{
		.name = "reset_assert",
		.handler = cc27xx_handle_reset_assert_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Reset the device through the SEC-AP and keep the boot ROM "
			"in SACI.",
	},
	{
		.name = "saci_exit",
		.handler = cc27xx_handle_saci_exit_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Leave SACI: halt at the application's entry if the reset "
			"halts, else run.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cc27xx_command_handlers[] = {
	{
		.name = "cc27xx",
		.mode = COMMAND_ANY,
		.help = "cc27xx flash command group",
		.usage = "",
		.chain = cc27xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver cc27xx_flash = {
	.name = "cc27xx",
	.commands = cc27xx_command_handlers,
	.flash_bank_command = cc27xx_flash_bank_command,
	.erase = cc27xx_erase,
	.write = cc27xx_write,
	.read = default_flash_read,
	.probe = cc27xx_probe,
	.auto_probe = cc27xx_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = cc27xx_protect_check,
	.info = cc27xx_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
