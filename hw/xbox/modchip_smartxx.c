/*
 * SmartXX modchip emulation
 *
 * The classic SmartXX (not the OPX variant) carries a 4 MB flash presented to
 * the Xbox through the LPC ROM window, with a software-selected bank register
 * and no physical bank switches. Its distinguishing feature is that the flash
 * is deliberately awkward to talk to: both the JEDEC command bytes and the
 * command addresses are bit-permuted, so a stock flasher sees a chip that
 * refuses every command.
 *
 * The CPLD XORs two physical ranges with 0xD7 on the way in and on the way
 * out: the upper 192 KB of slot 7 (0x190000-0x1BFFFF) and of the bootloader
 * (0x1D0000-0x1FFFFF), but only when they are reached through one of the wide
 * banks. Through the 256 KB bank that owns the range the die is read and
 * written raw. Both halves are measured on the LPC bus of a real chip: the
 * boot ROM and loader read the bootloader bank raw through bank 0 (and the
 * loader's signature check only passes on those bytes), while the OS's
 * self-check reads the same bytes through the full-chip bank and gets them
 * XORed. Erase leaves 0xFF on the die, which the wide banks read as 0x28.
 *
 * Image files hold the die, so they are the backing store as they are. That
 * is also what PrometheOS stores: it reads through the full-chip bank and
 * XORs the windows back to die form, and its writeBank XORs on the way out
 * again so the CPLD lands the die bytes.
 *
 * The official firmware leaves the 2BL area empty and boots through the Visor
 * wrap: MCPX panics, continues at RAM 0, and a far jump planted there by the
 * xcodes enters flash at 0xFF001000.
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/option.h"
#include "qemu/datadir.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/system.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/isa/isa.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/notify.h"
#include "exec/translation-block.h"
#include "hw/xbox/modchip_smartxx_keys.h"

/* I/O register block 0xF700-0xF7FF. 0xF700-0xF704 and 0xF70D are the named
 * registers; 0xF705 is written to hand control back to the onboard TSOP.
 * 0xF7B6 is the handshake clock. */
#define SMARTXX_REGISTER_BASE     0xF700
#define SMARTXX_REGISTER_SIZE     0x100
#define SMARTXX_REG_DISP          0x0 /* LCD data and control, also SPI */
#define SMARTXX_REG_ID            0x1 /* read: chip id, write: LCD backlight */
#define SMARTXX_REG_BANKING       0x3
#define SMARTXX_REG_FLASH_ENABLE  0x4
#define SMARTXX_REG_TSOP          0x5
#define SMARTXX_REG_SYS_COMMIT    0x7 /* write 0xFF to end the handshake */
#define SMARTXX_REG_SYS_DATA      0x8 /* read: key stream, write: response 1 */
#define SMARTXX_REG_SYS_RESP2     0x9
#define SMARTXX_REG_SYS_DATA_HI   0xB /* last byte of a dword read of SYS_DATA */
#define SMARTXX_REG_IO            0xD
#define SMARTXX_REG_SYSCON_CLOCK  0xB6 /* write 1 to start, 0 to clock, 0x80 to end */

#define SMARTXX_SYSCON_CLOCK_PORT \
    (SMARTXX_REGISTER_BASE + SMARTXX_REG_SYSCON_CLOCK)

/* LPC ROM window. Array-mode fetches come from a ROM device region's RAM
 * tiles, so executing from flash stays fast and guest writes never touch the
 * tiles (which would look like self-modifying code to TCG). Every write goes
 * to the command state machine instead. Reads during a JEDEC sequence
 * (autoselect ids) go through an overlapping IO region enabled while a
 * command is in progress or flash-enable is set. */
#define SMARTXX_LPC_ROM_START 0xFF000000
#define SMARTXX_LPC_ROM_SIZE  0x01000000

/*
 * The CPLD streams a key: a write of 1 to 0xF7B6 restarts it, every write of 0
 * clocks out the next dword, presented as four bytes at 0xF708-0xF70B, and a
 * write of 0x80 marks the end of the boot handshake without stopping the
 * stream. The same registers are also decoded from LPC memory cycles at
 * 0x8xxxxxxx by their low 16 bits, and the official firmware mixes port and
 * memory accesses to them at random (from the ACPI timer) so that a clone
 * which only decodes port I/O fails.
 *
 * The official firmware uses the stream as key material, not as a nonce: the
 * loader XORs the first 0x746 dwords into an embedded table and jumps into
 * the result, which must decrypt to code, and the OS then keeps clocking the
 * stream for its runtime self-check, hashing one dword per 8 bytes of the
 * bootloader and slot-7 banks (0x2FFC0 / 8 = 49136 dwords) against a digest
 * stored in flash at 0x1EFFC0. A mismatch power-cycles the console. The
 * sequence is not linear (Berlekamp-Massey gives N/2), so it cannot be
 * derived; it is replayed from captures of the real chips
 * (modchip_smartxx_keys.h, one stream per variant). Past the end of a capture
 * the stand-in below takes over, which nothing has been seen to read.
 *
 * The two variants carry different keys and expect different responses, so the
 * pair has to follow whichever chip the id register claims to be - answering an
 * OPX with the classic response fails the check just as surely as saying
 * nothing at all.
 *
 * Nothing here gates flash access. On a real board the handshake is what
 * unlocks a chip whose flash reports locked manufacturer IDs, but this device
 * reports unlocked ones, so the handshake is answered rather than enforced.
 */
#define SMARTXX_SYSCON_LENGTH         0x746

/* LPC memory alias of the register block: the CPLD decodes memory cycles at
 * 0x80000000-0x80FFFFFF by their low 16 bits. */
#define SMARTXX_LPC_MEM_START 0x80000000
#define SMARTXX_LPC_MEM_SIZE  0x01000000

#define SMARTXX_SYSCON_OPX_KEY_FIRST  0xD9D120DB
#define SMARTXX_SYSCON_OPX_KEY_LAST   0xBED5CD65
#define SMARTXX_SYSCON_OPX_RESP1      0xFC
#define SMARTXX_SYSCON_OPX_RESP2      0x40

#define SMARTXX_SYSCON_CLS_KEY_FIRST  0x1F623100
#define SMARTXX_SYSCON_CLS_KEY_LAST   0x28B5E8CC
#define SMARTXX_SYSCON_CLS_RESP1      0xB0
#define SMARTXX_SYSCON_CLS_RESP2      0xB0

/* Chip identity, and what software does with it. The value decides more than the
 * name: it is what tells the guest which bank map and which handshake key it is
 * talking to, so getting it wrong sends the guest looking for its payload in a
 * bank the fitted part cannot address.
 *
 * The two bytes here are the ones that work against real firmware. Note they do
 * not agree on how they are decoded, which is why they are spelled out
 * separately rather than derived:
 *
 *   0xF5  OPX. Both accept nibble 5.
 *
 *   0xF1  Classic. The flash tool takes 1, 2 or 8 here, but the boot-time
 *         firmware switches on the nibble and only has arms for 1 and 5, so 1
 *         is the only value the two agree on. With 8 the firmware falls through
 *         its variant switch with the key length left uninitialised.
 *
 * The whole byte is also checked against 0x11/0x15 (Aladdin) and 0x69
 * (Xchanger) before the nibble is masked, so those three are unusable however
 * their nibbles decode.
 */
#define SMARTXX_ID_OPX            0xF5
#define SMARTXX_ID_CLASSIC        0xF1

/* The chip was sold with either a 2 or a 4 MB part fitted, chosen by the
 * flash-size property rather than guessed from the image, so that an image of
 * the wrong size is a clear error instead of a chip that quietly behaves like
 * the other variant. The bank layout is the same either way: a 2 MB board is
 * the bottom half of the map, and with no A21 the system banks above 2 MB alias
 * down onto the user slots. */
#define SMARTXX_FLASH_2MB         (2 * 1024 * 1024)
#define SMARTXX_FLASH_4MB         (4 * 1024 * 1024)
#define SMARTXX_FLASH_MAX_SIZE    SMARTXX_FLASH_4MB
#define SMARTXX_TSOP_MAX_SIZE     (1024 * 1024)

/* Macronix parts, one per fitted size, as the real chips answered the JEDEC
 * autoselect on the LPC bus (0xB0/0x8A and 0xB0/0x19 on the bus, which is
 * these values through the data-line permutation). The parts are x16 flash
 * in byte mode, so the device id sits at A0 = 1, which is CPU offset 2. */
#define SMARTXX_MANUF_ID          0xC2
#define SMARTXX_DEV_ID_2MB        0x49 /* MX29LV160B */
#define SMARTXX_DEV_ID_4MB        0xA8 /* MX29LV320B */

#define SMARTXX_BANK_BOOTLOADER   0
#define SMARTXX_BANK_FULL_4096K   0x20
#define SMARTXX_BANK_FULL_2048K   0x20

/* Command and unlock addresses as they appear on the bus, i.e. already permuted
 * from the JEDEC 0x555 and 0xAAA. They are spelled out here so the decode does
 * not need the forward permutation at all. */
#define SMARTXX_ADDR_555_MANGLED  0x4195
#define SMARTXX_ADDR_AAA_MANGLED  0x0A6A

#define SMARTXX_FLASH_FLUSH_DELAY_MS 2000
#define MCPX_SIZE (512)

extern MemoryRegion *rom_memory__; //FIXME

/* Set by the machine setup (vl.c) when a modchip provides its own flash
 * mapping, so the stock BIOS/MCPX setup in xbox.c is skipped. */
extern bool modchip_enabled;

/* Shared with modchip_xecuter.c, which defines it. */
void modchip_debug_log(const char *tag, const char *fmt, ...)
    G_GNUC_PRINTF(2, 3);

/* Debug verbosity as a bitmask:
 *   bit 0 (0x1) GENERAL - I/O register accesses
 *   bit 1 (0x2) FLASH   - flash command decode, banking, program/erase and
 *                         persistence
 *   bit 2 (0x4) READ    - individual ROM window reads, over the window
 *                         [SMARTXX_READ_LOG_FIRST, +SMARTXX_READ_LOG_COUNT)
 *
 * The read trace is expensive: the boot path fetches through this callback
 * millions of times, so it is a window rather than a cap. The FETCH count in
 * the flash stream is what tells you where to put that window - it reports the
 * read number at each bank change, so a bank switch that reports "after 68341
 * reads" is traced by setting FIRST a little below that.
 *
 * From a cold start the first ~450 reads are the MCPX boot ROM walking the
 * xcode table at offset 0x80 and running the NV2A memory init from it, so a
 * window starting at 0 mostly shows that. Bank changes are reported by the
 * flash stream regardless of this setting. */
#define SMARTXX_DEBUG_LEVEL 0x3

#define SMARTXX_READ_LOG_FIRST 0
#define SMARTXX_READ_LOG_COUNT 4096
/* After that burst, keep tracing the visor/bootloader window (LPC < 64 KB). */
#define SMARTXX_READ_LOG_POST_OFF  0x10000
#define SMARTXX_READ_LOG_POST_COUNT 2048

#if (SMARTXX_DEBUG_LEVEL & 0x1)
# define DPRINTF(fmt, ...) modchip_debug_log("smartxx-io", fmt, ## __VA_ARGS__)
#else
# define DPRINTF(fmt, ...) do { } while (0)
#endif

#if (SMARTXX_DEBUG_LEVEL & 0x2)
# define SXLOG(fmt, ...) modchip_debug_log("smartxx-flash", fmt, ## __VA_ARGS__)
#else
# define SXLOG(fmt, ...) do { } while (0)
#endif

#if (SMARTXX_DEBUG_LEVEL & 0x4)
# define SXRLOG(fmt, ...) modchip_debug_log("smartxx-read", fmt, ## __VA_ARGS__)
#else
# define SXRLOG(fmt, ...) do { } while (0)
#endif

static uint8_t smartxx_flash_raw[SMARTXX_FLASH_MAX_SIZE];
static uint8_t smartxx_tsop_raw[SMARTXX_TSOP_MAX_SIZE];
static uint8_t smartxx_mcpx_raw[MCPX_SIZE];

typedef enum {
    SMARTXX_FLASH_IDLE,
    SMARTXX_FLASH_UNLOCK_1,
    SMARTXX_FLASH_UNLOCK_2,
    SMARTXX_FLASH_ERASE_1,
    SMARTXX_FLASH_ERASE_2,
    SMARTXX_FLASH_ERASE_3,
    SMARTXX_FLASH_CHIPID,
    SMARTXX_FLASH_WRITE,
} SmartxxFlashState;

G_GNUC_UNUSED static const char *smartxx_state_name(SmartxxFlashState st)
{
    switch (st) {
    case SMARTXX_FLASH_IDLE:     return "IDLE";
    case SMARTXX_FLASH_UNLOCK_1: return "UNLOCK_1";
    case SMARTXX_FLASH_UNLOCK_2: return "UNLOCK_2";
    case SMARTXX_FLASH_ERASE_1:  return "ERASE_1";
    case SMARTXX_FLASH_ERASE_2:  return "ERASE_2";
    case SMARTXX_FLASH_ERASE_3:  return "ERASE_3";
    case SMARTXX_FLASH_CHIPID:   return "CHIPID";
    case SMARTXX_FLASH_WRITE:    return "WRITE";
    }
    return "?";
}

typedef struct SmartxxState {
    ISADevice dev;
    MemoryRegion io;
    MemoryRegion lpc_mem;
    MemoryRegion flash_mem;
    MemoryRegion flash_mmio;

    char *rom_file; /* flash image, sized to match flash_size */
    uint32_t flash_size;

    /* Captured key stream of the fitted variant, one dword per clock. */
    const uint32_t *syscon_stream;
    uint32_t syscon_stream_len;
    bool syscon_stream_overrun;

    uint8_t bank;
    bool flash_enable;
    bool tsop;

    /* Last values written to the registers we do not act on, so reads see
     * something plausible. */
    uint8_t disp;
    uint8_t backlight;
    uint8_t io_reg;

    SmartxxFlashState flash_state;

    uint32_t tsop_size;

    QEMUTimer *flush_timer;
    Notifier exit_notifier;
    bool dirty;

    /* Rate limiting for the read trace. */
    uint64_t read_count;
    uint64_t post_read_count;
    uint32_t read_log_key;
    bool logged_payload;
    bool logged_syscon_read;
    bool syscon_active;
    uint32_t syscon_clocks;
    uint32_t syscon_latch;
    uint8_t syscon_resp1;
    uint8_t syscon_resp2;

    /* Set once flash_mem is in the address map. Rebuilds before that must
     * not invalidate TBs. */
    bool rom_mapped;
    bool flash_mmio_on;
} SmartxxState;

#define SMARTXX_DEVICE(obj) \
    OBJECT_CHECK(SmartxxState, (obj), "modchip-smartxx")

/*
 * Undo the chip's address permutation. The forward direction is a pure bit
 * permutation: a three-cycle moving bit 2 to 7, 7 to 6 and 6 back to 2, plus a
 * swap of bits 10 and 14. Nothing at bit 16 or above moves, so this only ever
 * shuffles an address within a 64 KB window.
 */
static uint32_t smartxx_unmangle_addr(uint32_t addr)
{
    uint32_t out = addr & ~(uint32_t)((1 << 2) | (1 << 6) | (1 << 7) |
                                      (1 << 10) | (1 << 14));

    if (addr & (1 << 7))  out |= 1 << 2;
    if (addr & (1 << 2))  out |= 1 << 6;
    if (addr & (1 << 6))  out |= 1 << 7;
    if (addr & (1 << 14)) out |= 1 << 10;
    if (addr & (1 << 10)) out |= 1 << 14;

    return out;
}

/*
 * Undo the chip's command permutation: bits 1 and 5 swap, as do bits 4 and 6.
 * That is its own inverse, so the same routine serves both directions. It fixes
 * 0xAA, 0x55 and 0x80, which is why the JEDEC unlock cycles pass through
 * unchanged and only the command byte that follows them looks wrong.
 */
static uint8_t smartxx_unmangle_cmd(uint8_t value)
{
    uint8_t out = value & ~(uint8_t)((1 << 1) | (1 << 4) | (1 << 5) | (1 << 6));

    if (value & (1 << 5)) out |= 1 << 1;
    if (value & (1 << 1)) out |= 1 << 5;
    if (value & (1 << 6)) out |= 1 << 4;
    if (value & (1 << 4)) out |= 1 << 6;

    return out;
}

/* Which of the two parts is fitted. The 2 MB board is the OPX and the 4 MB one
 * is the classic chip, and the two differ in more than capacity: the id byte,
 * the handshake key and the bank map above the user slots all follow from it. */
static bool smartxx_is_opx(SmartxxState *s)
{
    return s->flash_size == SMARTXX_FLASH_2MB;
}

/*
 * Bank register decode.
 *
 * The seven user slots count *down* in address as the bank number goes up,
 * which is why slot 1 is bank 7 at the bottom of the chip and the bootloader
 * is bank 0 at the top of the first 2 MB.
 *
 * OPX (2 MB) and classic (4 MB) both use a formula. The map repeats every
 * 0x40: only the low 6 bits of the bank id matter. 0-7 are 256 KB counting
 * down, 8-10 are 512 KB counting up, and unknown ids alias to bank 0.
 *
 * OPX then has bank 11 as the lower 1 MB and 0x20 as the full 2 MB.
 *
 * Classic keeps those, with 0x20 as the full 4 MB, and adds windows in the
 * upper 2 MB: 1 MB banks at 0x200000/0x300000, 512 KB banks 15-18, 2 MB
 * halves, a 1 MB window at 0x100000, and 0x19 as another full-chip alias.
 */
/* Shared 0-10 decode. Unknown ids keep w=18, v=7 (alias of bank 0). */
static void smartxx_bank_window_low(uint8_t id, uint32_t *w, uint32_t *v)
{
    *w = 18;
    *v = (id < 8) ? (id ^ 7) : 7;

    if (id >= 8 && id <= 10) {
        *w = 19;
        *v = id - 8;
    }
}

/* OPX bank map from hardware dumps. The map repeats every 0x40 (id & 0x3F).
 * LPC offset i maps to physical start + (i & (size - 1)). */
static void smartxx_opx_bank_window(uint8_t id, uint32_t *offset, uint32_t *size)
{
    id &= 0x3F; /* period 0x40 */

    uint32_t w, v;

    smartxx_bank_window_low(id, &w, &v);

    if (id == 11) {
        w = 20;
        v = 0;
    } else if (id == 0x20) {
        w = 21;
        v = 0;
    }

    *offset = v << w;
    *size = 1u << w;
}

/* Classic 4 MB map. Period 0x40. Bank 11 is the lower 1 MB; 0x19 and 0x20
 * are the full 4 MB. */
static void smartxx_classic_bank_window(uint8_t id, uint32_t *offset,
                                        uint32_t *size)
{
    id &= 0x3F; /* period 0x40 */

    uint32_t w, v;

    smartxx_bank_window_low(id, &w, &v);

    if (id == 11 || id == 0x13) {
        w = 20;
        v = 0;
    } else if (id == 0x14) {
        w = 20;
        v = 1;
    } else if (id == 12 || id == 0x15) {
        w = 20;
        v = 2;
    } else if (id == 13 || id == 0x16) {
        w = 20;
        v = 3;
    } else if (id >= 15 && id <= 18) {
        w = 19;
        v = 4 + (id - 15);
    } else if (id == 0x17) {
        w = 21;
        v = 0;
    } else if (id == 14 || id == 0x18) {
        w = 21;
        v = 1;
    } else if (id == 0x19 || id == 0x20) {
        w = 22;
        v = 0;
    }

    *offset = v << w;
    *size = 1u << w;
}

static bool smartxx_bank_decode(SmartxxState *s, uint8_t bank,
                                uint32_t *offset, uint32_t *size)
{
    if (smartxx_is_opx(s)) {
        smartxx_opx_bank_window(bank, offset, size);
    } else {
        smartxx_classic_bank_window(bank, offset, size);
    }

    return true;
}

/* Resolve a ROM-space offset to the backing image and the address within it.
 * Returns NULL when nothing is mapped. */
static uint8_t *smartxx_resolve(SmartxxState *s, hwaddr offset, uint32_t *pos)
{
    if (s->tsop) {
        /* The chip has released the bus and the onboard TSOP flash responds,
         * mirrored across the ROM window. */
        if (s->tsop_size == 0) {
            return NULL;
        }
        *pos = offset % s->tsop_size;
        return smartxx_tsop_raw;
    }

    uint32_t bk_off, bk_size;
    if (!smartxx_bank_decode(s, s->bank, &bk_off, &bk_size)) {
        return NULL;
    }

    /* Wrapping on a short part is what the hardware does: the address lines the
     * fitted chip does not have simply are not there, so a bank above the top
     * of the image aliases down. */
    *pos = (bk_off + (offset & (bk_size - 1))) % s->flash_size;
    return smartxx_flash_raw;
}

/* Whether a physical flash offset falls in one of the two obfuscated ranges:
 * the upper 192 KB of slot 7 (bank 1, 0x180000) and of the bootloader (bank
 * 0, 0x1C0000). The addresses are fixed by the bank map, so they are the same
 * on a 2 MB part as on a 4 MB one. */
#define SMARTXX_XOR_BYTE 0xD7

static bool smartxx_is_xor_offset(uint32_t pos)
{
    return (pos >= 0x190000 && pos < 0x1C0000) ||
           (pos >= 0x1D0000 && pos < 0x200000);
}

/* The XOR applies through the wide banks only; the 256 KB banks that own the
 * two ranges see the die raw. Same in both directions. */
static uint8_t smartxx_cpld_xor(SmartxxState *s, uint32_t pos, uint8_t byte)
{
    uint32_t bk_off, bk_size;

    if (s->tsop || !smartxx_is_xor_offset(pos)) {
        return byte;
    }
    smartxx_bank_decode(s, s->bank, &bk_off, &bk_size);
    return bk_size > 0x40000 ? byte ^ SMARTXX_XOR_BYTE : byte;
}

static uint32_t smartxx_rom_window_size(SmartxxState *s)
{
    uint32_t win;

    if (s->tsop) {
        win = s->tsop_size ? s->tsop_size : SMARTXX_LPC_ROM_SIZE;
    } else {
        uint32_t bk_off, bk_size;

        smartxx_bank_decode(s, s->bank, &bk_off, &bk_size);
        win = bk_size;
    }
    if (win == 0 || win > SMARTXX_LPC_ROM_SIZE) {
        win = SMARTXX_LPC_ROM_SIZE;
    }
    return win;
}

/* Invalidate TBs for a range of the ROM window after poking the RAM backing.
 * Use ram_addr, not the guest physical window: TCG indexes code by RAMBlock. */
static void smartxx_flush_rom_range(SmartxxState *s, hwaddr offset,
                                    uint32_t size)
{
    ram_addr_t ram;

    if (!s->rom_mapped || size == 0) {
        return;
    }
    if (offset >= SMARTXX_LPC_ROM_SIZE) {
        return;
    }
    if (size > SMARTXX_LPC_ROM_SIZE - offset) {
        size = SMARTXX_LPC_ROM_SIZE - offset;
    }
    ram = memory_region_get_ram_addr(&s->flash_mem);
    if (ram == RAM_ADDR_INVALID) {
        return;
    }
    tb_invalidate_phys_range(NULL, ram + offset, ram + offset + size - 1);
    memory_region_set_dirty(&s->flash_mem, offset, size);
}

static void smartxx_rebuild_rom_view(SmartxxState *s)
{
    uint8_t *rom = memory_region_get_ram_ptr(&s->flash_mem);
    uint32_t win = smartxx_rom_window_size(s);
    uint32_t i, off;

    for (i = 0; i < win; i++) {
        uint32_t pos;
        uint8_t *chip = smartxx_resolve(s, i, &pos);
        uint8_t byte = chip ? chip[pos] : 0xFF;

        if (chip == smartxx_flash_raw) {
            byte = smartxx_cpld_xor(s, pos, byte);
        }
        rom[i] = byte;
    }
    for (off = win; off < SMARTXX_LPC_ROM_SIZE; off += win) {
        uint32_t n = MIN(win, SMARTXX_LPC_ROM_SIZE - off);

        memcpy(rom + off, rom, n);
    }

    smartxx_flush_rom_range(s, 0, SMARTXX_LPC_ROM_SIZE);
}

/* Cache-style stores into the LPC window. Patch the ROM RAM tiles in place
 * and flush only those addresses so a write at 0x83E7 cannot invalidate the
 * visor stub still running at 0xFF001000. */
static void smartxx_patch_rom_view(SmartxxState *s, hwaddr offset,
                                   uint64_t value, unsigned int size)
{
    uint8_t *rom = memory_region_get_ram_ptr(&s->flash_mem);
    uint32_t win = smartxx_rom_window_size(s);
    unsigned int i;

    for (i = 0; i < size; i++) {
        uint32_t at = ((uint32_t)offset + i) & (win - 1);
        uint8_t byte = (value >> (i * 8)) & 0xFF;
        uint32_t off;

        for (off = at; off < SMARTXX_LPC_ROM_SIZE; off += win) {
            rom[off] = byte;
            smartxx_flush_rom_range(s, off, 1);
        }
    }
}

static void smartxx_set_flash_mmio(SmartxxState *s, bool enable)
{
    if (s->flash_mmio_on == enable) {
        return;
    }
    s->flash_mmio_on = enable;
    memory_region_set_enabled(&s->flash_mmio, enable);
}

static void smartxx_flash_enter_idle(SmartxxState *s)
{
    s->flash_state = SMARTXX_FLASH_IDLE;
    smartxx_set_flash_mmio(s, false);
}

static uint8_t smartxx_chip_id(SmartxxState *s)
{
    return smartxx_is_opx(s) ? SMARTXX_ID_OPX : SMARTXX_ID_CLASSIC;
}

static uint8_t smartxx_dev_id(SmartxxState *s)
{
    return s->flash_size == SMARTXX_FLASH_2MB ? SMARTXX_DEV_ID_2MB
                                              : SMARTXX_DEV_ID_4MB;
}

/* Sector geometry of the fitted flash part.
 *
 * Classic (4 MB) matches PrometheOS: 8 KB boot blocks across the bottom 64 KB,
 * then uniform 64 KB sectors.
 *
 * OPX (2 MB) is a bottom-boot Am29F160-style map, not the same 8 KB split:
 *   0x000000-0x003FFF  16 KB
 *   0x004000-0x007FFF   8 KB (two sectors)
 *   0x008000-0x00FFFF  32 KB
 *   0x010000-0x1FFFFF  64 KB
 */
static uint32_t smartxx_sector_size(SmartxxState *s, uint32_t pos)
{
    if (smartxx_is_opx(s)) {
        if (pos <= 0x003FFF) {
            return 16 * 1024;
        }
        if (pos <= 0x007FFF) {
            return 8 * 1024;
        }
        if (pos <= 0x00FFFF) {
            return 32 * 1024;
        }
        return 64 * 1024;
    }

    return pos <= 0x00FFFF ? 8 * 1024 : 64 * 1024;
}

static void smartxx_flash_flush(SmartxxState *s)
{
    if (!s->dirty || s->rom_file == NULL || *s->rom_file == '\0') {
        return;
    }

    int fd = qemu_open(s->rom_file, O_WRONLY | O_BINARY, NULL);
    if (fd < 0) {
        fprintf(stderr, "smartxx: could not open '%s' to persist flash\n",
                s->rom_file);
        return;
    }

    ssize_t written = write(fd, smartxx_flash_raw, s->flash_size);
    close(fd);

    if (written != (ssize_t)s->flash_size) {
        fprintf(stderr,
                "smartxx: short write persisting flash to '%s' (%zd/%u)\n",
                s->rom_file, written, s->flash_size);
        return;
    }

    s->dirty = false;
    SXLOG("PERSIST wrote %u bytes back to '%s'", s->flash_size, s->rom_file);
}

static void smartxx_flash_flush_cb(void *opaque)
{
    smartxx_flash_flush((SmartxxState *)opaque);
}

static void smartxx_flash_exit_notify(Notifier *n, void *opaque)
{
    SmartxxState *s = container_of(n, SmartxxState, exit_notifier);
    smartxx_flash_flush(s);
}

static void smartxx_flash_mark_dirty(SmartxxState *s)
{
    s->dirty = true;

    if (s->flush_timer) {
        timer_mod(s->flush_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                                      SMARTXX_FLASH_FLUSH_DELAY_MS);
    }
}

/*
 * Writes that are not a JEDEC cycle still land on the LPC ROM window. On the
 * real CPU that window is write-back cached, which is why a kernel can store a
 * value "in ROM" and read it back on the next instruction (see xbox.c). The
 * flash chip never programs. Keep the store in the backing image for this
 * session only so the read-back works, and do not persist it.
 */
static void smartxx_rom_cache_write(SmartxxState *s, hwaddr offset,
                                    uint64_t value, unsigned int size)
{
    uint32_t pos;
    uint8_t *chip = smartxx_resolve(s, offset, &pos);

    if (chip == NULL || chip == smartxx_tsop_raw) {
        return;
    }

    for (unsigned int i = 0; i < size; i++) {
        uint32_t at = (pos + i) % s->flash_size;
        uint8_t byte = (value >> (i * 8)) & 0xFF;

        chip[at] = smartxx_cpld_xor(s, at, byte);
    }
    smartxx_patch_rom_view(s, offset, value, size);
}

static void smartxx_erase_sector(SmartxxState *s, hwaddr offset)
{
    uint32_t pos;
    uint8_t *chip;

    /* The erase target arrives permuted, unlike the byte-program address.
     * Above 64 KB it makes no difference, since the permutation cannot move an
     * address out of its own 64 KB sector, but it does decide which boot
     * sector is meant (8 KB on classic, 16/8/32 KB on OPX). */
    chip = smartxx_resolve(s, smartxx_unmangle_addr(offset), &pos);
    if (chip == NULL || chip == smartxx_tsop_raw) {
        return;
    }

    uint32_t sector_size = smartxx_sector_size(s, pos);
    uint32_t base = pos & ~(sector_size - 1);

    if (base + sector_size > s->flash_size) {
        sector_size = s->flash_size - base;
    }

    SXLOG("ERASE  sector base=0x%06x size=%uK (target 0x%06x)", base,
          sector_size / 1024, pos);
    memset(&chip[base], 0xFF, sector_size);
    smartxx_flash_mark_dirty(s);
    smartxx_rebuild_rom_view(s);
}

static void smartxx_erase_chip(SmartxxState *s)
{
    if (s->tsop) {
        return;
    }

    SXLOG("ERASE  whole chip");
    memset(smartxx_flash_raw, 0xFF, s->flash_size);
    smartxx_flash_mark_dirty(s);
    smartxx_rebuild_rom_view(s);
}

static uint64_t smartxx_flash_read(void *opaque, hwaddr offset, unsigned size)
{
    SmartxxState *s = opaque;

    if (s->flash_state == SMARTXX_FLASH_CHIPID) {
        uint8_t id;

        /* Offset bit 1 is the die's A0 (byte mode): 0 = manufacturer,
         * 1 = device. A1 set = sector protect verify: nothing protected.
         * The id bytes cross the CPLD's data-line permutation like the
         * command bytes do, in the other direction. */
        switch ((offset >> 1) & 0x03) {
        case 0:  id = smartxx_unmangle_cmd(SMARTXX_MANUF_ID); break;
        case 1:  id = smartxx_unmangle_cmd(smartxx_dev_id(s)); break;
        default: id = 0; break;
        }

        SXLOG("CHIPID off=0x%06x -> 0x%02x", (uint32_t)offset, id);
        return id;
    }

    uint32_t pos;
    uint8_t *chip = smartxx_resolve(s, offset, &pos);
    if (chip == NULL) {
        return 0xFFFFFFFF >> ((4 - size) * 8);
    }

    bool is_flash = (chip != smartxx_tsop_raw);
    uint32_t limit = is_flash ? s->flash_size : s->tsop_size;

    uint64_t val = 0;
    for (unsigned int i = 0; i < size; i++) {
        uint32_t at = (pos + i) % limit;
        uint8_t byte = chip[at];

        if (is_flash) {
            byte = smartxx_cpld_xor(s, at, byte);
        }

        val |= (uint64_t)byte << (i * 8);
    }

    s->read_count++;

    /* The first fetch from a newly selected bank is worth a line of its own: it
     * is the proof that the guest not only set the bank register but went on to
     * execute from it, which the register write alone does not tell you. It
     * belongs to the flash stream rather than the read trace so that it stays
     * visible once the per-read trace is off. */
    uint32_t key = s->tsop ? 0x100 : s->bank;
    if (key != s->read_log_key) {
        s->read_log_key = key;
        SXLOG("FETCH  first read from %s bank=%u -> 0x%06x (after %" PRIu64
              " reads)", s->tsop ? "tsop" : "flash", s->bank, pos,
              s->read_count);
    }

    /* The visor stub's first copy only reaches LPC 0x83E9. The kernel blob
     * starts at 0x1F94 and runs to 0x2C04C, so the first read at 64 KB is
     * the second-stage memcpy after the PIC handshake. */
    if (!s->logged_payload && is_flash &&
        offset >= 0x10000 && offset < 0x40000) {
        s->logged_payload = true;
        error_report("smartxx: kernel fetch off=0x%06x -> flash 0x%06x "
                     "(after %" PRIu64 " reads)",
                     (uint32_t)offset, pos, s->read_count);
    }

#if (SMARTXX_DEBUG_LEVEL & 0x4)
    bool in_burst = s->read_count > SMARTXX_READ_LOG_FIRST &&
        s->read_count <= (uint64_t)SMARTXX_READ_LOG_FIRST +
                         SMARTXX_READ_LOG_COUNT;
    bool post_visor = s->read_count > SMARTXX_READ_LOG_COUNT &&
                      offset < SMARTXX_READ_LOG_POST_OFF &&
                      s->post_read_count < SMARTXX_READ_LOG_POST_COUNT;

    if (post_visor) {
        s->post_read_count++;
    }
    if (in_burst || post_visor) {
        /* The full window offset matters as much as the resolved address: it is
         * what distinguishes code running at the top of the ROM window, which a
         * bank switch pulls the ground out from under, from a deliberate read of
         * a bank's contents. */
        SXRLOG("READ   #%-3" PRIu64 " off=0x%08x -> %s 0x%06x = 0x%0*" PRIx64,
               s->read_count, (uint32_t)offset, s->tsop ? "tsop" : "flash", pos,
               (int)size * 2, val);
    }
#endif

    return val;
}

static void smartxx_flash_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned int size)
{
    SmartxxState *s = opaque;
    uint16_t cmd_addr = (uint16_t)offset;
    uint8_t raw = (uint8_t)value;

    /* Command bytes arrive permuted. Undo that here so the state machine below
     * is plain JEDEC. The permutation fixes 0xAA, 0x55 and 0x80, so a flasher
     * that does not know about the chip gets through the unlock cycles and then
     * falls over on the command itself - which is the entire point of the
     * obfuscation, so such a flasher is left to fail rather than humoured. */
    uint8_t data = smartxx_unmangle_cmd(raw);

    SXLOG("WRITE  off=0x%06x val=0x%02x (cmd 0x%02x) size=%u [state=%s bank=%u"
          " fe=%d]",
          (uint32_t)offset, raw, data, size,
          smartxx_state_name(s->flash_state), s->bank, s->flash_enable);

    if (s->flash_state != SMARTXX_FLASH_WRITE &&
        (raw == 0x90 || raw == 0xA0 || raw == 0x30 || raw == 0x10 ||
         raw == 0xF0)) {
        SXLOG("WRITE  note: 0x%02x is an unobfuscated JEDEC command, so the "
              "guest does not know this chip's command permutation", raw);
    }

    /* Reset aborts any command sequence, but must not swallow a data byte of a
     * program cycle that happens to look like one. */
    if (s->flash_state != SMARTXX_FLASH_WRITE && data == 0xF0) {
        smartxx_flash_enter_idle(s);
        return;
    }

    bool at_555 = (cmd_addr == SMARTXX_ADDR_555_MANGLED);
    bool at_aaa = (cmd_addr == SMARTXX_ADDR_AAA_MANGLED);

    switch (s->flash_state) {
    case SMARTXX_FLASH_IDLE:
        if (at_aaa && data == 0xAA) {
            s->flash_state = SMARTXX_FLASH_UNLOCK_1;
            smartxx_set_flash_mmio(s, true);
        } else {
            smartxx_rom_cache_write(s, offset, value, size);
        }
        break;

    case SMARTXX_FLASH_UNLOCK_1:
        if (at_555 && data == 0x55) {
            s->flash_state = SMARTXX_FLASH_UNLOCK_2;
        } else {
            smartxx_flash_enter_idle(s);
        }
        break;

    case SMARTXX_FLASH_UNLOCK_2:
        if (at_aaa && data == 0x80) {
            s->flash_state = SMARTXX_FLASH_ERASE_1;
        } else if (at_aaa && data == 0x90) {
            s->flash_state = SMARTXX_FLASH_CHIPID;
        } else if (at_aaa && data == 0xA0) {
            s->flash_state = SMARTXX_FLASH_WRITE;
        } else {
            smartxx_flash_enter_idle(s);
        }
        break;

    case SMARTXX_FLASH_ERASE_1:
        if (at_aaa && data == 0xAA) {
            s->flash_state = SMARTXX_FLASH_ERASE_2;
        } else {
            smartxx_flash_enter_idle(s);
        }
        break;

    case SMARTXX_FLASH_ERASE_2:
        if (at_555 && data == 0x55) {
            s->flash_state = SMARTXX_FLASH_ERASE_3;
        } else {
            smartxx_flash_enter_idle(s);
        }
        break;

    case SMARTXX_FLASH_ERASE_3:
        /* Program and erase are the only things the flash enable gates; the
         * sequence itself is allowed to run either way, so a guest that forgets
         * the register fails visibly rather than silently desynchronising. */
        if (!s->flash_enable) {
            SXLOG("ERASE  ignored, flash enable is off");
        } else if (at_aaa && data == 0x10) {
            smartxx_erase_chip(s);
        } else if (data == 0x30) {
            smartxx_erase_sector(s, offset);
        }
        smartxx_flash_enter_idle(s);
        break;

    case SMARTXX_FLASH_CHIPID:
        /* Autoselect is only left by a reset, handled above. */
        break;

    case SMARTXX_FLASH_WRITE: {
        uint32_t pos;
        uint8_t *chip = smartxx_resolve(s, offset, &pos);

        /* Program data is not permuted, and unlike erase the address is not
         * either: it goes onto the bus straight through, and through a wide
         * bank the CPLD XORs it on the way to the die in the two ranges. */
        if (!s->flash_enable) {
            SXLOG("WRITE  ignored, flash enable is off");
        } else if (chip != NULL && chip != smartxx_tsop_raw) {
            for (unsigned int i = 0; i < size; i++) {
                uint32_t at = (pos + i) % s->flash_size;
                uint8_t byte = (value >> (i * 8)) & 0xFF;
                chip[at] = smartxx_cpld_xor(s, at, byte);
            }
            smartxx_flash_mark_dirty(s);
        }
        smartxx_rebuild_rom_view(s);
        smartxx_flash_enter_idle(s);
        break;
    }
    }
}

static const MemoryRegionOps smartxx_flash_ops = {
    .read = smartxx_flash_read,
    .write = smartxx_flash_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .impl.unaligned = true,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint32_t smartxx_syscon_value(SmartxxState *s, uint32_t i);
static void smartxx_syscon_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned int size);

static uint32_t smartxx_syscon_lane(SmartxxState *s, hwaddr addr,
                                    unsigned int size)
{
    unsigned off = addr - SMARTXX_REG_SYS_DATA;
    uint32_t val = s->syscon_latch >> (off * 8);

    if (size == 1) {
        return val & 0xff;
    }
    if (size == 2) {
        return val & 0xffff;
    }
    return val;
}

static void smartxx_io_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned int size)
{
    SmartxxState *s = opaque;

    DPRINTF("write 0x%04x = 0x%02x", (unsigned)(SMARTXX_REGISTER_BASE + addr),
            (uint8_t)val);

    switch (addr) {
    case SMARTXX_REG_BANKING: {
        /* The OS rewrites the same bank twice per step of its self-check,
         * 49K times; rebuilding the 16 MB window each time would dominate. */
        if ((uint8_t)val == s->bank) {
            break;
        }
        s->bank = val;

        uint32_t bk_off, bk_size;
        if (smartxx_bank_decode(s, s->bank, &bk_off, &bk_size)) {
            SXLOG("IO write BANK=0x%02x (%u) -> off=0x%06x win=%uK%s", s->bank,
                  s->bank, bk_off, bk_size / 1024,
                  bk_off >= s->flash_size ?
                      "  ** above the fitted part, aliases to 0x000000 **" :
                      "");
        } else {
            SXLOG("IO write BANK=0x%02x (%u) -> NOT IN THE BANK MAP, reads will "
                  "return 0xFF", s->bank, s->bank);
        }
        smartxx_rebuild_rom_view(s);
        break;
    }
    case SMARTXX_REG_FLASH_ENABLE:
        s->flash_enable = (val & 1) != 0;
        smartxx_set_flash_mmio(s, s->flash_enable);
        if (!s->flash_enable) {
            smartxx_rebuild_rom_view(s);
        }
        break;
    case SMARTXX_REG_TSOP:
        /* Hand the bus to the motherboard flash. The guest reboots straight
         * after, so there is nothing to do but remember it. */
        if (val & 1) {
            s->tsop = true;
            SXLOG("IO write TSOP -> passthrough (%uK image)",
                  s->tsop_size / 1024);
            smartxx_rebuild_rom_view(s);
        }
        break;
    case SMARTXX_REG_ID:
        /* Reads are the chip id, writes are the LCD backlight. */
        s->backlight = val;
        break;
    case SMARTXX_REG_DISP:
        s->disp = val;
        break;
    case SMARTXX_REG_IO:
        s->io_reg = val;
        break;
    case SMARTXX_REG_SYS_DATA:
        s->syscon_resp1 = val;
        break;
    case SMARTXX_REG_SYS_RESP2:
        s->syscon_resp2 = val;
        break;
    case SMARTXX_REG_SYS_COMMIT:
        if ((val & 0xFF) == 0xFF) {
            uint8_t want1 = smartxx_is_opx(s) ? SMARTXX_SYSCON_OPX_RESP1
                                              : SMARTXX_SYSCON_CLS_RESP1;
            uint8_t want2 = smartxx_is_opx(s) ? SMARTXX_SYSCON_OPX_RESP2
                                              : SMARTXX_SYSCON_CLS_RESP2;
            bool ok = (s->syscon_resp1 == want1 && s->syscon_resp2 == want2);

            SXLOG("SYSCON response 0x%02x/0x%02x after %u dwords - %s "
                  "(expected 0x%02x/0x%02x for a %s)",
                  s->syscon_resp1, s->syscon_resp2, s->syscon_clocks,
                  ok ? "matches this chip's key" :
                       "does NOT match, the guest read a different key than "
                       "the one streamed to it",
                  want1, want2, smartxx_is_opx(s) ? "SmartXX OPX" : "SmartXX");
        }
        break;
    case SMARTXX_REG_SYSCON_CLOCK:
        smartxx_syscon_write(opaque, addr, val, size);
        break;
    default:
        break;
    }
}

static uint64_t smartxx_io_read(void *opaque, hwaddr addr, unsigned int size)
{
    SmartxxState *s = opaque;
    uint32_t val = 0;

    switch (addr) {
    case SMARTXX_REG_ID:
        val = smartxx_chip_id(s);
        break;
    case SMARTXX_REG_BANKING:
        val = s->bank;
        break;
    case SMARTXX_REG_FLASH_ENABLE:
        val = s->flash_enable;
        break;
    case SMARTXX_REG_DISP:
        val = s->disp;
        break;
    case SMARTXX_REG_IO:
        val = s->io_reg;
        break;
    case SMARTXX_REG_SYS_DATA:
    case SMARTXX_REG_SYS_DATA + 1:
    case SMARTXX_REG_SYS_DATA + 2:
    case SMARTXX_REG_SYS_DATA_HI:
        /* PrometheOS uses _inpd(0xF708). Some builds issue that as four byte
         * INs of 0xF708-0xF70B rather than one dword IN; the classic first
         * dword is 0x1F623100, whose low byte is 0, so returning 0 for the
         * upper lanes makes the assembled value look like "not ready". */
        val = smartxx_syscon_lane(s, addr, size);
        break;
    case SMARTXX_REG_SYSCON_CLOCK:
        val = 0;
        break;
    default:
        break;
    }

    if (addr >= SMARTXX_REG_SYS_DATA && addr <= SMARTXX_REG_SYS_DATA_HI) {
        if (!s->logged_syscon_read) {
            s->logged_syscon_read = true;
            SXLOG("SYSCON first data read port=0x%04x size=%u latch=0x%08x "
                  "clocks=%u -> 0x%08x",
                  (unsigned)(SMARTXX_REGISTER_BASE + addr), size,
                  s->syscon_latch, s->syscon_clocks, val);
        }
        /* One line per dword would be 1862 of them, so report the two ends
         * that get checked and a marker every so often in between. The
         * wait-for-ready poll after start is clocks==0; skip it here, the
         * start line already printed the first dword. */
        uint32_t index = s->syscon_clocks ? s->syscon_clocks - 1 : 0;

        if (s->syscon_clocks && addr == SMARTXX_REG_SYS_DATA &&
            (index == 0 || index + 1 >= SMARTXX_SYSCON_LENGTH ||
             (index % 512) == 0)) {
            DPRINTF("read  0x%04x = 0x%08x  (key dword %u of %u, size %u)",
                    (unsigned)(SMARTXX_REGISTER_BASE + addr), s->syscon_latch,
                    index, SMARTXX_SYSCON_LENGTH - 1, size);
        }
        return val;
    }

    DPRINTF("read  0x%04x = 0x%02x%s",
            (unsigned)(SMARTXX_REGISTER_BASE + addr), val,
            addr == SMARTXX_REG_ID           ? "  (id)" :
            addr == SMARTXX_REG_BANKING      ? "  (bank)" :
            addr == SMARTXX_REG_FLASH_ENABLE ? "  (flash enable)" :
            addr == SMARTXX_REG_DISP         ? "  (display)" :
            addr == SMARTXX_REG_IO           ? "  (io)" :
            addr == SMARTXX_REG_SYSCON_CLOCK ? "  (handshake clock)" :
                                               "  (unimplemented register)");

    return val;
}

/* One dword of the key stream: the capture, or past its end a stand-in whose
 * two ends are the values PrometheOS checks. */
static uint32_t smartxx_syscon_value(SmartxxState *s, uint32_t i)
{
    if (i < s->syscon_stream_len) {
        return s->syscon_stream[i];
    }
    if (!s->syscon_stream_overrun) {
        s->syscon_stream_overrun = true;
        SXLOG("SYSCON clock %u is past the end of the captured stream "
              "(%u dwords), continuing with the synthetic one", i,
              s->syscon_stream_len);
    }

    if (i == 0) {
        return smartxx_is_opx(s) ? SMARTXX_SYSCON_OPX_KEY_FIRST
                                 : SMARTXX_SYSCON_CLS_KEY_FIRST;
    }
    if (i == SMARTXX_SYSCON_LENGTH - 1) {
        return smartxx_is_opx(s) ? SMARTXX_SYSCON_OPX_KEY_LAST
                                 : SMARTXX_SYSCON_CLS_KEY_LAST;
    }

    uint32_t x = i * 2654435761u;
    x ^= x >> 15;
    x *= 2246822519u;
    x ^= x >> 13;

    return x;
}

static void smartxx_syscon_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned int size)
{
    SmartxxState *s = opaque;

    switch (val & 0xFF) {
    case 0x01:
        /* Present dword 0 immediately. The firmware polls 0xF708 until it is
         * non-zero before it starts clocking; leaving the latch at 0 here
         * deadlocks, and the classic key's low byte is 0x00 so a partial
         * read would still look empty. The first clock still yields dword 0
         * because clocks stays at 0 until then. */
        s->syscon_active = true;
        s->syscon_clocks = 0;
        s->syscon_latch = smartxx_syscon_value(s, 0);
        s->syscon_resp1 = 0;
        s->syscon_resp2 = 0;
        SXLOG("SYSCON start, first dword 0x%08x", s->syscon_latch);
        break;

    case 0x00:
        /* Clock, then the guest reads the dword it shifted out. The stream
         * keeps going after the 0x80 marker: the OS clocks another 49136
         * dwords for its self-check. */
        if (s->syscon_clocks < 4 ||
            s->syscon_clocks + 1 == SMARTXX_SYSCON_LENGTH) {
            SXLOG("SYSCON clock %u -> 0x%08x", s->syscon_clocks,
                  smartxx_syscon_value(s, s->syscon_clocks));
        }
        s->syscon_latch = smartxx_syscon_value(s, s->syscon_clocks);
        s->syscon_clocks++;
        break;

    case 0x80:
        s->syscon_active = false;
        SXLOG("SYSCON handshake ended after %u dwords%s", s->syscon_clocks,
              s->syscon_clocks == SMARTXX_SYSCON_LENGTH ?
                  "" : " - NOT the expected 0x746");
        break;

    default:
        DPRINTF("write 0x%04x = 0x%02x  (unexpected handshake command)",
                SMARTXX_SYSCON_CLOCK_PORT, (uint8_t)val);
        break;
    }
}

static const MemoryRegionOps smartxx_io_ops = {
    .read  = smartxx_io_read,
    .write = smartxx_io_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .impl.unaligned = true,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* The register block seen through LPC memory cycles at 0x8xxxxxxx. Only the
 * low 16 bits select a register; anything outside 0xF700-0xF7FF is open bus. */
static uint64_t smartxx_lpc_mem_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    uint32_t off = addr & 0xFFFF;

    if (off >= SMARTXX_REGISTER_BASE &&
        off < SMARTXX_REGISTER_BASE + SMARTXX_REGISTER_SIZE) {
        return smartxx_io_read(opaque, off - SMARTXX_REGISTER_BASE, size);
    }
    return 0xFFFFFFFFu >> ((4 - size) * 8);
}

static void smartxx_lpc_mem_write(void *opaque, hwaddr addr, uint64_t val,
                                  unsigned int size)
{
    uint32_t off = addr & 0xFFFF;

    if (off >= SMARTXX_REGISTER_BASE &&
        off < SMARTXX_REGISTER_BASE + SMARTXX_REGISTER_SIZE) {
        smartxx_io_write(opaque, off - SMARTXX_REGISTER_BASE, val, size);
    }
}

static const MemoryRegionOps smartxx_lpc_mem_ops = {
    .read  = smartxx_lpc_mem_read,
    .write = smartxx_lpc_mem_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .valid.unaligned = true,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .impl.unaligned = true,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static bool smartxx_load_image(const char *file, uint8_t *buf, uint32_t size,
                               Error **errp)
{
    int fd = qemu_open(file, O_RDONLY | O_BINARY, NULL);
    if (fd < 0) {
        error_setg(errp, "smartxx: '%s' could not be opened", file);
        return false;
    }

    int rc = read(fd, buf, size);
    close(fd);

    if (rc != (int)size) {
        error_setg(errp, "smartxx: '%s' read failure", file);
        return false;
    }

    return true;
}

/* Load the onboard TSOP flash image, used once the guest asks for
 * passthrough. */
static void smartxx_load_tsop(SmartxxState *s)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    const char *bios_name = ms->firmware;

    s->tsop_size = 0;

    if (bios_name == NULL || *bios_name == '\0') {
        return;
    }

    char *filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename == NULL) {
        return;
    }

    int size = get_image_size(filename, NULL);
    if (size > 0 && size <= SMARTXX_TSOP_MAX_SIZE && (size % 65536) == 0) {
        int fd = qemu_open(filename, O_RDONLY | O_BINARY, NULL);
        if (fd >= 0) {
            if (read(fd, smartxx_tsop_raw, size) == size) {
                s->tsop_size = size;
            }
            close(fd);
        }
    }

    g_free(filename);
}

static void smartxx_realize(DeviceState *dev, Error **errp)
{
    SmartxxState *s = SMARTXX_DEVICE(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    Error *err = NULL;

    modchip_enabled = true;

    if (s->flash_size != SMARTXX_FLASH_2MB &&
        s->flash_size != SMARTXX_FLASH_4MB) {
        error_setg(errp, "smartxx: flash-size of %u, expected 2 or 4 MB",
                   s->flash_size);
        return;
    }

    /* The selected variant fixes the size, so a truncated or padded image is
     * rejected rather than booting to a confusing hang. */
    int image_size = get_image_size(s->rom_file, NULL);
    if (image_size != (int)s->flash_size) {
        error_setg(errp, "smartxx: '%s' size of %d, expected %u for the %u MB "
                   "variant", s->rom_file, image_size, s->flash_size,
                   s->flash_size / (1024 * 1024));
        return;
    }

    memset(smartxx_flash_raw, 0xFF, SMARTXX_FLASH_MAX_SIZE);

    if (!smartxx_load_image(s->rom_file, smartxx_flash_raw, s->flash_size,
                            errp)) {
        return;
    }
    s->syscon_stream = smartxx_is_opx(s) ? smartxx_key_stream_opx
                                         : smartxx_key_stream_classic;
    s->syscon_stream_len = SMARTXX_KEY_STREAM_LENGTH;

    smartxx_load_tsop(s);

    /* Read MCPX Dump (512 bytes) */
    const char *bootrom_file =
        object_property_get_str(qdev_get_machine(), "bootrom", NULL);
    bool have_bootrom = false;

    if ((bootrom_file != NULL) && *bootrom_file) {
        char *filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bootrom_file);
        assert(filename);
        int fd = qemu_open(filename, O_RDONLY | O_BINARY, NULL);
        assert(fd >= 0);
        have_bootrom = read(fd, smartxx_mcpx_raw, MCPX_SIZE) == MCPX_SIZE;
        close(fd);
        g_free(filename);
    }

    /* Power-on state. There are no bank switches on this chip, so it always
     * comes up on the bootloader bank and its menu decides what to run. */
    s->bank = SMARTXX_BANK_BOOTLOADER;
    s->flash_enable = false;
    s->tsop = false;
    s->flash_state = SMARTXX_FLASH_IDLE;
    s->read_count = 0;
    s->post_read_count = 0;
    s->read_log_key = 0xFFFFFFFF;
    s->logged_payload = false;
    s->logged_syscon_read = false;
    s->syscon_active = false;
    s->syscon_clocks = 0;
    s->syscon_latch = 0;
    s->syscon_resp1 = 0;
    s->syscon_resp2 = 0;
    s->syscon_stream_overrun = false;
    s->rom_mapped = false;
    s->flash_mmio_on = false;

    s->dirty = false;
    s->flush_timer =
        timer_new_ms(QEMU_CLOCK_REALTIME, smartxx_flash_flush_cb, s);
    s->exit_notifier.notify = smartxx_flash_exit_notify;
    qemu_add_exit_notifier(&s->exit_notifier);

    #define ROM_START SMARTXX_LPC_ROM_START
    #define ROM_AREA  SMARTXX_LPC_ROM_SIZE

    /* ROM device: array reads come straight from the RAM tiles, every write
     * reaches the command state machine whether or not the JEDEC overlay is
     * up. The official OS issues its autoselect twice in a row, and the
     * second sequence used to land on a read-only region and vanish. */
    if (!memory_region_init_rom_device(&s->flash_mem, OBJECT(s),
                                       &smartxx_flash_ops, s, "smartxx.bios",
                                       ROM_AREA, errp)) {
        return;
    }
    memory_region_rom_device_set_romd(&s->flash_mem, true);
    memory_region_init_io(&s->flash_mmio, OBJECT(s), &smartxx_flash_ops, s,
                          "smartxx.flash-mmio", ROM_AREA);
    smartxx_rebuild_rom_view(s);

    memory_region_add_subregion(rom_memory__, ROM_START, &s->flash_mem);
    memory_region_add_subregion_overlap(rom_memory__, ROM_START,
                                        &s->flash_mmio, 2);
    memory_region_set_enabled(&s->flash_mmio, false);
    s->rom_mapped = true;

    /* MCPX boot ROM, overlaid on top of the flash at the very top of the ROM
     * window. The real part only drives the last 512 bytes, but the overlay has
     * to be page aligned to stay off the slow path, so the rest of the page is
     * primed with whatever the flash presents there - otherwise a 2BL that
     * lives right below the boot ROM would read back as zeroes.
     *
     * The subregion has to be named "xbox.mcpx": xbox_lpc_enable_mcpx_rom()
     * looks it up by that name to switch the boot ROM off, whether that is a
     * successful 2BL or the Visor panic path, which is why it is added
     * directly rather than by alias. */
    unsigned int page_size = 4096;
    MemoryRegion *mr_mcpx = g_malloc(sizeof(MemoryRegion));
    memory_region_init_ram(mr_mcpx, NULL, "xbox.mcpx", page_size, &err);
    uint8_t *mcpx_data = memory_region_get_ram_ptr(mr_mcpx);

    unsigned int primed = have_bootrom ? page_size - MCPX_SIZE : page_size;
    for (unsigned int i = 0; i < primed; i++) {
        uint32_t pos;
        uint8_t *chip = smartxx_resolve(s, ROM_AREA - page_size + i, &pos);
        uint8_t byte = chip ? chip[pos] : 0xFF;

        /* This page is the top of the bootloader bank, read through bank 0,
         * which sees the die raw; the helper knows that. */
        if (chip == smartxx_flash_raw) {
            byte = smartxx_cpld_xor(s, pos, byte);
        }

        mcpx_data[i] = byte;
    }

    /* Without a boot ROM the whole page stays flash, so the CPU boots straight
     * from the image's own reset vector. */
    if (have_bootrom) {
        memcpy(mcpx_data + page_size - MCPX_SIZE, smartxx_mcpx_raw, MCPX_SIZE);
    }

    memory_region_add_subregion_overlap(rom_memory__, -page_size, mr_mcpx, 1);

    memory_region_init_io(&s->io, OBJECT(s), &smartxx_io_ops, s, "smartxx.io",
                          SMARTXX_REGISTER_SIZE);
    isa_register_ioport(isa, &s->io, SMARTXX_REGISTER_BASE);

    memory_region_init_io(&s->lpc_mem, OBJECT(s), &smartxx_lpc_mem_ops, s,
                          "smartxx.lpc-mem", SMARTXX_LPC_MEM_SIZE);
    memory_region_add_subregion(rom_memory__, SMARTXX_LPC_MEM_START,
                                &s->lpc_mem);

    SXLOG("READY  rom='%s' %s chip=0x%02x flash=%uK (id 0x%02x/0x%02x) tsop=%uK "
          "io=0x%04x-0x%04x bank=%u rom=on key=%u dwords",
          s->rom_file, smartxx_is_opx(s) ? "OPX" : "classic", smartxx_chip_id(s),
          s->flash_size / 1024, SMARTXX_MANUF_ID,
          smartxx_dev_id(s), s->tsop_size / 1024, SMARTXX_REGISTER_BASE,
          SMARTXX_REGISTER_BASE + SMARTXX_REGISTER_SIZE - 1, s->bank,
          s->syscon_stream_len);
}

static const Property smartxx_properties[] = {
    DEFINE_PROP_STRING("rom-path", SmartxxState, rom_file),
    DEFINE_PROP_UINT32("flash-size", SmartxxState, flash_size,
                       SMARTXX_FLASH_4MB),
};

static const VMStateDescription vmstate_smartxx = {
    .name = "modchip-smartxx",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void smartxx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "SmartXX modchip";
    dc->realize = smartxx_realize;
    dc->vmsd = &vmstate_smartxx;
    device_class_set_props(dc, smartxx_properties);
}

static const TypeInfo smartxx_type_info = {
    .name          = "modchip-smartxx",
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(SmartxxState),
    .class_init    = smartxx_class_init,
};

static void smartxx_register_types(void)
{
    type_register_static(&smartxx_type_info);
}

type_init(smartxx_register_types)
