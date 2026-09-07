/*
 * Xecuter 3 Modchip
 *
 * Modelled on the ModXover firmware re-implementation
 * (https://github.com/Team-Resurgent/ModXover) and loosely based on the Xenium
 * modchip device in modchip_xenium.c.
 *
 * The LCD, WS2812 and IO expander parts of the real chip are not emulated, and
 * the four physical bank-select switches are represented by a fixed property
 * value; banking is otherwise driven entirely by the I/O registers.
 *
 * Copyright (c) 2026 Team Resurgent
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
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
#include "qemu/timer.h"
#include "qemu/notify.h"

/* smbus_xbox_smc.c: was the reset in progress requested as a warm reset? */
bool xbox_smc_take_warm_reset(void);

/* I/O register block, matching ModXover's MODXO_REGISTER_X3_* */
#define XECUTER_REGISTER_BASE     0xF500
#define XECUTER_REGISTER_SIZE     16
#define XECUTER_REG_VERSION       0x0
#define XECUTER_REG_CONTROL       0x1
#define XECUTER_REG_STATUS        0x2
#define XECUTER_REG_VERSION_2     0x9

/* The display registers at 0x3-0x8 belong to lcd_xecuter.c, which claims them
 * out of this window when an LCD is attached. */

#define XECUTER_VERSION           0xE1
#define XECUTER_VERSION_2         0x69

/* CONTROL bits */
#define XECUTER_CONTROL_TSOP      0x20
#define XECUTER_CONTROL_BACKUP    0x80

/* STATUS bits: when SOFTCONTROL is set the bank comes from the low nibble of
 * STATUS, otherwise from the physical bank switches. */
#define XECUTER_STATUS_SOFTCONTROL 0x80
#define XECUTER_STATUS_BANK_MASK   0x0F

/* AM29F016D-compatible main flash, SST-style backup flash */
#define XECUTER_MAIN_MANUF_ID     0x01
#define XECUTER_MAIN_DEV_ID       0xAD
#define XECUTER_BACKUP_MANUF_ID   0x1C
#define XECUTER_BACKUP_DEV_ID     0x92

#define XECUTER_MAIN_SIZE         (2 * 1024 * 1024)
#define XECUTER_BACKUP_SIZE       (256 * 1024)
#define XECUTER_TSOP_MAX_SIZE     (1024 * 1024)

/* Flash command / unlock addresses. ModXover decodes the low 16 bits and
 * expects the x16-style 0x5555 / 0x2AAA pair; the byte-mode 0x555 / 0x2AA pair
 * used by some flashers is accepted as well. */
#define XECUTER_ADDR_5555         0x5555
#define XECUTER_ADDR_2AAA         0x2AAA
#define XECUTER_ADDR_555          0x0555
#define XECUTER_ADDR_2AA          0x02AA

/* After the last flash write/erase, wait this long with no further activity
 * before persisting the image back to its file. */
#define XECUTER_FLASH_FLUSH_DELAY_MS 2000
#define MCPX_SIZE (512)

extern MemoryRegion *rom_memory__; //FIXME

/* Set by the machine setup (vl.c) when a modchip provides its own flash
 * mapping, so the stock BIOS/MCPX setup in xbox.c is skipped. */
extern bool modchip_enabled;

/* Debug verbosity as a bitmask:
 *   bit 0 (0x1) GENERAL - I/O register accesses
 *   bit 1 (0x2) FLASH   - flash command decode, banking, program/erase and
 *                         persistence
 * Temporarily enabled; set to 0x0 to silence. */
#define XECUTER_DEBUG_LEVEL 0x0

/* Everything goes to modchip.log in xemu's working directory as well as to
 * stderr, since the windowed Windows build has nowhere to send stderr and any
 * console attached to it can go away mid-boot. */
void modchip_debug_log(const char *tag, const char *fmt, ...)
    G_GNUC_PRINTF(2, 3);

void modchip_debug_log(const char *tag, const char *fmt, ...)
{
    static FILE *log_file;
    static bool log_file_tried;

    if (!log_file_tried) {
        log_file_tried = true;
        log_file = fopen("modchip.log", "w");
    }

    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (log_file) {
        fprintf(log_file, "[%s] %s\n", tag, buf);
        fflush(log_file);
    }

    fprintf(stderr, "[%s] %s\n", tag, buf);
    fflush(stderr);
}

#if (XECUTER_DEBUG_LEVEL & 0x1)
# define DPRINTF(fmt, ...) modchip_debug_log("xecuter-io", fmt, ## __VA_ARGS__)
#else
# define DPRINTF(fmt, ...) do { } while (0)
#endif

#if (XECUTER_DEBUG_LEVEL & 0x2)
# define XFLOG(fmt, ...) modchip_debug_log("xecuter-flash", fmt, ## __VA_ARGS__)
#else
# define XFLOG(fmt, ...) do { } while (0)
#endif

static uint8_t xecuter_main_raw[XECUTER_MAIN_SIZE];
static uint8_t xecuter_backup_raw[XECUTER_BACKUP_SIZE];
static uint8_t xecuter_tsop_raw[XECUTER_TSOP_MAX_SIZE];
static uint8_t xecuter_mcpx_raw[MCPX_SIZE];

typedef enum {
    XECUTER_FLASH_IDLE,
    XECUTER_FLASH_UNLOCK_1,
    XECUTER_FLASH_UNLOCK_2,
    XECUTER_FLASH_ERASE_1,
    XECUTER_FLASH_ERASE_2,
    XECUTER_FLASH_ERASE_3,
    XECUTER_FLASH_CHIPID,
    XECUTER_FLASH_WRITE,
} XecuterFlashState;

G_GNUC_UNUSED static const char *xecuter_state_name(XecuterFlashState st)
{
    switch (st) {
    case XECUTER_FLASH_IDLE:     return "IDLE";
    case XECUTER_FLASH_UNLOCK_1: return "UNLOCK_1";
    case XECUTER_FLASH_UNLOCK_2: return "UNLOCK_2";
    case XECUTER_FLASH_ERASE_1:  return "ERASE_1";
    case XECUTER_FLASH_ERASE_2:  return "ERASE_2";
    case XECUTER_FLASH_ERASE_3:  return "ERASE_3";
    case XECUTER_FLASH_CHIPID:   return "CHIPID";
    case XECUTER_FLASH_WRITE:    return "WRITE";
    }
    return "?";
}

typedef struct XecuterState {
    ISADevice dev;
    MemoryRegion io;
    MemoryRegion flash_mem;

    char *rom_file;      /* 2 MB main flash image */
    char *recovery_file; /* 256 KB recovery (backup) flash image */

    /* Stand-in for the four physical bank-select switches, used while soft
     * control is disabled. */
    uint8_t bank_switches;

    uint8_t control;
    uint8_t status;

    XecuterFlashState flash_state;

    uint32_t tsop_size;

    QEMUTimer *flush_timer;
    Notifier exit_notifier;
    bool main_dirty;
    bool backup_dirty;

    /* Counts reads so that only the first few are logged in full. */
    uint64_t read_count;
} XecuterState;

#define XECUTER_DEVICE(obj) \
    OBJECT_CHECK(XecuterState, (obj), "modchip-xecuter")

static bool xecuter_is_backup(XecuterState *s)
{
    return (s->control & XECUTER_CONTROL_BACKUP) != 0;
}

static bool xecuter_is_tsop(XecuterState *s)
{
    return (s->control & XECUTER_CONTROL_TSOP) != 0;
}

static uint8_t xecuter_get_bank(XecuterState *s)
{
    if (!(s->status & XECUTER_STATUS_SOFTCONTROL)) {
        return s->bank_switches & XECUTER_STATUS_BANK_MASK;
    }
    return s->status & XECUTER_STATUS_BANK_MASK;
}

/* Window size the selected bank exposes in the ROM address space. Banks 0-7
 * are 256 KB, 8-11 are 512 KB, 12-13 are 1 MB and 14-15 map the whole 2 MB. */
static uint32_t xecuter_bank_mask(XecuterState *s, uint8_t bank)
{
    if (xecuter_is_backup(s)) {
        return 0x3FFFF;
    }

    switch (bank) {
    case 0 ... 7:   return 0x3FFFF;
    case 8 ... 11:  return 0x7FFFF;
    case 12 ... 13: return 0xFFFFF;
    default:        return 0x1FFFFF;
    }
}

static uint32_t xecuter_bank_offset(XecuterState *s, uint8_t bank)
{
    if (xecuter_is_backup(s)) {
        return 0;
    }

    /* Bank names follow the X3 switch reference. The switch value is formed
     * with switch 1 as the low bit and "on" reading as 0, so e.g. Bank 1-4
     * (on on off off) is 12. */
    switch (bank) {
    case 0:  return 0x000000; /* Bank 1    256K */
    case 1:  return 0x040000; /* Bank 2    256K */
    case 2:  return 0x080000; /* Bank 3    256K */
    case 3:  return 0x0C0000; /* Bank 4    256K */
    case 4:  return 0x100000; /* Bank 5    256K */
    case 5:  return 0x140000; /* Bank 6    256K */
    case 6:  return 0x180000; /* Bank 7    256K */
    case 7:  return 0x1C0000; /* Bank 8    256K */
    case 8:  return 0x000000; /* Bank 1-2  512K */
    case 9:  return 0x080000; /* Bank 3-4  512K */
    case 10: return 0x100000; /* Bank 5-6  512K */
    case 11: return 0x180000; /* Bank 7-8  512K */
    case 12: return 0x000000; /* Bank 1-4  1M */
    case 13: return 0x100000; /* Bank 5-8  1M */
    default: return 0x000000; /* Bank 1-8  2M */
    }
}

/* Resolve a ROM-space offset to the backing image and the address within it.
 * Returns NULL when nothing is mapped. */
static uint8_t *xecuter_resolve(XecuterState *s, hwaddr offset, uint32_t *pos)
{
    if (xecuter_is_tsop(s)) {
        /* Passthrough: the modchip releases the bus and the onboard TSOP flash
         * responds, mirrored across the ROM window. */
        if (s->tsop_size == 0) {
            return NULL;
        }
        *pos = offset % s->tsop_size;
        return xecuter_tsop_raw;
    }

    uint8_t bank = xecuter_get_bank(s);
    *pos = (offset & xecuter_bank_mask(s, bank)) + xecuter_bank_offset(s, bank);

    if (xecuter_is_backup(s)) {
        *pos %= XECUTER_BACKUP_SIZE;
        return xecuter_backup_raw;
    }

    *pos %= XECUTER_MAIN_SIZE;
    return xecuter_main_raw;
}

/* Sector geometry of the active chip. The 2 MB main flash is uniform 64 KB;
 * the 256 KB backup flash has smaller boot sectors at the top. */
static uint32_t xecuter_sector_size(XecuterState *s, uint32_t pos)
{
    if (!xecuter_is_backup(s)) {
        return 64 * 1024;
    }

    if (pos <= 0x02FFFF) {
        return 64 * 1024;
    } else if (pos <= 0x037FFF) {
        return 32 * 1024;
    } else if (pos <= 0x03BFFF) {
        return 8 * 1024;
    } else {
        return 16 * 1024;
    }
}

static void xecuter_flash_flush(XecuterState *s)
{
    struct {
        bool *dirty;
        const char *file;
        const uint8_t *data;
        uint32_t size;
    } images[] = {
        { &s->main_dirty, s->rom_file, xecuter_main_raw, XECUTER_MAIN_SIZE },
        { &s->backup_dirty, s->recovery_file, xecuter_backup_raw,
          XECUTER_BACKUP_SIZE },
    };

    for (int i = 0; i < ARRAY_SIZE(images); i++) {
        if (!*images[i].dirty || images[i].file == NULL ||
            *images[i].file == '\0') {
            continue;
        }

        int fd = qemu_open(images[i].file, O_WRONLY | O_BINARY, NULL);
        if (fd < 0) {
            fprintf(stderr, "xecuter: could not open '%s' to persist flash\n",
                    images[i].file);
            continue;
        }

        ssize_t written = write(fd, images[i].data, images[i].size);
        close(fd);

        if (written != (ssize_t)images[i].size) {
            fprintf(stderr,
                    "xecuter: short write persisting flash to '%s' (%zd/%u)\n",
                    images[i].file, written, images[i].size);
            continue;
        }

        *images[i].dirty = false;
        XFLOG("PERSIST wrote %u bytes back to '%s'", images[i].size,
              images[i].file);
    }
}

static void xecuter_flash_flush_cb(void *opaque)
{
    xecuter_flash_flush((XecuterState *)opaque);
}

static void xecuter_flash_exit_notify(Notifier *n, void *opaque)
{
    XecuterState *s = container_of(n, XecuterState, exit_notifier);
    xecuter_flash_flush(s);
}

static void xecuter_flash_mark_dirty(XecuterState *s)
{
    if (xecuter_is_backup(s)) {
        s->backup_dirty = true;
    } else {
        s->main_dirty = true;
    }

    if (s->flush_timer) {
        timer_mod(s->flush_timer, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) +
                                      XECUTER_FLASH_FLUSH_DELAY_MS);
    }
}

static void xecuter_erase_sector(XecuterState *s, hwaddr offset)
{
    uint32_t pos;
    uint8_t *chip = xecuter_resolve(s, offset, &pos);
    if (chip == NULL || chip == xecuter_tsop_raw) {
        return;
    }

    uint32_t chip_size =
        xecuter_is_backup(s) ? XECUTER_BACKUP_SIZE : XECUTER_MAIN_SIZE;
    uint32_t sector_size = xecuter_sector_size(s, pos);
    uint32_t base = pos & ~(sector_size - 1);

    if (base + sector_size > chip_size) {
        sector_size = chip_size - base;
    }

    XFLOG("ERASE  sector base=0x%06x size=%uK (target 0x%06x)", base,
          sector_size / 1024, pos);
    memset(&chip[base], 0xFF, sector_size);
    xecuter_flash_mark_dirty(s);
}

static void xecuter_erase_chip(XecuterState *s)
{
    if (xecuter_is_tsop(s)) {
        return;
    }

    if (xecuter_is_backup(s)) {
        memset(xecuter_backup_raw, 0xFF, XECUTER_BACKUP_SIZE);
    } else {
        memset(xecuter_main_raw, 0xFF, XECUTER_MAIN_SIZE);
    }

    XFLOG("ERASE  whole %s chip", xecuter_is_backup(s) ? "backup" : "main");
    xecuter_flash_mark_dirty(s);
}

static uint64_t xecuter_flash_read(void *opaque, hwaddr offset, unsigned size)
{
    XecuterState *s = opaque;

    if (s->flash_state == XECUTER_FLASH_CHIPID) {
        bool backup = xecuter_is_backup(s);
        uint8_t id;
        const char *what;

        /* Autoselect only decodes the low address bits, so the IDs are
         * mirrored throughout the window. A guest that reads a 0x7F
         * continuation code retries at 0x100/0x101 and relies on this. */
        switch (offset & 0x03) {
        case 0:
            id = backup ? XECUTER_BACKUP_MANUF_ID : XECUTER_MAIN_MANUF_ID;
            what = "manufacturer";
            break;
        case 1:
            id = backup ? XECUTER_BACKUP_DEV_ID : XECUTER_MAIN_DEV_ID;
            what = "device";
            break;
        default:
            /* Sector protect verify: nothing is protected. */
            id = 0;
            what = "sector-protect";
            break;
        }

        XFLOG("CHIPID %s chip %s id at off=0x%06x -> 0x%02x",
              backup ? "recovery" : "main", what, (uint32_t)offset, id);

        return id;
    }

    uint32_t pos;
    uint8_t *chip = xecuter_resolve(s, offset, &pos);
    if (chip == NULL) {
        return 0xFFFFFFFF >> ((4 - size) * 8);
    }

    uint64_t val = 0;
    for (unsigned int i = 0; i < size; i++) {
        val |= (uint64_t)chip[pos + i] << (i * 8);
    }

    s->read_count++;

#if (XECUTER_DEBUG_LEVEL & 0x2)
    /* Log the first few reads in full, then only a change of selected
     * chip/bank, which keeps the log readable however hot this path gets. */
    const char *chip_name = chip == xecuter_tsop_raw ? "tsop" :
                            (chip == xecuter_backup_raw ? "recovery" : "main");

    static uint32_t last_key = 0xFFFFFFFF;
    uint32_t key = ((uint32_t)xecuter_get_bank(s) << 8) | (uint8_t)chip_name[0];

    if (s->read_count <= 16) {
        XFLOG("READ   off=0x%06x -> %s+0x%06x size=%u val=0x%0*x",
              (uint32_t)offset, chip_name, pos, size, size * 2, (uint32_t)val);
    } else if (key != last_key) {
        XFLOG("READ   mapping now %s bank=%u, at off=0x%06x (read #%" PRIu64 ")",
              chip_name, xecuter_get_bank(s), (uint32_t)offset, s->read_count);
    }

    last_key = key;
#endif

    return val;
}

static void xecuter_flash_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned int size)
{
    XecuterState *s = opaque;
    uint8_t data = (uint8_t)value;
    uint16_t cmd = (uint16_t)offset;

    XFLOG("WRITE  off=0x%06x val=0x%02x size=%u [state=%s bank=%u]",
          (uint32_t)offset, data, size, xecuter_state_name(s->flash_state),
          xecuter_get_bank(s));

    /* Reset aborts any command sequence, but must not swallow a data byte of a
     * program cycle that happens to be 0xF0. */
    if (s->flash_state != XECUTER_FLASH_WRITE && data == 0xF0) {
        s->flash_state = XECUTER_FLASH_IDLE;
        return;
    }

    bool at_5555 = (cmd == XECUTER_ADDR_5555) || (cmd == XECUTER_ADDR_555);
    bool at_2aaa = (cmd == XECUTER_ADDR_2AAA) || (cmd == XECUTER_ADDR_2AA);

    switch (s->flash_state) {
    case XECUTER_FLASH_IDLE:
        if (at_5555 && data == 0xAA) {
            s->flash_state = XECUTER_FLASH_UNLOCK_1;
        }
        break;

    case XECUTER_FLASH_UNLOCK_1:
        s->flash_state = (at_2aaa && data == 0x55) ? XECUTER_FLASH_UNLOCK_2 :
                                                     XECUTER_FLASH_IDLE;
        break;

    case XECUTER_FLASH_UNLOCK_2:
        if (at_5555 && data == 0x80) {
            s->flash_state = XECUTER_FLASH_ERASE_1;
        } else if (at_5555 && data == 0x90) {
            s->flash_state = XECUTER_FLASH_CHIPID;
        } else if (at_5555 && data == 0xA0) {
            s->flash_state = XECUTER_FLASH_WRITE;
        } else {
            s->flash_state = XECUTER_FLASH_IDLE;
        }
        break;

    case XECUTER_FLASH_ERASE_1:
        s->flash_state = (at_5555 && data == 0xAA) ? XECUTER_FLASH_ERASE_2 :
                                                     XECUTER_FLASH_IDLE;
        break;

    case XECUTER_FLASH_ERASE_2:
        s->flash_state = (at_2aaa && data == 0x55) ? XECUTER_FLASH_ERASE_3 :
                                                     XECUTER_FLASH_IDLE;
        break;

    case XECUTER_FLASH_ERASE_3:
        if (at_5555 && data == 0x10) {
            xecuter_erase_chip(s);
        } else if (data == 0x30) {
            xecuter_erase_sector(s, offset);
        }
        s->flash_state = XECUTER_FLASH_IDLE;
        break;

    case XECUTER_FLASH_CHIPID:
        /* Autoselect is only left by a reset, handled above. */
        break;

    case XECUTER_FLASH_WRITE: {
        uint32_t pos;
        uint8_t *chip = xecuter_resolve(s, offset, &pos);
        if (chip != NULL && chip != xecuter_tsop_raw) {
            for (unsigned int i = 0; i < size; i++) {
                chip[pos + i] = (value >> (i * 8)) & 0xFF;
            }
            xecuter_flash_mark_dirty(s);
        }
        s->flash_state = XECUTER_FLASH_IDLE;
        break;
    }
    }
}

static const MemoryRegionOps xecuter_flash_ops = {
    .read = xecuter_flash_read,
    .write = xecuter_flash_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void xecuter_io_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned int size)
{
    XecuterState *s = opaque;

    DPRINTF("write 0x%04x = 0x%02x", (unsigned)(XECUTER_REGISTER_BASE + addr),
            (uint8_t)val);

    switch (addr) {
    case XECUTER_REG_CONTROL:
        s->control = val;
        XFLOG("IO write CONTROL=0x%02x -> chip=%s bank=%u off=0x%06x "
              "win=%uK", s->control,
              xecuter_is_tsop(s) ? "tsop" :
                                   (xecuter_is_backup(s) ? "recovery" : "main"),
              xecuter_get_bank(s),
              xecuter_bank_offset(s, xecuter_get_bank(s)),
              (xecuter_bank_mask(s, xecuter_get_bank(s)) + 1) / 1024);
        break;
    case XECUTER_REG_STATUS:
        s->status = val;
        XFLOG("IO write STATUS=0x%02x -> soft=%d bank=%u off=0x%06x win=%uK",
              s->status, (s->status & XECUTER_STATUS_SOFTCONTROL) != 0,
              xecuter_get_bank(s),
              xecuter_bank_offset(s, xecuter_get_bank(s)),
              (xecuter_bank_mask(s, xecuter_get_bank(s)) + 1) / 1024);
        break;
    default:
        /* Version registers are read-only; the remaining registers are not
         * emulated. */
        break;
    }
}

static uint64_t xecuter_io_read(void *opaque, hwaddr addr, unsigned int size)
{
    XecuterState *s = opaque;
    uint32_t val = 0;

    switch (addr) {
    case XECUTER_REG_VERSION:
        val = XECUTER_VERSION;
        break;
    case XECUTER_REG_VERSION_2:
        val = XECUTER_VERSION_2;
        break;
    case XECUTER_REG_CONTROL:
        /* Without soft control the register reads back the bank switches. */
        val = (s->status & XECUTER_STATUS_SOFTCONTROL) ?
                  s->control :
                  (s->bank_switches & XECUTER_STATUS_BANK_MASK);
        break;
    case XECUTER_REG_STATUS:
        val = s->status;
        break;
    default:
        break;
    }

    DPRINTF("read  0x%04x = 0x%02x%s",
            (unsigned)(XECUTER_REGISTER_BASE + addr), val,
            addr == XECUTER_REG_VERSION   ? "  (version)" :
            addr == XECUTER_REG_VERSION_2 ? "  (version2)" :
            addr == XECUTER_REG_CONTROL   ? "  (control)" :
            addr == XECUTER_REG_STATUS    ? "  (status)" :
                                            "  (unimplemented register)");

    return val;
}

static const MemoryRegionOps xecuter_io_ops = {
    .read  = xecuter_io_read,
    .write = xecuter_io_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

/* Read `size` bytes of `file` into `buf`, returning true on success. */
static bool xecuter_load_image(const char *file, uint8_t *buf, uint32_t size,
                               Error **errp)
{
    int image_size = get_image_size(file, NULL);
    if (image_size != (int)size) {
        error_setg(errp, "xecuter: '%s' size of %d, expected %u", file,
                   image_size, size);
        return false;
    }

    int fd = qemu_open(file, O_RDONLY | O_BINARY, NULL);
    if (fd < 0) {
        error_setg(errp, "xecuter: '%s' could not be opened", file);
        return false;
    }

    int rc = read(fd, buf, size);
    close(fd);

    if (rc != (int)size) {
        error_setg(errp, "xecuter: '%s' read failure", file);
        return false;
    }

    return true;
}

/* Load the onboard TSOP flash image, used while CONTROL selects passthrough. */
static void xecuter_load_tsop(XecuterState *s)
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
    if (size > 0 && size <= XECUTER_TSOP_MAX_SIZE && (size % 65536) == 0) {
        int fd = qemu_open(filename, O_RDONLY | O_BINARY, NULL);
        if (fd >= 0) {
            if (read(fd, xecuter_tsop_raw, size) == size) {
                s->tsop_size = size;
            }
            close(fd);
        }
    }

    g_free(filename);
}

/* Bank selection survives a warm reset (that is how a slot is booted); a
 * power cycle, or a reset from the host UI, returns to the power-on state. */
static void xecuter_reset(DeviceState *dev)
{
    XecuterState *s = XECUTER_DEVICE(dev);

    if (!xbox_smc_take_warm_reset()) {
        s->status = 0x00;
        s->control = 0x0F;
    }
    s->flash_state = XECUTER_FLASH_IDLE;
}

static void xecuter_realize(DeviceState *dev, Error **errp)
{
    XecuterState *s = XECUTER_DEVICE(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    Error *err = NULL;

    modchip_enabled = true;

    memset(xecuter_main_raw, 0xFF, XECUTER_MAIN_SIZE);
    memset(xecuter_backup_raw, 0xFF, XECUTER_BACKUP_SIZE);

    if (!xecuter_load_image(s->rom_file, xecuter_main_raw, XECUTER_MAIN_SIZE,
                            errp)) {
        return;
    }

    if (!xecuter_load_image(s->recovery_file, xecuter_backup_raw,
                            XECUTER_BACKUP_SIZE, errp)) {
        return;
    }

    xecuter_load_tsop(s);

    /* Read MCPX Dump (512 bytes) */
    const char *bootrom_file =
        object_property_get_str(qdev_get_machine(), "bootrom", NULL);
    bool have_bootrom = false;

    if ((bootrom_file != NULL) && *bootrom_file) {
        char *filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bootrom_file);
        assert(filename);
        int fd = qemu_open(filename, O_RDONLY | O_BINARY, NULL);
        assert(fd >= 0);
        have_bootrom = read(fd, xecuter_mcpx_raw, MCPX_SIZE) == MCPX_SIZE;
        close(fd);
        g_free(filename);
    }

    /* Power-on state, as in ModXover's powerup(): hardware bank switches
     * select the bank until the guest enables soft control. */
    s->status = 0x00;
    s->control = 0x0F;
    s->flash_state = XECUTER_FLASH_IDLE;

    s->main_dirty = false;
    s->backup_dirty = false;
    s->flush_timer =
        timer_new_ms(QEMU_CLOCK_REALTIME, xecuter_flash_flush_cb, s);
    s->exit_notifier.notify = xecuter_flash_exit_notify;
    qemu_add_exit_notifier(&s->exit_notifier);
    #define ROM_START 0xFF000000
    #define ROM_AREA  0x01000000

    memory_region_init_rom_device(&s->flash_mem, OBJECT(s), &xecuter_flash_ops,
                                  s, "xecuter.bios", ROM_AREA, &err);
    memory_region_rom_device_set_romd(&s->flash_mem, false);

    /* Alias the flash over the whole ROM window (0xFF000000 - 0xFFFFFFFF) */
    MemoryRegion *mr_bios = g_malloc(sizeof(MemoryRegion));
    memory_region_init_alias(mr_bios, NULL, "xecuter.bios.alias", &s->flash_mem,
                             0, ROM_AREA);
    memory_region_add_subregion(rom_memory__, ROM_START, mr_bios);

    /* MCPX boot ROM, overlaid on top of the flash at the very top of the ROM
     * window. The real part only drives the last 512 bytes, but the overlay
     * has to be page aligned to stay off the slow path, so the rest of the
     * page is primed with whatever the flash presents there - otherwise the
     * 2BL, which lives right below the boot ROM, would read back as zeroes.
     * The bank is fixed by the switches for as long as this overlay is
     * visible, so priming it once here stays correct.
     *
     * The subregion has to be named "xbox.mcpx": xbox_lpc_enable_mcpx_rom()
     * looks it up by that name to switch the boot ROM off once the 2BL is
     * done with it, which is why it is added directly rather than by alias. */
    unsigned int page_size = 4096;
    MemoryRegion *mr_mcpx = g_malloc(sizeof(MemoryRegion));
    memory_region_init_ram(mr_mcpx, NULL, "xbox.mcpx", page_size, &err);
    uint8_t *mcpx_data = memory_region_get_ram_ptr(mr_mcpx);

    unsigned int primed = have_bootrom ? page_size - MCPX_SIZE : page_size;
    for (unsigned int i = 0; i < primed; i++) {
        uint32_t pos;
        uint8_t *chip =
            xecuter_resolve(s, ROM_AREA - page_size + i, &pos);
        mcpx_data[i] = chip ? chip[pos] : 0xFF;
    }

    /* Without a boot ROM the whole page stays flash, so the CPU boots straight
     * from the image's own reset vector. */
    if (have_bootrom) {
        memcpy(mcpx_data + page_size - MCPX_SIZE, xecuter_mcpx_raw, MCPX_SIZE);
    }

    memory_region_add_subregion_overlap(rom_memory__, -page_size, mr_mcpx, 1);

    memory_region_init_io(&s->io, OBJECT(s), &xecuter_io_ops, s, "xecuter.io",
                          XECUTER_REGISTER_SIZE);
    isa_register_ioport(isa, &s->io, XECUTER_REGISTER_BASE);

    XFLOG("READY  main='%s' recovery='%s' tsop=%uK io=0x%04x-0x%04x"
          "  control=0x%02x status=0x%02x switches=%u bank=%u",
          s->rom_file, s->recovery_file, s->tsop_size / 1024,
          XECUTER_REGISTER_BASE,
          XECUTER_REGISTER_BASE + XECUTER_REGISTER_SIZE - 1, s->control,
          s->status, s->bank_switches, xecuter_get_bank(s));
}

static const Property xecuter_properties[] = {
    DEFINE_PROP_STRING("rom-path", XecuterState, rom_file),
    DEFINE_PROP_STRING("recovery-path", XecuterState, recovery_file),
    DEFINE_PROP_UINT8("bank-switches", XecuterState, bank_switches, 15),
};

static const VMStateDescription vmstate_xecuter = {
    .name = "modchip-xecuter",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void xecuter_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xecuter_realize;
    device_class_set_legacy_reset(dc, xecuter_reset);
    dc->vmsd = &vmstate_xecuter;
    device_class_set_props(dc, xecuter_properties);
}

static const TypeInfo xecuter_type_info = {
    .name          = "modchip-xecuter",
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(XecuterState),
    .class_init    = xecuter_class_init,
};

static void xecuter_register_types(void)
{
    type_register_static(&xecuter_type_info);
}

type_init(xecuter_register_types)
