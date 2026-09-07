/*
 * Xenium Modchip - https://github.com/Ryzee119/OpenXenium
 *
 * Copyright (c) 2021 Mike Davis
 * Copyright (c) 2021 Ryzee119
 *
 * Updated by EqUiNoX to implement the flash and banking.
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
#include "hw/loader.h"
#include "hw/char/serial.h"
#include "hw/isa/isa.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "qemu/notify.h"

/* smbus_xbox_smc.c: was the reset in progress requested as a warm reset? */
bool xbox_smc_take_warm_reset(void);

#define XENIUM_REGISTER_BASE 0xEE
#define XENIUM_REGISTER0 0
#define XENIUM_REGISTER1 1

/* Debug verbosity as a bitmask, so the two channels are independent:
 *   bit 0 (0x1) GENERAL - general modchip logging (DPRINTF): IO registers,
 *               memory mapping, per-access chatter. This is the "everything
 *               else" firehose.
 *   bit 1 (0x2) FLASH   - detailed flash-transaction logging (XFLOG): command
 *               decode, state transitions, program/erase, persistence. This is
 *               the channel for debugging guest BIOS flashing.
 * Combine as needed:  0x0 silent, 0x1 general only, 0x2 flash only, 0x3 both.
 * FLASH is routed to stderr and flushed so it survives the windowed build - run
 * xemu-modchip.exe from a console (or redirect 2> flash.log) to capture it. */
#define XENIUM_DEBUG_LEVEL 0x0

#if (XENIUM_DEBUG_LEVEL & 0x1)
# define DPRINTF(format, ...) printf(format, ## __VA_ARGS__)
#else
# define DPRINTF(format, ...) do { } while (0)
#endif

#if (XENIUM_DEBUG_LEVEL & 0x2)
# define XFLOG(fmt, ...) do { \
        fprintf(stderr, "[xenium-flash] " fmt "\n", ## __VA_ARGS__); \
        fflush(stderr); \
    } while (0)
#else
# define XFLOG(fmt, ...) do { } while (0)
#endif

#define XENIUM_FLASH_MANUF_ID (0x01)
#define XENIUM_FLASH_DEV_ID (0xC4)
#define XENIUM_FLASH_SIZE (2 * 1024 * 1024)
#define XENIUM_MAX_BANK_SIZE (1024 * 1024)

/* After the last flash write/erase, wait this long with no further activity
 * before persisting the image back to the ROM file. Debouncing coalesces the
 * thousands of byte-program cycles of a bank flash into a single disk write. */
#define XENIUM_FLASH_FLUSH_DELAY_MS 2000
#define MCPX_SIZE (512)

extern MemoryRegion *rom_memory__; //FIXME

/* True when the Xenium modchip is active. Set early by the machine setup (vl.c)
 * so the normal BIOS flash init is skipped, and also on device realize. Gates
 * the NV2A hooks (PRMCIO detection, PCRTC banking) so stock behaviour is used
 * when the modchip is disabled. */
bool xenium_enabled = false;

extern bool modchip_enabled;

uint8_t xenium_raw[XENIUM_FLASH_SIZE];
uint8_t mcpx_raw[MCPX_SIZE];

typedef struct XeniumBank {
    unsigned int offset;
    unsigned int size;
} XeniumBank_t;

static const XeniumBank_t XeniumBank[11] = 
{
    {0, 1 * 1024 * 1024},   //TSOP
    {0x180000, 256 * 1024}, //Bootloader
    {0x100000, 512 * 1024}, //XeniumOS
    {0x000000, 256 * 1024}, //Bank 1 256k
    {0x040000, 256 * 1024}, //Bank 2 256k
    {0x080000, 256 * 1024}, //Bank 3 256k
    {0x0C0000, 256 * 1024}, //Bank 4 256k
    {0x000000, 512 * 1024}, //Bank 1 512k
    {0x080000, 512 * 1024}, //Bank 2 512k
    {0x000000, 1024 * 1024}, //Bank 1 1M
    {0x1C0000, 256 * 1024}, //Recovery + More XeniumOS Data + User settings
};

// Dumped using this script https://gist.github.com/LoveMHz/8c20b0bb7fcd88588a1740657396075c
static const uint8_t XeniumFlashCFI[] = {
    /* 00h */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 10h */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 20h */ 0x51, 0x51, 0x52, 0x52, 0x59, 0x59, 0x02, 0x02, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,
    /* 30h */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0x27, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03,
    /* 40h */ 0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x15, 0x15,
    /* 50h */ 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40,
    /* 60h */ 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
    /* 70h */ 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* 80h */ 0x50, 0x50, 0x52, 0x52, 0x49, 0x49, 0x31, 0x31, 0x33, 0x33, 0x0C, 0x0C, 0x02, 0x02, 0x01, 0x01,
    /* 90h */ 0x01, 0x01, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03,
    /* A0h */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* B0h */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06,
    /* C0h */ 0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x05, 0x05, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x15, 0x15,
    /* D0h */ 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40,
    /* E0h */ 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
    /* F0h */ 0x00, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

typedef enum {
    XENIUM_MEMORY_STATE_NORMAL,
    XENIUM_MEMORY_STATE_CFI,
    XENIUM_MEMORY_STATE_AUTOSELECT,
    XENIUM_MEMORY_STATE_SECTOR_ERASE,
    XENIUM_MEMORY_STATE_WRITE,
} XeniumMemoryState;

G_GNUC_UNUSED static const char *xenium_state_name(XeniumMemoryState st)
{
    switch (st) {
        case XENIUM_MEMORY_STATE_NORMAL:       return "NORMAL";
        case XENIUM_MEMORY_STATE_CFI:          return "CFI";
        case XENIUM_MEMORY_STATE_AUTOSELECT:   return "AUTOSELECT";
        case XENIUM_MEMORY_STATE_SECTOR_ERASE: return "SECTOR_ERASE";
        case XENIUM_MEMORY_STATE_WRITE:        return "WRITE";
    }
    return "?";
}

typedef struct XeniumState {
    ISADevice dev;
    SysBusDevice dev_sysbus;
    MemoryRegion io;
    MemoryRegion flash_mem;

    // SPI
    bool sck;
    bool cs;
    bool mosi;
    bool miso_1;    // pin 1
    bool miso_4;    // pin 4

    unsigned char led;              // XXXXXBGR
    unsigned short bank_control;    // determines flash address mask

    bool recovery;  // 0 is active

    char *rom_file;
    XeniumMemoryState flash_state;
    unsigned char flash_cycle;

    // Deferred (debounced) write-back of the flash image to rom_file
    QEMUTimer *flush_timer;
    Notifier exit_notifier;
    bool flash_dirty;
} XeniumState;

#define XENIUM_DEVICE(obj) \
    OBJECT_CHECK(XeniumState, (obj), "modchip-xenium")

static void xenium_io_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned int size)
{
    XeniumState *s = opaque;

    DPRINTF("%s: Write 0x%llX to IO register 0x%llX\n",
        __func__, val, XENIUM_REGISTER_BASE + addr);

    switch(addr) {
        case XENIUM_REGISTER0:
            assert((val >> 3) == 0);    // un-known/used
            s->led = val;
            DPRINTF("%s: Set LED color(s) to %d\n", __func__, s->led);
            XFLOG("IO write reg0 (LED) = 0x%02x", (uint8_t)val);
        break;
        case XENIUM_REGISTER1: {
            assert((val & (1 << 7)) == 0);    // un-known/used
            s->sck = val & (1 << 6);
            s->cs = val & (1 << 5);
            s->mosi = val & (1 << 4);
            s->bank_control = val & 0xF;
            if (s->bank_control >= ARRAY_SIZE(XeniumBank)) {
                XFLOG("IO BANK 0x%x OUT OF RANGE (max %u) - clamping to 0",
                      s->bank_control, (unsigned)ARRAY_SIZE(XeniumBank) - 1);
                s->bank_control = 0;
            }
            G_GNUC_UNUSED unsigned int flash_off  = XeniumBank[s->bank_control].offset;
            G_GNUC_UNUSED unsigned int flash_size = XeniumBank[s->bank_control].size;
            DPRINTF("%s: Set Bank to %d, Offset: %08x, Size: %d bytes\n", __func__, s->bank_control,
                                                                          flash_off, flash_size);
            XFLOG("IO write reg1=0x%02x -> BANK=%u off=0x%06x size=%uK  sck=%d cs=%d mosi=%d",
                  (uint8_t)val, s->bank_control, flash_off, flash_size / 1024,
                  s->sck, s->cs, s->mosi);
        } break;
        default: assert(false);
    }
}

static uint64_t xenium_io_read(void *opaque, hwaddr addr, unsigned int size)
{
    XeniumState *s = opaque;
    uint32_t val = 0;

    switch(addr) {
        case XENIUM_REGISTER0:
            val = 0x55;     // genuine xenium!
        break;
        case XENIUM_REGISTER1:
            val = (s->recovery << 7) |
                (s->miso_1 << 5) |
                (s->miso_4 << 4) |
                s->bank_control;
        break;
        default: assert(false);
    }

    DPRINTF("%s: Read 0x%X from IO register 0x%llX\n",
        __func__, val, XENIUM_REGISTER_BASE + addr);

    XFLOG("IO read reg%u = 0x%02x  (recovery=%d miso1=%d miso4=%d bank=%u)",
          (unsigned)addr, val, s->recovery, s->miso_1, s->miso_4, s->bank_control);

    return val;
}

static const MemoryRegionOps xenium_io_ops = {
    .read  = xenium_io_read,
    .write = xenium_io_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

/* Erase the flash sector that contains absolute flash address `addr`, setting
 * every byte to 0xFF. The sector geometry mirrors the real Xenium flash (and
 * the flash device's sector map): uniform 64K sectors across the
 * main region, with smaller 32K/8K/16K boot sectors at the very top of the
 * 2MB device. Boundaries are all power-of-two aligned so a mask finds the
 * sector base. */
static void xenium_erase_sector(uint32_t addr)
{
    uint32_t sector_size;

    if (addr <= 0x1EFFFF) {
        sector_size = 64 * 1024;
    } else if (addr <= 0x1F7FFF) {
        sector_size = 32 * 1024;
    } else if (addr <= 0x1FBFFF) {
        sector_size = 8 * 1024;
    } else {
        sector_size = 16 * 1024;
    }

    uint32_t base = addr & ~(sector_size - 1);
    if (base >= XENIUM_FLASH_SIZE) {
        return;
    }
    if (base + sector_size > XENIUM_FLASH_SIZE) {
        sector_size = XENIUM_FLASH_SIZE - base;
    }

    DPRINTF("%s Erasing sector base=%08x size=%u (from addr %08x)\n",
        __func__, base, sector_size, addr);
    XFLOG("ERASE  sector base=0x%06x size=%uK  (target addr 0x%06x)",
          base, sector_size / 1024, addr);
    memset(&xenium_raw[base], 0xFF, sector_size);
}

/* Persist the in-memory flash image back to the backing ROM file. Invoked from
 * the debounce timer a short time after the last write/erase, and again from an
 * exit notifier so a flash issued just before quitting is not lost. */
static void xenium_flash_flush(XeniumState *s)
{
    if (!s->flash_dirty || s->rom_file == NULL || *s->rom_file == '\0') {
        return;
    }

    int fd = qemu_open(s->rom_file, O_WRONLY | O_BINARY, NULL);
    if (fd < 0) {
        fprintf(stderr, "xenium: could not open '%s' to persist flash\n", s->rom_file);
        return;
    }

    ssize_t written = write(fd, xenium_raw, XENIUM_FLASH_SIZE);
    close(fd);

    if (written != (ssize_t)XENIUM_FLASH_SIZE) {
        fprintf(stderr, "xenium: short write persisting flash to '%s' (%zd/%d)\n",
                s->rom_file, written, XENIUM_FLASH_SIZE);
        return;
    }

    s->flash_dirty = false;
    DPRINTF("%s Flash image persisted to '%s'\n", __func__, s->rom_file);
    XFLOG("PERSIST wrote %d bytes back to '%s'", XENIUM_FLASH_SIZE, s->rom_file);
}

static void xenium_flash_flush_cb(void *opaque)
{
    xenium_flash_flush((XeniumState *)opaque);
}

static void xenium_flash_exit_notify(Notifier *n, void *opaque)
{
    XeniumState *s = container_of(n, XeniumState, exit_notifier);
    xenium_flash_flush(s);
}

/* Mark the flash image modified and (re)arm the debounce timer. */
static void xenium_flash_mark_dirty(XeniumState *s)
{
    s->flash_dirty = true;
    if (s->flush_timer) {
        timer_mod(s->flush_timer,
            qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + XENIUM_FLASH_FLUSH_DELAY_MS);
    }
}

static uint64_t flash_read(void *opaque, hwaddr offset, unsigned size)
{
    XeniumState *s = opaque;
    if(s->flash_state == XENIUM_MEMORY_STATE_NORMAL) {
        //Handle mirroring
        offset %= XeniumBank[s->bank_control].size;
        //Handle banking
        offset |= XeniumBank[s->bank_control].offset;
        if (size == 1) {
            uint8_t *flash_mem = (uint8_t *)xenium_raw;
            return flash_mem[offset];
        }
        else if (size == 2) {
            uint16_t *flash_mem = (uint16_t *)xenium_raw;
            return flash_mem[offset/2];
        }
        else if (size == 4) {
            uint32_t *flash_mem = (uint32_t *)xenium_raw;
            return flash_mem[offset/4];
        }
        else {
            DPRINTF("%s Unsupported read len %d\n", __FUNCTION__, size);
            assert(0);
        }
    }

    DPRINTF("%s offset: %08x size: %d\n", __FUNCTION__, (uint32_t)offset, size);
    XFLOG("READ   state=%s off=0x%06x size=%u", xenium_state_name(s->flash_state),
          (uint32_t)offset, size);

    if(s->flash_state == XENIUM_MEMORY_STATE_CFI) {
        uint8_t cfi = XeniumFlashCFI[(size == 1 ? offset : offset << 1) % sizeof(XeniumFlashCFI)];
        XFLOG("READ   CFI[0x%02x] -> 0x%02x", (uint32_t)offset, cfi);
        return cfi;
    }

    if(s->flash_state == XENIUM_MEMORY_STATE_AUTOSELECT) {
        switch (offset) {
            case 0:
                DPRINTF("%s Sending Manufacturer ID %02x\n", __FUNCTION__, XENIUM_FLASH_MANUF_ID);
                XFLOG("READ   AUTOSELECT manufacturer id -> 0x%02x", XENIUM_FLASH_MANUF_ID);
                return XENIUM_FLASH_MANUF_ID;
            // The flash driver reads the device ID at byte offset 1
            // (manuf=lpcMemMap[0]; devid=lpcMemMap[1]); offset 2 is accepted
            // too for the x16-style autoselect convention.
            case 1:
            case 2:
                DPRINTF("%s Sending Device ID %02x\n", __FUNCTION__, XENIUM_FLASH_DEV_ID);
                XFLOG("READ   AUTOSELECT device id (off %u) -> 0x%02x", (uint32_t)offset, XENIUM_FLASH_DEV_ID);
                return XENIUM_FLASH_DEV_ID;
        }
        DPRINTF("%s Invalid Chip ID offset: %08x\n", __FUNCTION__, (uint32_t)offset);
        XFLOG("READ   AUTOSELECT unknown offset 0x%06x -> 0x00", (uint32_t)offset);
    }

    return 0;
}

static void flash_write(void *opaque, hwaddr offset, uint64_t value,
        unsigned int size)
{
    XeniumState *s = opaque;

    DPRINTF("%s offset: %08x value: %02x size: %d, cycle: %d\n", __FUNCTION__, (uint32_t)offset, (uint8_t)value, size, s->flash_cycle);
    XFLOG("WRITE  off=0x%06x val=0x%02x size=%u  [state=%s cycle=%u bank=%u]",
          (uint32_t)offset, (uint8_t)value, size,
          xenium_state_name(s->flash_state), s->flash_cycle, s->bank_control);

    // A data-program cycle takes priority over command decoding: while in the
    // WRITE state the next access programs a flash byte and must not be
    // re-parsed as a command. In particular data byte 0xF0 written to offset 0
    // is real flash data here, not a reset - checking reset first would silently
    // drop that byte and hang the guest's program-verify poll.
    if (s->flash_state == XENIUM_MEMORY_STATE_WRITE)
    {
        DPRINTF("%s Flash Write offset = %08x, value %02x\n", __FUNCTION__, (uint32_t)offset, (uint8_t)value);

        //Handle mirroring
        offset %= XeniumBank[s->bank_control].size;
        //Handle banking
        offset |= XeniumBank[s->bank_control].offset;
        if (size == 1) {
            uint8_t *flash_mem = (uint8_t *)xenium_raw;
            G_GNUC_UNUSED uint8_t old = flash_mem[offset];
            flash_mem[offset] = (uint8_t)value;
            XFLOG("PROGRAM abs=0x%06x  0x%02x -> 0x%02x%s", (uint32_t)offset, old, (uint8_t)value,
                  (~old & (uint8_t)value) ? "   (sets 1 bits - real HW needs prior erase)" : "");
        }
        else if (size == 2) {
            uint16_t *flash_mem = (uint16_t *)xenium_raw;
            flash_mem[offset/2] = (uint16_t)value;
            XFLOG("PROGRAM abs=0x%06x  16-bit 0x%04x", (uint32_t)offset, (uint16_t)value);
        }
        else if (size == 4) {
            uint32_t *flash_mem = (uint32_t *)xenium_raw;
            flash_mem[offset/4] = (uint32_t)value;
            XFLOG("PROGRAM abs=0x%06x  32-bit 0x%08x", (uint32_t)offset, (uint32_t)value);
        }
        else {
            DPRINTF("%s Unsupported write len %d\n", __FUNCTION__, size);
            assert(0);
        }

        xenium_flash_mark_dirty(s);

        s->flash_state = XENIUM_MEMORY_STATE_NORMAL;
        s->flash_cycle = 1;
        return;
    }

    // Reset (exits CFI / Autoselect and aborts an in-progress command sequence)
    if(offset == 0x00 && value == 0xF0 && size == 1) {
        DPRINTF("%s Flash Reset (Entering Normal flash state)\n", __FUNCTION__);
        XFLOG("CMD    reset (F0) -> NORMAL");

        s->flash_state = XENIUM_MEMORY_STATE_NORMAL;
        s->flash_cycle = 1;
        return;
    }

    // Command / unlock addresses are decoded on the low 12 bits only, so both
    // the 16-bit (0xAAAA / 0x5555) and short 12-bit (0xAAA / 0x555) unlock
    // address conventions are accepted (0xAAAA & 0xFFF == 0xAAA, etc.).
    hwaddr cmd = offset & 0xFFF;

    switch (s->flash_cycle)
    {
        case 1:
            // Enter CFI Mode
            if(cmd == 0xAA && value == 0x98 && size == 1) {
                DPRINTF("%s Entering CFI Mode flash state\n", __FUNCTION__);
                XFLOG("CMD    CFI query (98) -> CFI state");

                s->flash_state = XENIUM_MEMORY_STATE_CFI;
            }
            else if(cmd == 0xAAA && value == 0xAA && size == 1) {
                XFLOG("CMD    unlock 1/2 ok (AA@AAAA) -> cycle 2");
                s->flash_cycle++;
            }
            else {
                DPRINTF("%s Unimplemented Flash command\n", __FUNCTION__);
                XFLOG("CMD    UNEXPECTED at cycle 1: val=0x%02x off=0x%06x size=%u"
                      " (want AA@0xAAA/0xAAAA or 98@0xAA)", (uint8_t)value, (uint32_t)offset, size);
            }
            break;
        case 2:
            if(cmd == 0x555 && value == 0x55 && size == 1) {
                XFLOG("CMD    unlock 2/2 ok (55@5555) -> cycle 3");
                s->flash_cycle++;
            }
            else {
                DPRINTF("%s Unimplemented Flash command\n", __FUNCTION__);
                XFLOG("CMD    UNEXPECTED at cycle 2: val=0x%02x off=0x%06x size=%u"
                      " (want 55@0x555/0x5555) - resetting sequence", (uint8_t)value, (uint32_t)offset, size);
                s->flash_cycle = 1;
            }
            break;
        case 3:
            if(cmd == 0xAAA && value == 0x80 && size == 1) {
                XFLOG("CMD    erase setup (80) -> cycle 4");
                s->flash_cycle++;
            }
            else if(cmd == 0xAAA && value == 0x90 && size == 1) {
                DPRINTF("%s Entering Autoselect Mode flash state\n", __FUNCTION__);
                XFLOG("CMD    autoselect (90) -> AUTOSELECT state");

                s->flash_state = XENIUM_MEMORY_STATE_AUTOSELECT;
            }
            else if(cmd == 0xAAA && value == 0xA0 && size == 1) {
                DPRINTF("%s Entering flash write state\n", __FUNCTION__);
                XFLOG("CMD    program (A0) -> WRITE state (next access is the data byte)");

                s->flash_state = XENIUM_MEMORY_STATE_WRITE;
            }
            else {
                DPRINTF("%s Unimplemented Flash command\n", __FUNCTION__);
                XFLOG("CMD    UNEXPECTED at cycle 3: val=0x%02x off=0x%06x size=%u"
                      " (want 80/90/A0@0xAAA/0xAAAA) - resetting sequence",
                      (uint8_t)value, (uint32_t)offset, size);
                s->flash_cycle = 1;
            }
            break;
        case 4:
            if(cmd == 0xAAA && value == 0xAA && size == 1) {
                XFLOG("CMD    erase unlock 1/2 ok (AA@AAAA) -> cycle 5");
                s->flash_cycle++;
            }
            else {
                XFLOG("CMD    UNEXPECTED at cycle 4: val=0x%02x off=0x%06x size=%u"
                      " (want AA@0xAAA/0xAAAA) - resetting sequence", (uint8_t)value, (uint32_t)offset, size);
                s->flash_cycle = 1;
            }
            break;
        case 5:
            if(cmd == 0x555 && value == 0x55 && size == 1) {
                XFLOG("CMD    erase unlock 2/2 ok (55@5555) -> cycle 6 (await 0x30 confirm)");
                s->flash_cycle++;
            }
            else {
                XFLOG("CMD    UNEXPECTED at cycle 5: val=0x%02x off=0x%06x size=%u"
                      " (want 55@0x555/0x5555) - resetting sequence", (uint8_t)value, (uint32_t)offset, size);
                s->flash_cycle = 1;
            }
            break;
        case 6:
            if(value == 0x30 && size == 1) {
                //Handle mirroring + banking to resolve the absolute flash address
                hwaddr sector = offset;
                sector %= XeniumBank[s->bank_control].size;
                sector |= XeniumBank[s->bank_control].offset;

                DPRINTF("%s Sector Erase, bank-rel %04llx -> abs %08x\n", __FUNCTION__,
                        (unsigned long long)offset, (uint32_t)sector);
                XFLOG("CMD    sector-erase confirm (30) bank-rel 0x%06x -> abs 0x%06x",
                      (uint32_t)offset, (uint32_t)sector);

                xenium_erase_sector((uint32_t)sector);
                xenium_flash_mark_dirty(s);

                // Erase completes immediately; return to normal so the guest's
                // toggle / read-back poll sees the erased (0xFF) sector and exits.
                s->flash_state = XENIUM_MEMORY_STATE_NORMAL;
                s->flash_cycle = 1;
            }
            else {
                XFLOG("CMD    UNEXPECTED at cycle 6: val=0x%02x off=0x%06x size=%u"
                      " (want 30 confirm) - resetting sequence", (uint8_t)value, (uint32_t)offset, size);
                s->flash_cycle = 1;
            }
            break;
    }
}

static const MemoryRegionOps xenium_flash_ops = {
    .read = flash_read,
    .write = flash_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* The bank register survives a warm reset (that is how a slot is booted);
 * a power cycle, or a reset from the host UI, comes up on the bootloader. */
static void xenium_reset(DeviceState *dev)
{
    XeniumState *s = XENIUM_DEVICE(dev);

    if (!xbox_smc_take_warm_reset()) {
        s->bank_control = 1;
        s->recovery = 1;
        s->led = 1;
    }
    s->flash_state = XENIUM_MEMORY_STATE_NORMAL;
    s->flash_cycle = 1;
}

static void xenium_realize(DeviceState *dev, Error **errp)
{
    XeniumState *s = XENIUM_DEVICE(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    Error *err = NULL;

    xenium_enabled = true;
    modchip_enabled = true;

    //Read Xenium Flash Dump (2MB file)
    int fd = qemu_open(s->rom_file, O_RDONLY | O_BINARY, NULL);
    assert(fd >= 0);
    read(fd, xenium_raw, XENIUM_FLASH_SIZE);
    close(fd);

    //Read MCPX Dump (512 bytes)
    const char *bootrom_file =
        object_property_get_str(qdev_get_machine(), "bootrom", NULL);
    bool have_bootrom = false;

    if ((bootrom_file != NULL) && *bootrom_file) {
        char *filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bootrom_file);
        assert(filename);

        /* Read in MCPX ROM over last 512 bytes of BIOS data */
        int fd = qemu_open(filename, O_RDONLY | O_BINARY, NULL);
        assert(fd >= 0);
        have_bootrom = read(fd, mcpx_raw, MCPX_SIZE) == MCPX_SIZE;
        close(fd);
        g_free(filename);
    }

    // default state
    s->bank_control = 1;                         // bootloader
    s->recovery = 1;                             // inactive
    s->led = 1;                                  // red
    s->flash_state = XENIUM_MEMORY_STATE_NORMAL; // Default flash state
    s->flash_cycle = 1;                          // Flash command cycle tracker

    // Deferred flash persistence: a debounce timer writes the image back to the
    // ROM file ~2s after the last write, and an exit notifier flushes any
    // pending changes when xemu quits.
    s->flash_dirty = false;
    s->flush_timer = timer_new_ms(QEMU_CLOCK_REALTIME, xenium_flash_flush_cb, s);
    s->exit_notifier.notify = xenium_flash_exit_notify;
    qemu_add_exit_notifier(&s->exit_notifier);

    #define ROM_START 0xFF000000
    #define ROM_AREA  0x01000000

    memory_region_init_rom_device(&s->flash_mem, OBJECT(s), &xenium_flash_ops, s, "xenium.bios", ROM_AREA, &err);
    memory_region_rom_device_set_romd(&s->flash_mem, false);

    //Setup memory aliases over the entire Flash ROM mapped region. (0xFF000000 to 0xFFFFFFFF)
    MemoryRegion *mr_bios = g_malloc(sizeof(MemoryRegion));
    assert(mr_bios != NULL);
    memory_region_init_alias(mr_bios, NULL, "xenium.bios.alias", &s->flash_mem, 0, ROM_AREA);
    memory_region_add_subregion(rom_memory__, ROM_START, mr_bios);

    /* MCPX boot ROM, overlaid on the flash at the top of the ROM window. The
     * real part only drives the last 512 bytes, but the overlay has to be page
     * aligned to stay off the slow path, so the rest of the page is primed
     * with what the flash presents there - otherwise the 2BL, which sits right
     * below the boot ROM, reads back as zeroes. The bank does not change while
     * this overlay is visible, so priming it once here stays correct.
     *
     * The subregion has to be named "xbox.mcpx": xbox_lpc_enable_mcpx_rom()
     * looks it up by that name to switch the boot ROM off once the 2BL is done
     * with it, which is why it is added directly rather than by alias. */
    unsigned int page_size = 4096;
    MemoryRegion *mr_mcpx = g_malloc(sizeof(MemoryRegion));
    memory_region_init_ram(mr_mcpx, NULL, "xbox.mcpx", page_size, &err);
    uint8_t *mcpx_data = memory_region_get_ram_ptr(mr_mcpx);

    unsigned int primed = have_bootrom ? page_size - MCPX_SIZE : page_size;
    for (unsigned int i = 0; i < primed; i++) {
        hwaddr off = ROM_AREA - page_size + i;
        off %= XeniumBank[s->bank_control].size;
        off |= XeniumBank[s->bank_control].offset;
        mcpx_data[i] = xenium_raw[off];
    }

    /* Without a boot ROM the whole page stays flash, so the CPU boots straight
     * from the image's own reset vector. */
    if (have_bootrom) {
        memcpy(mcpx_data + page_size - MCPX_SIZE, mcpx_raw, MCPX_SIZE);
    }

    memory_region_add_subregion_overlap(rom_memory__, -page_size, mr_mcpx, 1);

    //Register Xenium Chip IO
    memory_region_init_io(&s->io, OBJECT(s), &xenium_io_ops, s, "xenium.io", 2);   // 0xEE & 0xEF
    isa_register_ioport(isa, &s->io, XENIUM_REGISTER_BASE);
}

static const Property xenium_properties[] = {
    DEFINE_PROP_STRING("rom-path", XeniumState, rom_file),
};

static const VMStateDescription vmstate_xenium = {
    .name = "modchip-xenium",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void xenium_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = xenium_realize;
    device_class_set_legacy_reset(dc, xenium_reset);
    dc->vmsd = &vmstate_xenium;
    device_class_set_props(dc, xenium_properties);
}

static const TypeInfo xenium_type_info = {
    .name          = "modchip-xenium",
    .parent        = TYPE_ISA_DEVICE,
    .instance_size = sizeof(XeniumState),
    .class_init    = xenium_class_init,
};

static void xenium_register_types(void)
{
    type_register_static(&xenium_type_info);
}

type_init(xenium_register_types)